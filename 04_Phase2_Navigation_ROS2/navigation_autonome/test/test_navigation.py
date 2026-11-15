"""Tests sans ROS : conversion de carte, contrôleur et boucle complète simulée.

Lancer depuis le dossier du paquet :  python -m pytest test
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from navigation_autonome.algos import astar, lisser  # noqa: E402
from navigation_autonome.algos.grille import INCONNU, LIBRE, OBSTACLE, carte_murs  # noqa: E402
from navigation_autonome.carte_ros import chemin_en_metres, grille_depuis_occupancy  # noqa: E402
from navigation_autonome.controle import PurePursuit, angle_normalise, obstacle_devant  # noqa: E402


# --------------------------------------------------------------- carte ROS

def test_conversion_occupancy_seuils_et_orientation():
    # Carte 3 colonnes × 2 lignes ; data commence en bas à gauche, ligne par ligne.
    data = [0, 30, 65,
            -1, 100, 10]
    g = grille_depuis_occupancy(3, 2, 0.05, -1.0, -2.0, data)
    assert g.donnees[0].tolist() == [LIBRE, LIBRE, OBSTACLE]
    assert g.donnees[1].tolist() == [INCONNU, OBSTACLE, LIBRE]
    # La case (ligne 0, colonne 2) est à x = -1 + 2.5 × 0.05, y = -2 + 0.5 × 0.05
    x, y = g.case_vers_metres((0, 2))
    assert math.isclose(x, -0.875) and math.isclose(y, -1.975)


def test_inconnu_libre():
    g = grille_depuis_occupancy(2, 1, 0.05, 0, 0, [-1, 0], inconnu_libre=True)
    assert g.est_libre((0, 0))


# --------------------------------------------------------------- contrôleur

def test_angle_normalise():
    assert math.isclose(angle_normalise(3 * math.pi / 2), -math.pi / 2)


def test_ligne_droite_avance_sans_tourner():
    pp = PurePursuit()
    pp.nouveau_chemin([(0, 0), (0.5, 0), (1.0, 0)])
    c = pp.commande(0.0, 0.0, 0.0)
    assert c.v > 0 and abs(c.w) < 1e-9 and not c.arrive


def test_but_derriere_tourne_sur_place():
    pp = PurePursuit()
    pp.nouveau_chemin([(0, 0), (-1.0, 0)])
    c = pp.commande(0.0, 0.0, 0.0)
    assert c.v == 0 and abs(c.w) > 0


def test_cible_a_gauche_tourne_a_gauche():
    pp = PurePursuit()
    pp.nouveau_chemin([(0, 0), (0.3, 0.1), (1.0, 0.5)])
    assert pp.commande(0.0, 0.0, 0.0).w > 0


def test_arrivee():
    pp = PurePursuit()
    pp.nouveau_chemin([(0, 0), (1.0, 0)])
    assert pp.commande(0.97, 0.0, 0.0).arrive


def test_limites_de_vitesse_respectees():
    pp = PurePursuit(v_max=0.15, w_max=1.5)
    pp.nouveau_chemin([(0, 0), (0.2, 0.15), (0.3, 0.5)])
    c = pp.commande(0.0, 0.0, 0.0)
    assert 0 <= c.v <= 0.15 and abs(c.w) <= 1.5


def test_obstacle_devant_cone_avant_uniquement():
    n = 360
    inc = 2 * math.pi / n
    vide = [math.inf] * n
    devant = list(vide); devant[5] = 0.15          # 5° à gauche, 15 cm
    cote = list(vide); cote[90] = 0.10             # 90° : sur le côté
    loin = list(vide); loin[0] = 0.50
    assert obstacle_devant(devant, 0.0, inc, 0.20)
    assert not obstacle_devant(cote, 0.0, inc, 0.20)
    assert not obstacle_devant(loin, 0.0, inc, 0.20)
    assert not obstacle_devant([0.0] * n, 0.0, inc, 0.20)   # 0 = pas de mesure


# --------------------------------------------------------------- boucle complète

@pytest.mark.parametrize("lissage", [True, False])
def test_robot_simule_atteint_le_but_sans_toucher_de_mur(lissage):
    """A* + Pure Pursuit sur un robot simulé (modèle unicycle du Burger)."""
    carte = carte_murs()                               # 60 × 80 cases de 5 cm = 3 m × 4 m
    grille = carte.gonfler(0.15)
    depart, arrivee = grille.case_libre_proche((5, 5)), grille.case_libre_proche((54, 74))
    res = astar(grille, depart, arrivee)
    assert res.trouve
    chemin = lisser(grille, res.chemin) if lissage else res.chemin
    points = chemin_en_metres(grille, chemin)

    pp = PurePursuit()
    pp.nouveau_chemin(points)
    x, y = points[0]
    cap, dt = 0.0, 0.05
    for _ in range(int(120 / dt)):                     # 2 minutes simulées au maximum
        c = pp.commande(x, y, cap)
        if c.arrive:
            break
        x += c.v * math.cos(cap) * dt
        y += c.v * math.sin(cap) * dt
        cap += c.w * dt
        # Le centre du robot ne doit jamais entrer dans un vrai mur (carte non gonflée).
        assert carte.est_libre(carte.metres_vers_case(x, y)), f"collision en ({x:.2f}, {y:.2f})"
    assert c.arrive, "le robot n'est pas arrivé en 2 minutes"
    assert math.dist((x, y), points[-1]) < 0.08
