import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from algos import CARTES, Grille, astar, dijkstra, lisser, rrt  # noqa: E402
from algos.grille import OBSTACLE, carte_labyrinthe, carte_murs, carte_vide, extremites  # noqa: E402
from algos.rrt import segment_libre  # noqa: E402


def chemin_valide(grille, chemin, depart, arrivee):
    """Le chemin relie départ et arrivée, sans traverser d'obstacle."""
    assert tuple(map(round, chemin[0])) == depart
    assert tuple(map(round, chemin[-1])) == arrivee
    for a, b in zip(chemin, chemin[1:]):
        assert segment_libre(grille, a, b)


@pytest.mark.parametrize("nom_carte", list(CARTES))
def test_astar_et_dijkstra_meme_cout_optimal(nom_carte):
    g = CARTES[nom_carte]()
    d, a = extremites(g)
    rd, ra = dijkstra(g, d, a), astar(g, d, a)
    assert rd.trouve and ra.trouve
    assert math.isclose(rd.cout, ra.cout)
    assert ra.noeuds_explores <= rd.noeuds_explores
    chemin_valide(g, ra.chemin, d, a)


def test_ligne_droite_sur_carte_vide():
    g = carte_vide(10, 10)
    r = astar(g, (1, 1), (1, 8))
    assert r.cout == 7
    assert len(r.chemin) == 8


def test_diagonale_coute_racine_de_deux():
    g = carte_vide(10, 10)
    r = astar(g, (1, 1), (8, 8))
    assert math.isclose(r.cout, 7 * math.sqrt(2))


def test_aucun_chemin_si_arrivee_enfermee():
    g = carte_vide(20, 20).donnees.copy()
    g[8:13, 8] = g[8:13, 12] = g[8, 8:13] = g[12, 8:13] = OBSTACLE
    g = Grille(g)
    for algo in (astar, dijkstra):
        assert not algo(g, (1, 1), (10, 10)).trouve
    assert not rrt(g, (1, 1), (10, 10), iterations_max=2000).trouve


def test_depart_dans_un_obstacle():
    g = carte_murs()
    mur = (5, g.largeur // 4)
    assert not astar(g, mur, (10, 10)).trouve


def test_pas_de_coupe_de_coin():
    g = np.zeros((5, 5), dtype=np.int16)
    g[1, 2] = g[2, 1] = OBSTACLE
    r = astar(Grille(g), (1, 1), (2, 2))
    assert r.cout > math.sqrt(2)


@pytest.mark.parametrize("graine", range(5))
def test_rrt_trouve_un_chemin_valide(graine):
    g = carte_murs()
    d, a = extremites(g)
    r = rrt(g, d, a, graine=graine)
    assert r.trouve
    chemin_valide(g, r.chemin, d, a)


def test_lissage_raccourcit_sans_traverser():
    g = carte_murs()
    d, a = extremites(g)
    r = rrt(g, d, a, graine=1)
    lisse = lisser(g, r.chemin)
    chemin_valide(g, lisse, d, a)
    assert len(lisse) <= len(r.chemin)


def test_gonflement_agrandit_les_obstacles():
    g = carte_vide(20, 20).donnees.copy()
    g[10, 10] = OBSTACLE
    g = Grille(g, resolution=0.05)
    gonflee = g.gonfler(0.105)      # rayon du Burger -> 3 cases
    assert not gonflee.est_libre((10, 13))
    assert gonflee.est_libre((10, 14))
    assert not gonflee.est_libre((1, 1))   # le mur extérieur est gonflé aussi


def test_conversion_metres_aller_retour():
    g = Grille(np.zeros((40, 40)), resolution=0.05, origine=(-1.0, -1.0))
    case = (12, 30)
    assert g.metres_vers_case(*g.case_vers_metres(case)) == case


def test_rrt_echoue_dans_les_passages_etroits_mais_pas_astar():
    g = carte_labyrinthe(couloir=1)
    d, a = extremites(g)
    assert astar(g, d, a).trouve
    assert not rrt(g, d, a, iterations_max=3000).trouve
