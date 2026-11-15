"""Cartes nommées, carte « présentable », formats ROS et mode slam_toolbox (sans ROS)."""

import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cartographie import INCONNU_UI, LIBRE_UI, OCCUPE_UI, Carte, composer  # noqa: E402
from cerveau import Cerveau, nom_de_carte  # noqa: E402


def grille_ros_salle(res=0.05, ox=-1.5, oy=-2.5, w=120, h=100):
    """OccupancyGrid factice : pièce 5 m x 4 m (murs à x=-1, 4 et y=-2, 2), inconnu autour."""
    g = np.full((h, w), -1, dtype=np.int16)
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    dedans = (X > -1.0) & (X < 4.0) & (Y > -2.0) & (Y < 2.0)
    g[dedans] = 0
    mur = (((np.abs(X + 1.0) < 0.05) | (np.abs(X - 4.0) < 0.05)) & (Y > -2.05) & (Y < 2.05)) | \
          (((np.abs(Y + 2.0) < 0.05) | (np.abs(Y - 2.0) < 0.05)) & (X > -1.05) & (X < 4.05))
    g[mur] = 100
    return g


def test_nom_de_carte():
    assert nom_de_carte("Salle 330 !") == "Salle_330"
    assert nom_de_carte("../../etc/passwd") == "etc_passwd"
    assert nom_de_carte("   ") == ""


def test_fusion_grille_ros_identite_et_decalee():
    g = grille_ros_salle()
    c = Carte()
    c.fusionner_grille(0.05, (-1.5, -2.5, 0.0), g.shape[1], g.shape[0], g.ravel())
    e = c.etats()
    l, k = c.vers_case(1.5, 0.0)
    assert e[l, k] == LIBRE_UI
    l, k = c.vers_case(4.0, 0.0)
    assert e[l, k] == OCCUPE_UI
    l, k = c.vers_case(6.0, 0.0)
    assert e[l, k] == INCONNU_UI
    # Repère de slam_toolbox tourné de 90° et décalé : le mur x=4 du repère slam se retrouve
    # en y = 1 + 4 = 5 dans notre repère (rotation +90° : (x, y) -> (-y, x), puis +(0, 1))
    c2 = Carte()
    c2.fusionner_grille(0.05, (-1.5, -2.5, 0.0), g.shape[1], g.shape[0], g.ravel(),
                        decalage=(0.0, 1.0, math.pi / 2))
    e2 = c2.etats()
    l, k = c2.vers_case(0.0, 5.0)
    assert e2[l, k] == OCCUPE_UI
    l, k = c2.vers_case(0.0, 2.5)
    assert e2[l, k] == LIBRE_UI


def test_fusion_garde_l_ancienne_carte_hors_de_la_nouvelle():
    c = Carte()
    c.lo[:] = 0
    l, k = c.vers_case(-3.0, 0.0)
    c.lo[l, k] = c.l_max                      # mur connu avant, hors de la nouvelle grille
    g = grille_ros_salle()
    c.fusionner_grille(0.05, (-1.5, -2.5, 0.0), g.shape[1], g.shape[0], g.ravel())
    assert c.etats()[l, k] == OCCUPE_UI


def test_etats_propres_enleve_le_bruit_sans_toucher_la_planification():
    c = Carte()
    g = grille_ros_salle()
    c.fusionner_grille(0.05, (-1.5, -2.5, 0.0), g.shape[1], g.shape[0], g.ravel())
    l, k = c.vers_case(1.5, 0.5)
    c.lo[l, k] = c.l_max                      # point de bruit isolé au milieu de la pièce
    l2, k2 = c.vers_case(4.0, 1.0)
    c.lo[l2, k2] = 0.0                        # trou d'une case dans le mur x = 4
    propre = c.etats_propres()
    assert propre[l, k] == LIBRE_UI and propre[l2, k2] == OCCUPE_UI
    assert c.etats()[l, k] == OCCUPE_UI       # la carte brute (planification) garde l'obstacle


def test_export_pgm_yaml():
    c = Carte()
    g = grille_ros_salle()
    c.fusionner_grille(0.05, (-1.5, -2.5, 0.0), g.shape[1], g.shape[0], g.ravel())
    with tempfile.TemporaryDirectory() as d:
        base = os.path.join(d, "salle")
        c.exporter_pgm(base, "salle")
        with open(base + ".pgm", "rb") as f:
            assert f.readline() == b"P5\n"
            f.readline()
            w, h = (int(v) for v in f.readline().split())
            assert f.readline() == b"255\n"
            img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
        assert set(np.unique(img)) <= {0, 205, 254} and (img == 0).any() and (img == 254).any()
        yaml = open(base + ".yaml", encoding="utf-8").read()
        assert "image: salle.pgm" in yaml and "resolution: 0.05" in yaml


def test_cartes_nommees_enregistrer_lister_charger():
    with tempfile.TemporaryDirectory() as d:
        c = Cerveau(mode="test")
        c.fichier_carte = os.path.join(d, "carte_robot.npz")
        c.recevoir_odom(0, 0, 0, 0, 0)
        g = grille_ros_salle()
        c.carte.fusionner_grille(0.05, (-1.5, -2.5, 0.0), g.shape[1], g.shape[0], g.ravel())
        c.lieux["Porte"] = (3.5, -1.5)
        ok, _ = c.enregistrer_carte(nom="Salle 330")
        assert ok and os.path.isfile(os.path.join(d, "Salle_330.npz"))
        assert os.path.isfile(os.path.join(d, "Salle_330.pgm"))
        assert open(os.path.join(d, "derniere_carte.txt")).read() == "Salle_330.npz"
        c.effacer_carte()
        ok, _ = c.enregistrer_carte(nom="Couloir")         # carte vide -> refus
        assert not ok
        liste = c.liste_cartes()
        assert [x["nom"] for x in liste] == ["Salle_330"] and liste[0]["lieux"] == 1
        assert liste[0]["surface_m2"] > 15
        # Un autre jour : on recharge la carte par son nom, le robot cherche sa position
        c2 = Cerveau(mode="test")
        c2.fichier_carte = os.path.join(d, "carte_robot.npz")
        ok, _ = c2.charger_carte_nommee("Salle_330")
        assert ok and not c2.localise and c2.lieux == {"Porte": (3.5, -1.5)}
        assert c2.charger_carte_nommee("inexistante") == (False, "carte introuvable")


def test_mode_slam_sans_ros():
    """Le cerveau suit slam_toolbox : TF map->odom = correction, /map recopiée."""
    c = Cerveau(mode="test")
    c.activer_slam()
    c.demarrer_slam()
    assert c.prendre_demandes_slam() == [("lancer", None, None)]
    c.recevoir_odom(1.0, 0.0, 0.0, 0, 0)
    c.recevoir_tf_slam(0.0, 0.0, 0.0)                    # 1er TF : alignement des repères
    c.recevoir_tf_slam(0.10, -0.05, 0.02)                # slam_toolbox corrige la dérive
    p = c.pose()
    assert abs(p[0] - (0.10 + math.cos(0.02))) < 1e-6 and abs(p[2] - 0.02) < 1e-9
    g = grille_ros_salle()
    c.recevoir_carte_slam(0.05, -1.5, -2.5, 0.0, g.shape[1], g.shape[0], g.ravel())
    assert c.slam["cartes_recues"] == 1
    l, k = c.carte.vers_case(4.0, 0.0)
    assert c.carte.etats()[l, k] == OCCUPE_UI
    # Pas d'intégration maison en mode slam
    v0 = c.carte.version
    c.recevoir_scan(0.0, 2 * math.pi / 360, 0.1, 12.0, [1.0] * 360)
    c.cycle()
    assert c.carte.version == v0


def test_mode_slam_rechargement_reprend_le_graphe():
    with tempfile.TemporaryDirectory() as d:
        c = Cerveau(mode="test")
        c.fichier_carte = os.path.join(d, "carte_robot.npz")
        c.activer_slam()
        c.demarrer_slam()
        c.recevoir_odom(0, 0, 0, 0, 0)
        c.recevoir_tf_slam(0.0, 0.0, 0.0)
        g = grille_ros_salle()
        c.recevoir_carte_slam(0.05, -1.5, -2.5, 0.0, g.shape[1], g.shape[0], g.ravel())
        c.prendre_demandes_slam()
        ok, _ = c.enregistrer_carte(nom="Salle")
        assert ok and ("serialiser", os.path.join(d, "Salle")) in c.prendre_demandes_slam()
        open(os.path.join(d, "Salle.posegraph"), "w").close()   # fichier écrit par slam_toolbox
        # Un autre jour
        c2 = Cerveau(mode="test")
        c2.fichier_carte = os.path.join(d, "carte_robot.npz")
        c2.activer_slam()
        c2.charger_carte(os.path.join(d, "Salle.npz"))
        assert ("arreter",) in c2.prendre_demandes_slam() and not c2.localise
        # La station retrouve le robot en (1, 0.5, 0.3) -> slam_toolbox relancé sur le graphe, à cette pose
        c2.recevoir_odom(0, 0, 0, 0, 0)
        c2.carte.correction = (1.0, 0.5, 0.3)
        c2.localise = True
        c2._relancer_slam_apres_localisation()
        d2 = c2.prendre_demandes_slam()
        assert d2[0][0] == "lancer" and d2[0][1] == os.path.join(d, "Salle")
        assert np.allclose(d2[0][2], (1.0, 0.5, 0.3))


def test_bulle_de_securite_voit_un_coin_frole_dans_un_virage():
    from cerveau import contact_prevu, distance_devant
    # Coin de meuble à gauche, juste à côté du robot (hors du cône avant de ±20°)
    angles = [math.radians(60)]
    dist = [0.14]
    assert distance_devant(angles, dist) is None                 # le cône avant ne le voit pas
    assert contact_prevu(angles, dist, 0.10, 0.6) is not None    # virage à gauche : il va frotter
    assert contact_prevu(angles, dist, 0.10, -0.6) is None       # virage à droite : il s'en éloigne
    assert contact_prevu(angles, dist, 0.0, 0.6) is None         # rotation sur place : permise
