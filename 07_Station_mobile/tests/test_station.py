"""Tests de la station (sans ROS 2).   python -m pytest -q tests

Les tests de mission font rouler le robot simulé en temps réel : ~1 min au total.
"""
import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import station_mobile as sm  # noqa: E402
from cartographie import OCCUPE_UI, Carte, composer, inverser  # noqa: E402
from cerveau import Cerveau, decrire_obstacle, distance_couloir, distance_devant, pourcentage_batterie  # noqa: E402
from navigation import ALGOS, obstacle_sur_chemin, planifier  # noqa: E402
from robot_simule import RobotSimule, salle_de_labo  # noqa: E402


def scan_salle(x, y, th, segments, n=400):
    inc = 2 * math.pi / n
    return [RobotSimule._rayon(x, y, th + i * inc, segments) for i in range(n)], inc


# ----------------------------------------------------------------- outils
def test_composition_de_poses():
    a = (1.0, 2.0, 0.7)
    b = composer(a, inverser(a))
    assert all(abs(v) < 1e-9 for v in b)


def test_pourcentage_batterie():
    assert pourcentage_batterie(12.6) == 100
    assert pourcentage_batterie(10.0) == 0
    assert pourcentage_batterie(11.3) == 50
    assert pourcentage_batterie(None) is None


def test_distance_devant_ignore_mesures_invalides():
    angles = [0.0, 0.1, 1.5, -0.1]
    assert distance_devant(angles, [0.0, float("nan"), 0.3, 0.8]) == 0.8
    assert distance_devant(angles, [float("inf"), 0.0, 0.3, 0.0]) is None


# ----------------------------------------------------------------- carte
def test_cartographie_retrouve_les_murs():
    segs = salle_de_labo()
    carte = Carte(recalage=False)
    for _ in range(5):
        d, inc = scan_salle(0, 0, 0, segs)
        carte.integrer((0, 0, 0), [i * inc for i in range(len(d))], d)
    e = carte.etats()
    l, c = carte.vers_case(-1.0, 0.0)          # mur gauche à x = -1
    assert (e[int(l) - 1:int(l) + 2, int(c) - 1:int(c) + 2] == OCCUPE_UI).any()
    l, c = carte.vers_case(0.5, 0.0)           # espace libre devant le robot
    assert e[int(l), int(c)] == 1


def test_recalage_corrige_une_erreur_d_odometrie():
    """L'odométrie croit le robot 6 cm et 3° à côté : le scan matching corrige."""
    segs = salle_de_labo()
    carte = Carte(recalage=True)
    d, inc = scan_salle(0, 0, 0, segs)
    angles = [i * inc for i in range(len(d))]
    for _ in range(8):
        carte.integrer((0, 0, 0), angles, d)
    vraie = (0.5, -0.3, 0.2)
    d, _ = scan_salle(*vraie, segs)
    odom_fausse = (0.56, -0.27, 0.2 + math.radians(3))
    pose = carte.integrer(odom_fausse, angles, d)
    erreur_sans = math.dist(odom_fausse[:2], vraie[:2])
    erreur_avec = math.dist(pose[:2], vraie[:2])
    assert erreur_avec < erreur_sans / 2
    assert abs(pose[2] - vraie[2]) < math.radians(1.5)


# ----------------------------------------------------------------- planification
def carte_de_test():
    segs = salle_de_labo()
    carte = Carte(recalage=False)
    # Le robot a déjà fait le tour de la table : tous ses côtés sont connus.
    for pose in [(0, 0, 0), (1.4, 1.1, 0), (1.4, -1.1, 0), (2.4, 0, 3.1)]:
        d, inc = scan_salle(*pose, segs)
        for _ in range(3):
            carte.integrer(pose, [i * inc for i in range(len(d))], d)
    return carte


def test_les_trois_algos_contournent_la_table():
    carte = carte_de_test()
    grille = carte.grille_planification(points=[(0, 0), (2.3, 0.1)])
    for algo in ALGOS:
        p = planifier(grille, (0, 0), (2.3, 0.1), algo, graine=1)
        assert p.succes, algo
        assert p.longueur_m >= 2.3 - 1e-6, algo           # jamais plus court que la ligne droite
        # Le chemin ne traverse pas la table (x 1.15..1.65, y -0.6..0.6)
        for (x0, y0), (x1, y1) in zip(p.points, p.points[1:]):
            for k in range(21):
                x, y = x0 + (x1 - x0) * k / 20, y0 + (y1 - y0) * k / 20
                assert not (1.15 - 0.1 < x < 1.65 + 0.1 and -0.6 - 0.1 < y < 0.6 + 0.1), algo


def test_astar_et_dijkstra_meme_longueur_astar_explore_moins():
    carte = carte_de_test()
    grille = carte.grille_planification(points=[(0, 0), (2.3, 0.1)])
    a = planifier(grille, (0, 0), (2.3, 0.1), "A*")
    d = planifier(grille, (0, 0), (2.3, 0.1), "Dijkstra")
    assert a.noeuds < d.noeuds


def test_destination_derriere_un_mur_au_plus_pres():
    """Derrière le mur gauche de la salle : le robot vise le point atteignable le plus proche."""
    carte = carte_de_test()
    pts = [(0, 0), (-1.5, 0.0)]
    grille = carte.grille_planification(points=pts)
    prudente = carte.grille_planification(points=pts, inconnu_libre=False)
    for algo in ALGOS:
        p = planifier(grille, (0, 0), (-1.5, 0.0), algo, graine=1, grille_prudente=prudente)
        assert p.succes and p.mode == "au_plus_pres", algo
        fin = p.points[-1]
        assert -1.0 < fin[0] < -0.6 and abs(fin[1]) < 0.3, (algo, fin)   # contre le mur, côté salle
        assert p.manque_m > 0.4


def grille_couloir(largeur_passage_m):
    """Mur vertical avec une ouverture de la largeur donnée (cases de 5 cm)."""
    from algos.grille import Grille, LIBRE, OBSTACLE
    g = np.full((60, 60), LIBRE, dtype=np.int16)
    g[:, 30] = OBSTACLE
    n = int(round(largeur_passage_m / 0.05))
    g[30 - n // 2:30 - n // 2 + n, 30] = LIBRE
    return Grille(g, 0.05, (0.0, 0.0))


def test_passage_etroit_marge_fine():
    g = grille_couloir(0.30)                     # 30 cm : trop étroit pour la marge normale
    p = planifier(g, (0.5, 1.5), (2.5, 1.5), "A*", grille_prudente=g)
    assert p.succes and p.mode == "etroit"
    g = grille_couloir(0.60)
    p = planifier(g, (0.5, 1.5), (2.5, 1.5), "A*", grille_prudente=g)
    assert p.succes and p.mode == "normal"


def test_aucun_point_plus_proche_reste_un_echec():
    g = grille_couloir(0.0)                      # mur plein
    p = planifier(g, (1.2, 1.5), (2.5, 1.5), "A*", grille_prudente=g)   # robot déjà contre le mur
    assert not p.succes and "plus près" in p.raison


def test_obstacle_sur_chemin():
    chemin = [(x / 20, 0.0) for x in range(40)]
    pts = np.array([[0.4, 0.05], [3.0, 3.0]])
    assert obstacle_sur_chemin(pts, chemin, 0) is not None
    pts = np.array([[0.4, 0.5]])
    assert obstacle_sur_chemin(pts, chemin, 0) is None


# ----------------------------------------------------------------- sécurité
def lidar_degage(c):
    """Scan récent sans aucun obstacle proche (sinon le manuel roule au pas)."""
    c.recevoir_scan(0.0, 2 * math.pi / 360, 0.1, 12.0, [3.0] * 360)


def test_arret_urgence_et_homme_mort_en_manuel():
    c = Cerveau(mode="test")
    c.recevoir_odom(0, 0, 0, 0, 0)
    lidar_degage(c)
    c.commande_manuelle(0.5, 3.0)
    v, w = c.cycle()
    assert (v, w) == (c.v_max, c.w_max)                  # plafonné
    time.sleep(0.6)
    assert c.cycle() == (0.0, 0.0)                       # homme mort
    c.commande_manuelle(0.1, 0)
    c.regler_arret(True)
    assert c.cycle() == (0.0, 0.0)
    ok, raison = c.nouvelle_mission(1, 1)
    assert not ok and "urgence" in raison


def test_mission_refusee_si_batterie_vide():
    c = Cerveau(mode="test")
    c.recevoir_odom(0, 0, 0, 0, 0)
    c.recevoir_batterie(10.3)
    ok, raison = c.nouvelle_mission(1, 0)
    assert not ok and "batterie" in raison


# ----------------------------------------------------------------- missions complètes
def mission(algo, obstacle=False, but=(2.3, 0.1), limite=70):
    c = Cerveau(mode="simulation")
    r = RobotSimule(c)
    r.start()
    try:
        time.sleep(1.2)
        assert c.nouvelle_mission(*but, algo)[0]
        t0, pose_obstacle = time.time(), None
        while time.time() - t0 < limite:
            time.sleep(0.25)
            if obstacle and pose_obstacle is None and c.etat == "en_route" and time.time() - t0 > 4:
                pose_obstacle = r.poser_obstacle(0.45)
            if c.etat in ("arrive", "echec"):
                break
        return c, r
    finally:
        r.arreter.set()


def test_mission_autonome_astar():
    c, r = mission("A*")
    assert c.etat == "arrive"
    assert r.collisions == 0
    assert math.dist(r.etat_reel()["pose_reelle"][:2], (2.3, 0.1)) < 0.25
    textes = [e["type"] for e in c.evenements]
    assert textes[-1] == "arrive"


def test_mission_avec_obstacle_imprevu_dijkstra():
    c, r = mission("Dijkstra", obstacle=True)
    assert c.etat == "arrive"
    assert r.collisions == 0
    types = [e["type"] for e in c.evenements]
    assert "obstacle" in types or "itineraire" in types


def test_mission_but_inaccessible_va_au_plus_pres():
    c, r = mission("A*", but=(-1.6, 0.0))
    assert c.etat == "arrive" and r.collisions == 0
    types = [e["type"] for e in c.evenements]
    assert "au_plus_pres" in types and types[-1] == "arrive_proche"
    assert c.bilan["au_plus_pres_manque_m"] > 0.4


def test_mission_rrt():
    c, r = mission("RRT")
    assert c.etat == "arrive" and r.collisions == 0


# ----------------------------------------------------------------- serveur web
def test_serveur_web():
    c = Cerveau(mode="simulation")
    simu = RobotSimule(c)
    simu.start()
    serveur = ThreadingHTTPServer(("127.0.0.1", 0), sm.fabriquer_gestionnaire(c, simu))
    threading.Thread(target=serveur.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{serveur.server_address[1]}"

    def get(ch):
        return json.loads(urllib.request.urlopen(base + ch, timeout=3).read())

    def post(ch, corps=None):
        req = urllib.request.Request(base + ch, data=json.dumps(corps or {}).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            return json.loads(urllib.request.urlopen(req, timeout=5).read())
        except urllib.error.HTTPError as e:
            return {"code": e.code, **json.loads(e.read() or b"{}")}

    try:
        page = urllib.request.urlopen(base + "/", timeout=3).read().decode()
        assert "GROUND CONTROL" in page
        time.sleep(1.0)
        e = get("/api/etat")
        assert e["station"] == "simulation" and e["tension"] and len(e["scan"]) > 300
        k = get("/api/carte")
        assert k["largeur"] * k["hauteur"] > 0 and k["cases"]
        cmp = post("/api/comparer", {"x": 2.3, "y": 0.1})
        assert [r["algo"] for r in cmp["resultats"]] == ["A*", "Dijkstra", "RRT"]
        assert post("/api/mission", {"x": 2.3, "y": 0.1, "algo": "A*"})["ok"]
        time.sleep(0.5)
        assert get("/api/etat")["etat"] in ("planification", "en_route")
        post("/api/arret")
        e = get("/api/etat")
        assert e["arret_urgence"] and e["etat"] == "attente"
        assert post("/api/mission", {"x": 1, "y": 1})["code"] == 409
        post("/api/reprendre")
        assert post("/api/joystick", {"v": "abc"})["code"] == 400
        assert post("/api/vitesse", {"niveau": 3})["v_max"] == 0.22
        try:
            urllib.request.urlopen(base + "/../cerveau.py", timeout=3)
            assert False
        except urllib.error.HTTPError as err:
            assert err.code == 404
    finally:
        serveur.shutdown()
        simu.arreter.set()


def test_reglage_vitesse_depuis_la_manette():
    c = Cerveau(mode="test")
    assert c.niveau_vitesse == 2 and c.v_max == 0.15
    assert c.regler_vitesse(1) == (1, 0.10) and c.suivi.v_max == 0.10
    assert c.regler_vitesse(3) == (3, 0.22)
    assert c.regler_vitesse(7) == (3, 0.22)               # niveau inconnu : ignoré
    lent = Cerveau(v_max=0.10, mode="test")                # plafond --vmax 0.10
    assert lent.regler_vitesse(3) == (3, 0.10)
    c.recevoir_odom(0, 0, 0, 0, 0)
    lidar_degage(c)
    c.regler_vitesse(1)
    c.commande_manuelle(1.0, 0)
    assert c.cycle()[0] == 0.10                            # le joystick respecte le niveau


def test_manuel_freine_et_bloque_devant_et_derriere():
    """Mur à 0.15 m derrière le robot : impossible de reculer, mais avancer reste possible."""
    angles = [i * 2 * math.pi / 360 for i in range(360)]
    assert abs(distance_couloir(angles, [0.15 / max(abs(math.cos(a)), 1e-3) for a in angles], sens=-1) - 0.15) < 0.01
    dist = [0.15 / -math.cos(a) if math.cos(a) < -0.2 else 0.0 for a in angles]   # mur derrière seulement
    c = Cerveau(mode="test")
    c.recevoir_odom(0, 0, 0, 0, 0)
    c.recevoir_scan(0.0, angles[1], 0.1, 12.0, dist)
    c.commande_manuelle(-1.0, 0)
    assert c.cycle()[0] == 0.0                              # recul bloqué
    c.commande_manuelle(1.0, 0)
    assert c.cycle()[0] == c.v_max                          # avance libre
    dist = [0.30 / math.cos(a) if math.cos(a) > 0.2 else 0.0 for a in angles]     # mur à 30 cm devant
    c.recevoir_scan(0.0, angles[1], 0.1, 12.0, dist)
    v = c.cycle()[0]
    assert 0 < v < c.v_max                                  # freinage progressif


def test_manuel_sans_lidar_roule_au_pas():
    c = Cerveau(mode="test")
    c.recevoir_odom(0, 0, 0, 0, 0)
    c.commande_manuelle(1.0, 0.5)
    v, w = c.cycle()
    assert v == 0.05 and w == 0.5


# ----------------------------------------------------------------- carte enregistrée
def cerveau_avec_carte_de_test():
    c = Cerveau(mode="test")
    c.carte = carte_de_test()
    c.recevoir_odom(0, 0, 0, 0, 0)
    return c


def test_enregistrer_et_recharger_la_carte_avec_les_lieux(tmp_path):
    c = cerveau_avec_carte_de_test()
    fichier = str(tmp_path / "carte.npz")
    c.fichier_carte = fichier
    assert c.memoriser_lieu("  Bureau du prof ")[0]          # enregistre aussi la carte
    assert c.lieux == {"Bureau du prof": (0.0, 0.0)}
    c.depart_base = (0.5, -0.2)
    assert c.enregistrer_carte()[0]
    d = Cerveau(mode="test")
    infos = d.charger_carte(fichier)
    assert np.array_equal(d.carte.etats(), c.carte.etats())
    assert d.lieux == {"Bureau du prof": (0.0, 0.0)} and d.depart_base == (0.5, -0.2)
    assert infos["date"] and not d.localise
    # Tant qu'il ne s'est pas retrouvé : pas de mission, et on n'écrase pas la carte
    d.recevoir_odom(0, 0, 0, 0, 0)
    ok, raison = d.nouvelle_mission(1.0, 1.0)
    assert not ok and "position" in raison
    assert not d.enregistrer_carte()[0]
    assert d.supprimer_lieu("Inconnu")[0] is False


def test_relocalisation_partout_sur_la_carte():
    """Robot posé n'importe où (odométrie à zéro) : il retrouve sa vraie position."""
    carte = carte_de_test()
    segs = salle_de_labo()
    for vraie in [(0.2, 0.3, 1.0), (2.4, 0.4, 2.8), (0.5, -1.2, -0.5), (3.4, -0.5, -2.0)]:
        d, inc = scan_salle(*vraie, segs)
        pose, qualite, ambigu = carte.localiser_global([i * inc for i in range(len(d))], d)
        assert math.dist(pose[:2], vraie[:2]) < 0.10, (vraie, pose)
        assert abs(math.atan2(math.sin(pose[2] - vraie[2]), math.cos(pose[2] - vraie[2]))) < math.radians(5)
        assert qualite > 0.7 and not ambigu


def test_carte_rechargee_robot_pose_ailleurs_puis_mission(tmp_path):
    """Session 1 : le robot cartographie. Session 2 : on le pose AILLEURS,
    il se retrouve seul, puis va à un lieu mémorisé sans collision."""
    fichier = str(tmp_path / "carte.npz")
    c, r = mission("A*")                          # session 1 : la mission construit la carte
    assert c.etat == "arrive"
    c.fichier_carte = fichier
    c.lieux["Table"] = (2.3, 0.1)
    assert c.enregistrer_carte()[0]

    d = Cerveau(mode="simulation")                # session 2 : nouveau démarrage
    d.charger_carte(fichier)
    vraie = (0.3, -1.3, 1.2)
    r2 = RobotSimule(d, depart=vraie)             # odométrie à (0, 0, 0), robot ailleurs
    r2.start()
    try:
        t0 = time.time()
        while not d.localise and time.time() - t0 < 15:
            time.sleep(0.2)
        assert d.localise, d.derniere_loc
        types = [e["type"] for e in d.evenements]
        assert "position_trouvee" in types
        pose, reel = d.pose(), r2.etat_reel()["pose_reelle"]
        assert math.dist(pose[:2], reel[:2]) < 0.10
        assert abs(math.atan2(math.sin(pose[2] - reel[2]), math.cos(pose[2] - reel[2]))) < math.radians(5)
        assert d.aller_lieu("Table", "A*")[0]
        t0 = time.time()
        while d.etat not in ("arrive", "echec") and time.time() - t0 < 70:
            time.sleep(0.25)
        assert d.etat == "arrive" and r2.collisions == 0
        assert math.dist(r2.etat_reel()["pose_reelle"][:2], (2.3, 0.1)) < 0.25
    finally:
        r2.arreter.set()


# ----------------------------------------------------------------- dégagement et exploration
def test_coince_devant_une_boite_il_recule_puis_arrive():
    c = Cerveau(mode="simulation")
    r = RobotSimule(c)
    r.start()
    try:
        time.sleep(1.2)
        assert c.nouvelle_mission(2.3, 0.9, "A*")[0]
        t0, pose = time.time(), False
        while time.time() - t0 < 70 and c.etat not in ("arrive", "echec"):
            if c.etat == "en_route" and not pose and time.time() - t0 > 3:
                r.poser_obstacle(0.30, 0.10)          # boîte collée devant lui, qui reste
                pose = True
            time.sleep(0.1)
        types = [e["type"] for e in c.evenements]
        assert c.etat == "arrive" and r.collisions == 0
        assert "recul" in types and c.bilan["reculs"] >= 1
    finally:
        r.arreter.set()


def test_anti_boucle_abandonne_au_plus_pres():
    c = Cerveau(mode="test")
    c.recevoir_odom(0, 0, 0, 0, 0)
    c.but = (2.0, 0.0)
    c.mission = {"meilleure_d": math.inf, "sans_progres": 0, "debut": time.monotonic(),
                 "tension": None, "distance": 0.0, "obstacles": 0, "replanifications": 0,
                 "algo": "A*", "reculs": 0}
    resultats = [c._abandon_si_bloque((0.0, 0.0, 0.0)) for _ in range(5)]
    assert resultats == [False, False, False, False, True]   # 1er = référence, puis 4 sans progrès
    assert c.etat == "echec" and c.evenements[-1]["type"] == "bloque_proche"


def test_exploration_autonome_toute_la_salle():
    c = Cerveau(mode="simulation")
    r = RobotSimule(c)
    r.start()
    try:
        time.sleep(1.2)
        assert c.explorer("A*")[0]
        t0 = time.time()
        while c.exploration is not None and time.time() - t0 < 240:
            time.sleep(0.5)
        assert c.exploration is None and r.collisions == 0
        assert "exploration_finie" in [e["type"] for e in c.evenements]
        libre = (c.carte.etats() == 1).sum() * c.carte.res ** 2
        assert libre > 15.0          # salle de 20 m², dont environ 1,5 m² de meubles
    finally:
        r.arreter.set()


def test_annonce_distance_et_cote_de_l_obstacle():
    assert decrire_obstacle(0.45, 0.0) == "à 35 cm, devant"
    assert decrire_obstacle(0.40, math.radians(35)) == "à 30 cm, devant à gauche"
    assert decrire_obstacle(0.60, math.radians(-80)) == "à 50 cm, à droite"
    # En manuel : prévenu à 50 cm, AVANT le freinage final et l'arrêt à 20 cm
    c = Cerveau(mode="test")
    c.recevoir_odom(0, 0, 0, 0, 0)
    n = 360
    dist = [3.0] * n
    dist[0] = 0.45                                   # obstacle droit devant, à 45 cm du centre
    c.recevoir_scan(0.0, 2 * math.pi / n, 0.1, 12.0, dist)
    c.commande_manuelle(0.15, 0.0)
    v, _ = c.cycle()
    assert 0 < v <= 0.15
    textes = [e["texte"] for e in c.evenements if e["type"] == "obstacle_loin"]
    assert textes == ["Attention, obstacle à 35 cm, devant. Je ralentis."]
