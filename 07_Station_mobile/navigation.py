"""Planification sur la carte construite par le robot, avec NOS algorithmes.

On réutilise tels quels A*, Dijkstra et RRT de la phase 1 (dossier `algos/`,
copie identique de 03_Phase1_Algorithmes/algos) et le suivi Pure Pursuit de la
phase 2 (`controle.py`, copie de 04_Phase2).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from algos import PLANIFICATEURS
from algos.rrt import lisser, rrt
from cartographie import OCCUPE_UI

ALGOS = list(PLANIFICATEURS)          # ["A*", "Dijkstra", "RRT"]
RAYON_ROBOT = 0.105                   # Burger : 105 mm
RAYON_GONFLAGE = 0.18                 # rayon + 7,5 cm de marge
RAYON_FIN = 0.10                      # 2e essai : 2 cases, soit au moins 12,5 cm entre le centre du robot et un mur
GAIN_MINI = 0.20                      # « au plus près » seulement s'il rapproche d'au moins 20 cm


@dataclass
class Plan:
    algo: str
    succes: bool
    points: list = field(default_factory=list)   # [(x, y)] en mètres, lissé
    longueur_m: float = 0.0
    temps_ms: float = 0.0
    noeuds: int = 0
    virages_deg: float = 0.0
    raison: str = ""
    mode: str = "normal"        # normal | etroit (marge fine) | au_plus_pres
    manque_m: float = 0.0       # au_plus_pres : distance qui reste jusqu'au but demandé

    def resume(self):
        return {"algo": self.algo, "succes": self.succes, "mode": self.mode,
                "manque_m": round(self.manque_m, 2),
                "longueur_m": round(self.longueur_m, 2),
                "temps_ms": round(self.temps_ms, 1), "noeuds": self.noeuds,
                "virages_deg": round(self.virages_deg), "raison": self.raison,
                "points": [[round(x, 3), round(y, 3)] for x, y in self.points]}


def longueur(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def virages(points, depart_cap=None):
    """Somme des changements de direction (en degrés) : coûteux en énergie."""
    caps = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(points, points[1:])
            if math.dist(a, b) > 1e-6]
    if depart_cap is not None:
        caps.insert(0, depart_cap)
    total = 0.0
    for c0, c1 in zip(caps, caps[1:]):
        total += abs(math.atan2(math.sin(c1 - c0), math.cos(c1 - c0)))
    return math.degrees(total)


def planifier(grille, depart, but, algo="A*", cap=None, graine=None, grille_prudente=None):
    """Calcule un chemin de `depart` à `but` (mètres), en faisant tout son possible :

    1. marge normale autour des obstacles (0,18 m) ;
    2. sinon marge fine (0,10 m), mais seulement dans l'espace DÉJÀ VU libre
       (`grille_prudente`) : une petite lacune dans un mur cartographié ne doit
       jamais devenir un « passage » ;
    3. sinon il vise le point ATTEIGNABLE le plus proche du but (« au plus près »),
       avec la marge normale.
    """
    t0 = time.perf_counter()
    if algo not in PLANIFICATEURS:
        return Plan(algo, False, raison="algorithme inconnu")
    p = _planifier(grille, depart, but, algo, cap, graine, RAYON_GONFLAGE)
    if p.succes:
        return p
    premier_echec = p
    if grille_prudente is not None:
        p = _planifier(grille_prudente, depart, but, algo, cap, graine, RAYON_FIN)
        if p.succes:
            p.mode = "etroit"
            p.temps_ms = (time.perf_counter() - t0) * 1000
            return p
    proche = point_atteignable_le_plus_proche(grille, depart, but, RAYON_GONFLAGE)
    if proche is not None:
        p = _planifier(grille, depart, proche, algo, cap, graine, RAYON_GONFLAGE)
        if p.succes:
            p.mode = "au_plus_pres"
            p.manque_m = math.dist(proche, but)
            p.temps_ms = (time.perf_counter() - t0) * 1000
            return p
    premier_echec.temps_ms = (time.perf_counter() - t0) * 1000
    if premier_echec.raison == "aucun chemin trouvé":
        premier_echec.raison = "aucun chemin, même au plus près"
    return premier_echec


def point_atteignable_le_plus_proche(grille, depart, but, rayon):
    """Case libre reliée au robot la plus proche du but, en mètres (ou None).

    Parcours en largeur depuis le robot sur la grille gonflée : on ne garde que des
    cases réellement atteignables, donc jamais un point derrière un mur.
    """
    g = grille.gonfler(rayon)
    d = g.metres_vers_case(*depart)
    if not g.dans_carte(d):
        return None
    if not g.est_libre(d):
        d = g.case_libre_proche(d)
    b = g.metres_vers_case(*but)
    vus, file, i = {d}, [d], 0
    meilleure, d_min = d, math.dist(d, b)
    while i < len(file) and len(vus) < 200_000:
        case = file[i]
        i += 1
        dist = math.dist(case, b)
        if dist < d_min:
            meilleure, d_min = case, dist
        for v, _ in g.voisins(case):
            if v not in vus:
                vus.add(v)
                file.append(v)
    gain = (math.dist(d, b) - d_min) * g.resolution
    if gain < GAIN_MINI:
        return None                   # on est déjà au plus près : rien à gagner
    return g.case_vers_metres(meilleure)


def _planifier(grille, depart, but, algo, cap, graine, rayon):
    t0 = time.perf_counter()
    g = grille.gonfler(rayon)

    d = g.metres_vers_case(*depart)
    if not g.dans_carte(d):
        return Plan(algo, False, raison="robot hors de la carte")
    if not g.est_libre(d):
        # Le robot est collé à un obstacle « gonflé » : on part de la case libre la plus proche
        d = g.case_libre_proche(d)
    b = g.metres_vers_case(*but)
    if not g.dans_carte(b):
        return Plan(algo, False, raison="destination hors de la carte")
    if not g.est_libre(b):
        b2 = g.case_libre_proche(b)
        if math.dist(b, b2) * g.resolution > 0.30:
            return Plan(algo, False, raison="destination dans un obstacle")
        b = b2

    if algo == "RRT":
        r = rrt(g, d, b, iterations_max=2500, graine=graine)   # borné : la Pi reste réactive
    else:
        r = PLANIFICATEURS[algo](g, d, b)
    if not r.trouve:
        return Plan(algo, False, noeuds=r.noeuds_explores,
                    temps_ms=(time.perf_counter() - t0) * 1000,
                    raison="aucun chemin trouvé")
    chemin = lisser(g, r.chemin)                         # moins de virages
    pts = [g.case_vers_metres(c) for c in chemin]
    pts[-1] = (float(but[0]), float(but[1])) if g.est_libre(g.metres_vers_case(*but)) else pts[-1]
    return Plan(algo, True, pts, longueur(pts), (time.perf_counter() - t0) * 1000,
                r.noeuds_explores, virages(pts, cap))


def points_lidar_monde(pose, angles, distances, rmin=0.1, rmax=3.5):
    a = np.asarray(angles, dtype=np.float64)
    r = np.asarray(distances, dtype=np.float64)
    ok = np.isfinite(r) & (r >= max(rmin, 0.05)) & (r <= rmax)
    a, r = a[ok], r[ok]
    x, y, t = pose
    return np.column_stack((x + r * np.cos(t + a), y + r * np.sin(t + a)))


def obstacle_sur_chemin(points_lidar, chemin_dense, indice, longueur_avant=0.6,
                        marge=0.12, avec_point=False):
    """Distance le long du chemin au premier point LiDAR qui le bloque, ou None.
    Avec `avec_point=True` : renvoie (distance, (x, y) du point qui bloque) ou None.

    Un point bloque le chemin s'il est à moins de `marge` (rayon du robot +
    1,5 cm) d'un point des `longueur_avant` prochains mètres du chemin.
    """
    if len(points_lidar) == 0 or not chemin_dense:
        return None
    parcouru = 0.0
    prec = chemin_dense[min(indice, len(chemin_dense) - 1)]
    for i in range(indice, len(chemin_dense)):
        p = chemin_dense[i]
        parcouru += math.dist(prec, p)
        prec = p
        if parcouru > longueur_avant:
            break
        d2 = (points_lidar[:, 0] - p[0]) ** 2 + (points_lidar[:, 1] - p[1]) ** 2
        k = int(d2.argmin())
        if d2[k] < marge * marge:
            if avec_point:
                return parcouru, (float(points_lidar[k, 0]), float(points_lidar[k, 1]))
            return parcouru
    return None


def chemin_traverse_obstacle(etats, carte, chemin_dense, indice, pas=3):
    """True si la suite du chemin passe sur un mur que la carte vient de découvrir."""
    n = etats.shape[0]
    r = int(RAYON_ROBOT / carte.res)                    # 2 cases = 10 cm
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disque = (yy * yy + xx * xx) <= r * r
    for (x, y) in chemin_dense[indice::pas]:
        l, c = carte.vers_case(x, y)
        l, c = int(l), int(c)
        if not (r <= l < n - r and r <= c < n - r):
            continue
        if ((etats[l - r:l + r + 1, c - r:c + r + 1] == OCCUPE_UI) & disque).any():
            return True
    return False
