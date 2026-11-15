"""Exploration autonome par « frontières » (Yamauchi, 1997).

Une frontière est une case LIBRE de la carte qui touche une case INCONNUE : c'est
le bord de ce que le robot a déjà vu. Pour découvrir une nouvelle zone, il suffit
d'aller à la frontière atteignable la plus proche ; le LiDAR révèle ce qu'il y a
derrière, la carte grandit, de nouvelles frontières apparaissent… et on recommence
jusqu'à ce qu'il n'en reste plus : la pièce est entièrement cartographiée.

Sans ROS : testable sous Windows.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from cartographie import INCONNU_UI, LIBRE_UI, OCCUPE_UI

TAILLE_MINI = 4            # cases (20 cm) : une frontière plus petite est du bruit
DISTANCE_MINI = 0.40       # m : on ne vise pas une frontière collée au robot


def dilater(masque, rayon_cases):
    """Grossit les obstacles d'un disque de `rayon_cases` (marge de sécurité du robot)."""
    sortie = masque.copy()
    r = int(rayon_cases)
    h, w = masque.shape
    for dl in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dl * dl + dc * dc > r * r or (dl == 0 and dc == 0):
                continue
            src = masque[max(0, -dl):h - max(0, dl), max(0, -dc):w - max(0, dc)]
            sortie[max(0, dl):h - max(0, -dl), max(0, dc):w - max(0, -dc)] |= src
    return sortie


def prochaine_frontiere(carte, pose, rayon=0.18, interdits=(), distance_interdite=0.5):
    """Point (x, y) de la frontière atteignable la plus proche, ou None si tout est exploré.

    Parcours en largeur depuis le robot, uniquement sur des cases déjà vues libres et
    loin des obstacles : la cible est donc toujours réellement atteignable.
    `interdits` : cibles déjà tentées sans succès (on ne s'y acharne pas).
    """
    e = carte.etats()
    l0, l1, c0, c1 = carte.zone_connue(marge_cases=2, points=[pose[:2]])
    e = e[l0:l1, c0:c1]
    bloque = dilater(e == OCCUPE_UI, math.ceil(rayon / carte.res))
    praticable = (e == LIBRE_UI) & ~bloque
    inconnu = e == INCONNU_UI
    voisin_inconnu = np.zeros_like(inconnu)
    voisin_inconnu[1:, :] |= inconnu[:-1, :]
    voisin_inconnu[:-1, :] |= inconnu[1:, :]
    voisin_inconnu[:, 1:] |= inconnu[:, :-1]
    voisin_inconnu[:, :-1] |= inconnu[:, 1:]
    frontiere = praticable & voisin_inconnu

    def metres(case):
        return (carte.origine[0] + (c0 + case[1] + 0.5) * carte.res,
                carte.origine[1] + (l0 + case[0] + 0.5) * carte.res)

    l, c = carte.vers_case(pose[0], pose[1])
    depart = (int(l) - l0, int(c) - c0)
    h, w = e.shape
    if not (0 <= depart[0] < h and 0 <= depart[1] < w):
        return None
    if not praticable[depart]:
        # Robot dans la marge d'un obstacle : on part de la case praticable la plus proche
        cases = np.argwhere(praticable)
        if not len(cases):
            return None
        k = np.argmin(((cases - np.array(depart)) ** 2).sum(axis=1))
        if math.dist(cases[k], depart) * carte.res > 0.4:
            return None
        depart = tuple(int(v) for v in cases[k])

    vus = np.zeros_like(praticable)
    vus[depart] = True
    file = deque([depart])
    deja = np.zeros_like(praticable)          # frontières déjà regroupées
    while file:
        case = file.popleft()
        if frontiere[case] and not deja[case]:
            groupe = _groupe(frontiere, deja, case)
            p = metres(case)
            if len(groupe) >= TAILLE_MINI and math.dist(p, pose[:2]) >= DISTANCE_MINI \
                    and all(math.dist(p, q) > distance_interdite for q in interdits):
                # On vise le milieu du groupe (la case du groupe la plus proche de son centre)
                g = np.array(groupe)
                centre = g.mean(axis=0)
                milieu = groupe[int(np.argmin(((g - centre) ** 2).sum(axis=1)))]
                return metres(milieu)
        cl, cc = case
        for dl, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            v = (cl + dl, cc + dc)
            if 0 <= v[0] < h and 0 <= v[1] < w and praticable[v] and not vus[v]:
                vus[v] = True
                file.append(v)
    return None


def _groupe(frontiere, deja, depart):
    """Cases de frontière reliées (8-voisinage) à `depart`."""
    h, w = frontiere.shape
    deja[depart] = True
    pile, groupe = [depart], []
    while pile:
        cl, cc = pile.pop()
        groupe.append((cl, cc))
        for dl in (-1, 0, 1):
            for dc in (-1, 0, 1):
                v = (cl + dl, cc + dc)
                if 0 <= v[0] < h and 0 <= v[1] < w and frontiere[v] and not deja[v]:
                    deja[v] = True
                    pile.append(v)
    return groupe
