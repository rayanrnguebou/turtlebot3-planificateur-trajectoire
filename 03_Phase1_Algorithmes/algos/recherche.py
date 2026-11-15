"""Recherche du meilleur chemin sur grille : base commune de Dijkstra et A*.

Dijkstra et A* sont le même algorithme ; seule change l'heuristique h(n) :
    Dijkstra : f(n) = g(n)            (h = 0, explore dans toutes les directions)
    A*       : f(n) = g(n) + h(n)     (h estime la distance restante)
Avec une heuristique admissible (qui ne surestime jamais), A* trouve le même
coût optimal que Dijkstra en explorant beaucoup moins de cases.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from typing import Callable

from .grille import Case, Grille
from .resultat import Resultat

Heuristique = Callable[[Case, Case], float]


def h_nulle(a: Case, b: Case) -> float:
    return 0.0


def h_octile(a: Case, b: Case) -> float:
    """Distance exacte sur une grille 8-connexe sans obstacles (admissible)."""
    dl, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dl, dc) + (math.sqrt(2) - 1) * min(dl, dc)


def h_euclidienne(a: Case, b: Case) -> float:
    return math.dist(a, b)


def meilleur_d_abord(grille: Grille, depart: Case, arrivee: Case,
                     heuristique: Heuristique, nom: str) -> Resultat:
    t0 = time.perf_counter()
    if not (grille.est_libre(depart) and grille.est_libre(arrivee)):
        return Resultat(nom, [], math.inf, 0, time.perf_counter() - t0)

    compteur = itertools.count()              # départage les égalités dans le tas
    ouverts = [(heuristique(depart, arrivee), next(compteur), depart)]
    g = {depart: 0.0}
    parent: dict[Case, Case] = {}
    fermes: set[Case] = set()
    ordre_exploration: list[Case] = []

    while ouverts:
        _, _, courant = heapq.heappop(ouverts)
        if courant in fermes:
            continue                          # entrée périmée du tas
        fermes.add(courant)
        ordre_exploration.append(courant)

        if courant == arrivee:
            chemin = [courant]
            while chemin[-1] in parent:
                chemin.append(parent[chemin[-1]])
            chemin.reverse()
            return Resultat(nom, chemin, g[arrivee], len(fermes),
                            time.perf_counter() - t0, ordre_exploration)

        for voisin, cout in grille.voisins(courant):
            if voisin in fermes:
                continue
            nouveau_g = g[courant] + cout
            if nouveau_g < g.get(voisin, math.inf):
                g[voisin] = nouveau_g
                parent[voisin] = courant
                f = nouveau_g + heuristique(voisin, arrivee)
                heapq.heappush(ouverts, (f, next(compteur), voisin))

    return Resultat(nom, [], math.inf, len(fermes), time.perf_counter() - t0, ordre_exploration)
