"""RRT (Rapidly-exploring Random Tree) et lissage du chemin.

Contrairement à A*/Dijkstra, RRT ne parcourt pas la grille case par case :
il tire des points au hasard dans l'espace continu et fait pousser un arbre
vers eux. Il trouve vite un chemin dans les grands espaces, mais ce chemin
n'est pas optimal et change d'une exécution à l'autre (d'où la graine).
"""

from __future__ import annotations

import math
import random
import time

from .grille import Case, Grille
from .resultat import Resultat

Point = tuple[float, float]   # (ligne, colonne) en coordonnées continues


def _case(p: Point) -> Case:
    return int(round(p[0])), int(round(p[1]))


def segment_libre(grille: Grille, a: Point, b: Point, pas: float = 0.3) -> bool:
    """Vérifie qu'un segment ne traverse aucun obstacle (échantillonnage)."""
    n = max(1, int(math.dist(a, b) / pas))
    for i in range(n + 1):
        t = i / n
        if not grille.est_libre(_case((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))):
            return False
    return True


def rrt(grille: Grille, depart: Case, arrivee: Case, pas_max: float = 3.0,
        biais_but: float = 0.1, iterations_max: int = 20000, graine: int | None = 0) -> Resultat:
    """
    pas_max      : longueur maximale d'une nouvelle branche (en cases)
    biais_but    : probabilité de tirer directement l'arrivée (accélère la convergence)
    graine       : rend le résultat reproductible ; None = aléatoire
    """
    t0 = time.perf_counter()
    rng = random.Random(graine)
    if not (grille.est_libre(depart) and grille.est_libre(arrivee)):
        return Resultat("RRT", [], math.inf, 0, time.perf_counter() - t0)

    noeuds: list[Point] = [(float(depart[0]), float(depart[1]))]
    parent: list[int] = [-1]
    but: Point = (float(arrivee[0]), float(arrivee[1]))
    arbre = []

    for _ in range(iterations_max):
        if rng.random() < biais_but:
            cible = but
        else:
            cible = (rng.uniform(0, grille.hauteur - 1), rng.uniform(0, grille.largeur - 1))

        i_proche = min(range(len(noeuds)), key=lambda i: math.dist(noeuds[i], cible))
        proche = noeuds[i_proche]
        d = math.dist(proche, cible)
        if d == 0:
            continue
        # On avance vers la cible ; si un obstacle bloque, on réessaie avec un
        # pas plus court (indispensable dans les couloirs étroits).
        pas = min(pas_max, d)
        while pas >= 0.5:
            nouveau = (proche[0] + pas / d * (cible[0] - proche[0]),
                       proche[1] + pas / d * (cible[1] - proche[1]))
            if segment_libre(grille, proche, nouveau):
                break
            pas /= 2
        else:
            continue

        noeuds.append(nouveau)
        parent.append(i_proche)
        arbre.append((proche, nouveau))

        if math.dist(nouveau, but) <= pas_max and segment_libre(grille, nouveau, but):
            noeuds.append(but)
            parent.append(len(noeuds) - 2)
            arbre.append((nouveau, but))
            chemin = []
            i = len(noeuds) - 1
            while i != -1:
                chemin.append(noeuds[i])
                i = parent[i]
            chemin.reverse()
            return Resultat("RRT", chemin, longueur(chemin), len(noeuds),
                            time.perf_counter() - t0, arbre=arbre)

    return Resultat("RRT", [], math.inf, len(noeuds), time.perf_counter() - t0, arbre=arbre)


def longueur(chemin: list[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(chemin, chemin[1:]))


def lisser(grille: Grille, chemin: list[Point]) -> list[Point]:
    """Supprime les détours : relie chaque point au point visible le plus lointain.

    Utile après RRT (chemin en zigzag) et aussi après A* (marches d'escalier
    dues à la grille). Le robot a ainsi moins de virages à faire.
    """
    if len(chemin) < 3:
        return list(chemin)
    lisse = [chemin[0]]
    i = 0
    while i < len(chemin) - 1:
        j = len(chemin) - 1
        while j > i + 1 and not segment_libre(grille, chemin[i], chemin[j]):
            j -= 1
        lisse.append(chemin[j])
        i = j
    return lisse
