"""Algorithme A* : chemin le plus court guidé par une heuristique."""

from .grille import Case, Grille
from .recherche import Heuristique, h_octile, meilleur_d_abord
from .resultat import Resultat


def astar(grille: Grille, depart: Case, arrivee: Case,
          heuristique: Heuristique = h_octile) -> Resultat:
    return meilleur_d_abord(grille, depart, arrivee, heuristique, "A*")
