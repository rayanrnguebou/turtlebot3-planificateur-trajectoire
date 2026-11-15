"""Algorithme de Dijkstra : chemin le plus court garanti, sans heuristique."""

from .grille import Case, Grille
from .recherche import h_nulle, meilleur_d_abord
from .resultat import Resultat


def dijkstra(grille: Grille, depart: Case, arrivee: Case) -> Resultat:
    return meilleur_d_abord(grille, depart, arrivee, h_nulle, "Dijkstra")
