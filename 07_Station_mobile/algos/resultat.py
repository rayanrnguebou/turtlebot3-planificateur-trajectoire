"""Format de sortie commun à tous les planificateurs, pour pouvoir les comparer."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Resultat:
    algo: str
    chemin: list[tuple[float, float]]  # points (ligne, colonne), vide si échec
    cout: float                        # longueur du chemin en cases
    noeuds_explores: int               # cases développées (A*/Dijkstra) ou nœuds de l'arbre (RRT)
    temps_s: float
    explores: list[tuple[int, int]] = field(default_factory=list, repr=False)   # pour l'affichage
    arbre: list[tuple[tuple[float, float], tuple[float, float]]] = field(default_factory=list, repr=False)

    @property
    def trouve(self) -> bool:
        return bool(self.chemin)
