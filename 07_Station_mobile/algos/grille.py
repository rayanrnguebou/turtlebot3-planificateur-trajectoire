"""Carte d'occupation 2D utilisée par tous les planificateurs.

Convention identique à ROS (nav_msgs/OccupancyGrid) :
    0   = case libre
    100 = obstacle
    -1  = inconnu (traité comme obstacle)

Une case est repérée par (ligne, colonne). La conversion vers des mètres
se fait avec `resolution` (m/case) et `origine` (coin bas-gauche en mètres),
ce qui permettra de brancher directement la carte produite par le SLAM.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

LIBRE = 0
OBSTACLE = 100
INCONNU = -1

# Déplacements 8-connexes : (d_ligne, d_colonne, coût)
VOISINS_8 = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
]

Case = tuple[int, int]


@dataclass
class Grille:
    donnees: np.ndarray
    resolution: float = 0.05          # m par case (valeur par défaut de Cartographer)
    origine: tuple[float, float] = (0.0, 0.0)
    _bloquee: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.donnees = np.asarray(self.donnees, dtype=np.int16)
        self._bloquee = self.donnees != LIBRE

    @property
    def hauteur(self) -> int:
        return self.donnees.shape[0]

    @property
    def largeur(self) -> int:
        return self.donnees.shape[1]

    def dans_carte(self, case: Case) -> bool:
        l, c = case
        return 0 <= l < self.hauteur and 0 <= c < self.largeur

    def est_libre(self, case: Case) -> bool:
        return self.dans_carte(case) and not self._bloquee[case]

    def voisins(self, case: Case):
        """Voisins libres en 8-connexité, sans couper les coins d'obstacles."""
        l, c = case
        for dl, dc, cout in VOISINS_8:
            voisin = (l + dl, c + dc)
            if not self.est_libre(voisin):
                continue
            if dl and dc and (self._bloquee[l + dl, c] or self._bloquee[l, c + dc]):
                continue
            yield voisin, cout

    def gonfler(self, rayon_m: float) -> "Grille":
        """Agrandit les obstacles du rayon du robot.

        Le Burger fait ~0.105 m de rayon : en gonflant les obstacles, on peut
        ensuite planifier comme si le robot était un point.
        """
        r = int(math.ceil(rayon_m / self.resolution))
        bloque = self._bloquee
        gonflee = bloque.copy()
        for dl in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dl * dl + dc * dc > r * r:
                    continue
                decale = np.zeros_like(bloque)
                src_l = slice(max(0, -dl), self.hauteur - max(0, dl))
                dst_l = slice(max(0, dl), self.hauteur - max(0, -dl))
                src_c = slice(max(0, -dc), self.largeur - max(0, dc))
                dst_c = slice(max(0, dc), self.largeur - max(0, -dc))
                decale[dst_l, dst_c] = bloque[src_l, src_c]
                gonflee |= decale
        nouvelles = np.where(gonflee, OBSTACLE, LIBRE).astype(np.int16)
        return Grille(nouvelles, self.resolution, self.origine)

    def case_libre_proche(self, case: Case) -> Case:
        """Case libre la plus proche (utile si un point tombe dans un obstacle gonflé)."""
        l0, c0 = case
        for r in range(max(self.hauteur, self.largeur)):
            candidates = [(l0 + dl, c0 + dc) for dl in range(-r, r + 1) for dc in range(-r, r + 1)
                          if max(abs(dl), abs(dc)) == r]
            libres = [c for c in candidates if self.est_libre(c)]
            if libres:
                return min(libres, key=lambda c: math.dist(c, case))
        raise ValueError("la carte ne contient aucune case libre")

    def case_vers_metres(self, case: Case) -> tuple[float, float]:
        """Centre de la case en mètres (x, y), repère de la carte."""
        l, c = case
        return (self.origine[0] + (c + 0.5) * self.resolution,
                self.origine[1] + (l + 0.5) * self.resolution)

    def metres_vers_case(self, x: float, y: float) -> Case:
        return (int((y - self.origine[1]) / self.resolution),
                int((x - self.origine[0]) / self.resolution))


def extremites(grille: Grille) -> tuple[Case, Case]:
    """Départ en bas à gauche, arrivée en haut à droite (cases libres)."""
    return (grille.case_libre_proche((1, 1)),
            grille.case_libre_proche((grille.hauteur - 2, grille.largeur - 2)))


def cout_chemin(chemin: list[Case]) -> float:
    """Longueur d'un chemin en cases (multiplier par la résolution pour des mètres)."""
    return sum(math.dist(a, b) for a, b in zip(chemin, chemin[1:]))


# --------------------------------------------------------------------------
# Cartes de démonstration
# --------------------------------------------------------------------------

def carte_vide(h: int = 60, l: int = 80) -> Grille:
    g = np.zeros((h, l), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OBSTACLE
    return Grille(g)


def carte_murs(h: int = 60, l: int = 80) -> Grille:
    """Trois murs en quinconce : oblige à zigzaguer."""
    g = carte_vide(h, l).donnees.copy()
    g[: int(h * 0.7), l // 4] = OBSTACLE
    g[int(h * 0.3):, l // 2] = OBSTACLE
    g[: int(h * 0.7), 3 * l // 4] = OBSTACLE
    return Grille(g)


def carte_labyrinthe(h: int = 63, l: int = 81, graine: int = 0, couloir: int = 3) -> Grille:
    """Labyrinthe parfait généré par backtracking.

    `couloir` = largeur des couloirs et des murs en cases. Avec 1, les
    couloirs sont si étroits que RRT n'y trouve presque jamais de chemin
    (problème classique des « passages étroits »).
    """
    rng = np.random.default_rng(graine)
    ph, pl = h // couloir, l // couloir
    ph -= 1 - ph % 2                          # dimensions impaires
    pl -= 1 - pl % 2
    p = np.full((ph, pl), OBSTACLE, dtype=np.int16)
    pile = [(1, 1)]
    p[1, 1] = LIBRE
    while pile:
        l0, c0 = pile[-1]
        options = [(l0 + dl, c0 + dc, dl, dc) for dl, dc in ((-2, 0), (2, 0), (0, -2), (0, 2))
                   if 0 < l0 + dl < ph - 1 and 0 < c0 + dc < pl - 1 and p[l0 + dl, c0 + dc] == OBSTACLE]
        if not options:
            pile.pop()
            continue
        nl, nc, dl, dc = options[rng.integers(len(options))]
        p[l0 + dl // 2, c0 + dc // 2] = LIBRE
        p[nl, nc] = LIBRE
        pile.append((nl, nc))
    return Grille(np.kron(p, np.ones((couloir, couloir), dtype=np.int16)))


def carte_aleatoire(h: int = 60, l: int = 80, densite: float = 0.25, graine: int = 0) -> Grille:
    """Obstacles rectangulaires aléatoires (type entrepôt encombré)."""
    rng = np.random.default_rng(graine)
    g = carte_vide(h, l).donnees.copy()
    surface_cible = densite * h * l
    while (g == OBSTACLE).sum() < surface_cible:
        bh, bl = rng.integers(2, 8, size=2)
        l0, c0 = rng.integers(1, h - bh - 1), rng.integers(1, l - bl - 1)
        g[l0:l0 + bh, c0:c0 + bl] = OBSTACLE
    # Garder les coins de départ et d'arrivée dégagés
    g[1:5, 1:5] = LIBRE
    g[-5:-1, -5:-1] = LIBRE
    return Grille(g)


CARTES = {
    "vide": carte_vide,
    "murs": carte_murs,
    "labyrinthe": carte_labyrinthe,
    "aleatoire": carte_aleatoire,
}
