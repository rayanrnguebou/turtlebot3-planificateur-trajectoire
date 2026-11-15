"""Conversion entre la carte ROS (nav_msgs/OccupancyGrid) et notre Grille.

Volontairement sans import ROS : ces fonctions se testent sous Windows.

Rappel du format OccupancyGrid :
    - data est une liste « à plat », ligne par ligne, en partant du coin
      (origin.x, origin.y) de la carte ;
    - ligne = axe y, colonne = axe x, ce qui correspond à nos cases (ligne, colonne) ;
    - valeurs : -1 inconnu, 0 à 100 = probabilité d'occupation en %.
"""

from __future__ import annotations

import numpy as np

from .algos.grille import INCONNU, LIBRE, OBSTACLE, Grille

# Mêmes seuils que map_server de ROS : >= 65 % occupé, sinon libre.
SEUIL_OCCUPE = 65


def grille_depuis_occupancy(largeur: int, hauteur: int, resolution: float,
                            origine_x: float, origine_y: float, data,
                            inconnu_libre: bool = False) -> Grille:
    """
    inconnu_libre : si True, les zones encore inexplorées sont traitées comme
    libres (utile pendant le SLAM, quand la carte est incomplète). Par défaut
    elles sont interdites, ce qui est plus prudent.
    """
    brut = np.asarray(data, dtype=np.int16).reshape(hauteur, largeur)
    grille = np.full_like(brut, LIBRE)
    grille[brut >= SEUIL_OCCUPE] = OBSTACLE
    if not inconnu_libre:
        grille[brut < 0] = INCONNU
    return Grille(grille, resolution, (origine_x, origine_y))


def chemin_en_metres(grille: Grille, chemin) -> list[tuple[float, float]]:
    """Liste de points (ligne, colonne) -> liste de points (x, y) en mètres."""
    return [grille.case_vers_metres(p) for p in chemin]
