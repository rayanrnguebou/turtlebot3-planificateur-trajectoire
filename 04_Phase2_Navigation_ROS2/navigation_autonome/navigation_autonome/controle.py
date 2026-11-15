"""Suivi de trajectoire par « Pure Pursuit » (poursuite pure).

Principe : on vise un point du chemin situé à une distance fixe devant le
robot (la distance de visée). On calcule l'arc de cercle qui y mène, ce qui
donne directement la vitesse de rotation. C'est simple, stable, et très
utilisé sur les robots à deux roues comme le Burger.

Sans import ROS : testable sous Windows.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Point = tuple[float, float]


def angle_normalise(a: float) -> float:
    """Ramène un angle dans ]-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def densifier(chemin: list[Point], pas: float) -> list[Point]:
    """Ajoute des points intermédiaires pour qu'ils soient espacés d'au plus `pas`."""
    if len(chemin) < 2:
        return list(chemin)
    dense = [chemin[0]]
    for (x0, y0), (x1, y1) in zip(chemin, chemin[1:]):
        n = max(1, math.ceil(math.dist((x0, y0), (x1, y1)) / pas))
        dense.extend((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(1, n + 1))
    return dense


@dataclass
class Commande:
    v: float        # vitesse linéaire (m/s)
    w: float        # vitesse angulaire (rad/s)
    arrive: bool = False


@dataclass
class PurePursuit:
    distance_visee: float = 0.25     # m
    v_max: float = 0.15              # m/s (le Burger accepte 0.22 au maximum)
    w_max: float = 1.5               # rad/s (maximum du Burger : 2.84)
    tolerance_arrivee: float = 0.08  # m
    angle_rotation_sur_place: float = math.radians(60)

    def __post_init__(self) -> None:
        self.chemin: list[Point] = []
        self._indice = 0

    def nouveau_chemin(self, chemin: list[Point], pas: float = 0.05) -> None:
        # Le chemin lissé a de longs segments : on le rééchantillonne tous les
        # `pas` mètres, sinon le point visé pourrait sauter en arrière.
        self.chemin = densifier(chemin, pas)
        self._indice = 0

    def _point_vise(self, x: float, y: float) -> Point:
        # Avancer l'indice jusqu'au point du chemin le plus proche (on ne recule jamais).
        meilleur = self._indice
        d_min = math.dist((x, y), self.chemin[meilleur])
        # Fenêtre limitée : évite de « sauter » vers une partie du chemin qui repasse près du robot.
        for i in range(self._indice, min(len(self.chemin), self._indice + 40)):
            d = math.dist((x, y), self.chemin[i])
            if d < d_min:
                meilleur, d_min = i, d
        self._indice = meilleur

        # Premier point au-delà de la distance de visée, sinon le dernier.
        for i in range(self._indice, len(self.chemin)):
            if math.dist((x, y), self.chemin[i]) >= self.distance_visee:
                return self.chemin[i]
        return self.chemin[-1]

    def commande(self, x: float, y: float, cap: float) -> Commande:
        """Pose du robot (x, y en m, cap en rad) -> vitesses à envoyer."""
        if not self.chemin:
            return Commande(0.0, 0.0, arrive=True)

        but = self.chemin[-1]
        d_but = math.dist((x, y), but)
        if d_but < self.tolerance_arrivee:
            return Commande(0.0, 0.0, arrive=True)

        cx, cy = self._point_vise(x, y)
        alpha = angle_normalise(math.atan2(cy - y, cx - x) - cap)

        # Cible trop sur le côté ou derrière : on tourne sur place d'abord.
        if abs(alpha) > self.angle_rotation_sur_place:
            return Commande(0.0, math.copysign(self.w_max * 0.6, alpha))

        # Ralentir dans les virages et à l'approche du but.
        v = self.v_max * max(0.3, math.cos(alpha))
        v = min(v, max(0.05, d_but))
        L = max(math.dist((x, y), (cx, cy)), 1e-3)
        w = 2.0 * v * math.sin(alpha) / L          # courbure de l'arc × vitesse
        w = max(-self.w_max, min(self.w_max, w))
        return Commande(v, w)


def obstacle_devant(distances: list[float], angle_min: float, increment: float,
                    distance_securite: float, demi_angle: float = math.radians(30)) -> bool:
    """True si un point du LiDAR est trop près dans le cône avant du robot.

    distances : ranges du message sensor_msgs/LaserScan (0 ou inf = pas de mesure).
    """
    for i, d in enumerate(distances):
        if not (0.0 < d < distance_securite) or math.isinf(d) or math.isnan(d):
            continue
        a = angle_normalise(angle_min + i * increment)
        if abs(a) <= demi_angle:
            return True
    return False
