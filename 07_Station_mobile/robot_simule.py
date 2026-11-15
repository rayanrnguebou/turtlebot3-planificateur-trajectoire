"""Robot virtuel pour essayer la station sans le vrai TurtleBot3.

Il imite le Burger : unicycle, LiDAR 400 points à 10 Hz (comme le LDS-03) avec
du bruit et des mesures manquantes, et une odométrie qui DÉRIVE (erreur de
2 % sur la distance, 3 % sur les rotations), pour que le recalage de la
cartographie ait un vrai travail à faire. La batterie simulée se vide
doucement. Tout ce qui est simulé est signalé par le badge SIMULATION.
"""

from __future__ import annotations

import math
import random
import threading
import time


def _boite(cx, cy, dx, dy):
    p = [(cx - dx, cy - dy), (cx + dx, cy - dy), (cx + dx, cy + dy), (cx - dx, cy + dy)]
    return [(*p[i], *p[(i + 1) % 4]) for i in range(4)]


def salle_de_labo():
    """Pièce de 5 m x 4 m, avec tables, armoire et un muret (en mètres)."""
    murs = [(-1.0, -2.0, 4.0, -2.0), (4.0, -2.0, 4.0, 2.0),
            (4.0, 2.0, -1.0, 2.0), (-1.0, 2.0, -1.0, -2.0)]
    return (murs
            + _boite(1.4, 0.0, 0.25, 0.6)        # table au milieu
            + _boite(3.0, 1.4, 0.4, 0.25)        # armoire
            + _boite(2.6, -1.2, 0.2, 0.2)        # carton
            + [(0.4, 2.0, 0.4, 1.0)])            # muret


# Endroits libres de la salle simulée, pour poser le robot « n'importe où »
DEPARTS_LIBRES = [(0.0, 0.0, 0.0), (2.3, 0.3, 3.0), (0.3, -1.3, 1.2),
                  (3.4, -0.4, -2.0), (-0.4, 1.2, -0.8), (2.0, 1.0, 0.0)]


class RobotSimule(threading.Thread):
    def __init__(self, cerveau, derive=True, vitesse_temps=1.0, depart=(0.0, 0.0, 0.0)):
        super().__init__(daemon=True)
        self.cerveau = cerveau
        self.vitesse_temps = vitesse_temps
        # Pose réelle. L'odométrie, elle, démarre toujours à (0, 0, 0), comme sur le
        # vrai robot : poser le robot ailleurs teste la relocalisation.
        self.x, self.y, self.th = depart
        self.ox = self.oy = self.oth = 0.0              # pose estimée (odométrie)
        self.err_v, self.err_w = (1.02, 0.97) if derive else (1.0, 1.0)
        self.tension = 12.2
        self.segments_fixes = salle_de_labo()
        self.obstacles_ajoutes = []
        self.verrou = threading.Lock()
        self.arreter = threading.Event()
        self.collisions = 0

    # -------- obstacle imprévu (bouton de la simulation) --------
    def poser_obstacle(self, distance=0.6, demi_taille=0.12):
        """Pose une boîte devant le robot, sur sa trajectoire."""
        cx = self.x + distance * math.cos(self.th)
        cy = self.y + distance * math.sin(self.th)
        with self.verrou:
            self.obstacles_ajoutes.append(_boite(cx, cy, demi_taille, demi_taille))
        return cx, cy

    def retirer_obstacles(self):
        with self.verrou:
            self.obstacles_ajoutes.clear()

    def segments(self):
        with self.verrou:
            return self.segments_fixes + [s for b in self.obstacles_ajoutes for s in b]

    # -------- géométrie --------
    @staticmethod
    def _rayon(x, y, angle, segments):
        dx, dy = math.cos(angle), math.sin(angle)
        meilleur = math.inf
        for x1, y1, x2, y2 in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
            if t > 0 and 0 <= u <= 1 and t < meilleur:
                meilleur = t
        return meilleur

    @staticmethod
    def _collision(x, y, segments, rayon=0.105):
        for x1, y1, x2, y2 in segments:
            ex, ey = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey)))
            if math.hypot(x - (x1 + t * ex), y - (y1 + t * ey)) < rayon:
                return True
        return False

    # -------- boucle --------
    def run(self):
        dt = 0.05
        n = 400
        inc = 2 * math.pi / n
        k = 0
        v = w = 0.0
        while not self.arreter.is_set():
            if k % 2 == 0:
                try:
                    v, w = self.cerveau.cycle()          # 10 Hz, comme sur le robot
                except Exception as e:                   # comme pont_ros : stop, et on continue
                    print(f"Erreur dans le cerveau : {e!r} (robot arrêté pour ce cycle)")
                    v, w = 0.0, 0.0
            segs = self.segments()
            nx = self.x + v * math.cos(self.th) * dt
            ny = self.y + v * math.sin(self.th) * dt
            if self._collision(nx, ny, segs):
                if v != 0:
                    self.collisions += 1
                v_reel = 0.0
            else:
                self.x, self.y, v_reel = nx, ny, v
            self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))
            # Odométrie avec erreur systématique
            vo, wo = v_reel * self.err_v, w * self.err_w
            self.ox += vo * math.cos(self.oth) * dt
            self.oy += vo * math.sin(self.oth) * dt
            self.oth = math.atan2(math.sin(self.oth + wo * dt), math.cos(self.oth + wo * dt))
            self.cerveau.recevoir_odom(self.ox, self.oy, self.oth, vo, wo)
            if k % 2 == 1:                                # LiDAR à 10 Hz
                d = []
                for i in range(n):
                    r = self._rayon(self.x, self.y, self.th + i * inc, segs)
                    if random.random() < 0.03 or r > 8.0:
                        r = 0.0                           # pas de retour
                    else:
                        r += random.gauss(0, 0.006)
                    d.append(r)
                self.cerveau.recevoir_scan(0.0, inc, 0.1, 12.0, d)
            if k % 20 == 0:
                self.tension -= 0.0004 + 0.004 * (abs(v_reel) + 0.05 * abs(w))
                self.cerveau.recevoir_batterie(max(self.tension, 9.8))
            k += 1
            time.sleep(dt / self.vitesse_temps)

    def etat_reel(self):
        return {"pose_reelle": [round(self.x, 3), round(self.y, 3), round(self.th, 3)],
                "obstacles": [[round(b[0][0], 3), round(b[0][1], 3), round(b[1][0], 3),
                               round(b[2][1], 3)] for b in self.obstacles_ajoutes],
                "collisions": self.collisions}
