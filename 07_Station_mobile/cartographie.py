"""Cartographie en temps réel (SLAM léger) à partir du LiDAR et de l'odométrie.

Principe, en deux parties :

1. **Carte d'occupation probabiliste.** La pièce est découpée en cases de 5 cm.
   Chaque case stocke un score (« log-odds ») : à chaque tour du LiDAR, les
   cases traversées par un rayon deviennent un peu plus « libres », et la case
   où le rayon s'arrête un peu plus « occupée ». Un objet qui bouge finit donc
   par disparaître de la carte, et une erreur de mesure isolée ne compte pas.

2. **Recalage par corrélation de scans (scan matching).** L'odométrie des roues
   dérive : patinage, petites erreurs d'angle. Avant d'ajouter un scan, on teste
   de petites corrections de position autour de la pose donnée par l'odométrie
   (± 10 cm, ± 4°), et on garde celle où les points du LiDAR tombent le mieux
   sur les murs déjà cartographiés. C'est le principe des SLAM de type Hector ou
   du « correlative scan matching » de Cartographer, en version simplifiée.

Repère : le point de départ du robot est (0, 0). x vers l'avant au départ, y à
gauche, comme dans ROS.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np

from algos.grille import LIBRE, OBSTACLE, Grille

INCONNU_UI, LIBRE_UI, OCCUPE_UI = 0, 1, 2


def composer(a, b):
    """Composition de poses 2D : a ∘ b (b exprimée dans le repère de a)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by,
            math.atan2(math.sin(at + bt), math.cos(at + bt)))


def inverser(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-c * ax - s * ay, s * ax - c * ay, -at)


class Carte:
    def __init__(self, resolution=0.05, taille_m=16.0, portee_max=3.5,
                 recalage=True):
        self.res = resolution
        self.n = int(round(taille_m / resolution))
        # origine = coin bas-gauche, en mètres ; le départ (0, 0) est au centre
        self.origine = (-taille_m / 2, -taille_m / 2)
        self.lo = np.zeros((self.n, self.n), dtype=np.float32)
        self.portee_max = portee_max
        self.recalage_actif = recalage
        # Correction map <- odom trouvée par le recalage
        self.correction = (0.0, 0.0, 0.0)
        self.scans_integres = 0
        self.version = 0
        self.derniere_qualite = None
        # Réglages du modèle de capteur
        self.l_occ, self.l_libre = 0.9, -0.35
        self.l_min, self.l_max = -4.0, 4.0

    # ------------------------------------------------------------------
    def vers_case(self, x, y):
        """Mètres -> (ligne, colonne). Accepte des tableaux numpy."""
        c = np.floor((np.asarray(x) - self.origine[0]) / self.res).astype(np.int32)
        l = np.floor((np.asarray(y) - self.origine[1]) / self.res).astype(np.int32)
        return l, c

    def pose_carte(self, pose_odom):
        """Pose dans le repère de la carte (odométrie corrigée)."""
        return composer(self.correction, pose_odom)

    @staticmethod
    def points_valides(angles, distances, rmin, rmax):
        a = np.asarray(angles, dtype=np.float64)
        r = np.asarray(distances, dtype=np.float64)
        ok = np.isfinite(r) & (r >= max(rmin, 0.05)) & (r <= rmax)
        return a[ok], r[ok]

    # ------------------------------------------------------------------
    def _score(self, xs, ys):
        """Somme des scores positifs (cases occupées) sous des points."""
        l, c = self.vers_case(xs, ys)
        dedans = (l >= 0) & (l < self.n) & (c >= 0) & (c < self.n)
        vals = np.zeros(l.shape, dtype=np.float32)
        vals[dedans] = self.lo[l[dedans], c[dedans]]
        return np.clip(vals, 0, None).sum(axis=-1)

    def recaler(self, pose, a, r):
        """Cherche la meilleure petite correction de la pose (scan matching)."""
        if self.scans_integres < 5 or len(r) < 40:
            return pose, None
        dur = r < self.portee_max
        a, r = a[dur], r[dur]
        if len(r) > 180:                       # sous-échantillonner : plus rapide
            pas = int(math.ceil(len(r) / 180))
            a, r = a[::pas], r[::pas]
        x0, y0, t0 = pose
        dxy = np.arange(-4, 5) * (self.res / 2)            # ±10 cm, pas 2,5 cm
        dth = np.radians(np.arange(-4, 5))                 # ±4°, pas 1°
        DX, DY, DT = np.meshgrid(dxy, dxy, dth, indexing="ij")
        DX, DY, DT = DX.ravel(), DY.ravel(), DT.ravel()
        th = t0 + DT[:, None] + a[None, :]
        xs = x0 + DX[:, None] + r[None, :] * np.cos(th)
        ys = y0 + DY[:, None] + r[None, :] * np.sin(th)
        scores = self._score(xs, ys)
        i0 = int(np.argmin(np.abs(DX) + np.abs(DY) + np.abs(DT)))   # pas de correction
        ib = int(np.argmax(scores))
        ref = scores[i0]
        qualite = float(scores[ib] / (len(r) * self.l_max))
        self.derniere_qualite = round(qualite, 3)
        # On n'accepte la correction que si elle est nettement meilleure, et si
        # le scan colle assez à la carte (sinon : pièce trop vide, on ne touche à rien).
        if scores[ib] > ref * 1.05 + 1e-6 and qualite > 0.15:
            return (x0 + DX[ib], y0 + DY[ib], t0 + DT[ib]), qualite
        return pose, qualite

    # ------------------------------------------------------------------
    def integrer(self, pose_odom, angles, distances, rmin=0.1, rmax=12.0):
        """Ajoute un tour de LiDAR. Renvoie la pose corrigée dans la carte."""
        a, r = self.points_valides(angles, distances, rmin, rmax)
        pose = self.pose_carte(pose_odom)
        if self.recalage_actif:
            nouvelle, _ = self.recaler(pose, a, r)
            if nouvelle is not pose:
                # correction = nouvelle ∘ odom⁻¹
                self.correction = composer(nouvelle, inverser(pose_odom))
                pose = nouvelle
        x, y, t = pose
        if len(r) == 0:
            return pose

        # Cases libres : le long de chaque rayon, jusqu'au point touché (ou portée max)
        r_libre = np.minimum(r, self.portee_max) - self.res
        pas = np.arange(0, self.portee_max, self.res)
        ang = t + a
        m = pas[None, :] < r_libre[:, None]
        px = x + pas[None, :] * np.cos(ang)[:, None]
        py = y + pas[None, :] * np.sin(ang)[:, None]
        l, c = self.vers_case(px[m], py[m])
        idx = self._indices_uniques(l, c)
        self.lo.flat[idx] += self.l_libre

        # Cases occupées : extrémités des rayons qui ont touché quelque chose
        touche = r < self.portee_max
        l, c = self.vers_case(x + r[touche] * np.cos(ang[touche]),
                              y + r[touche] * np.sin(ang[touche]))
        idx = self._indices_uniques(l, c)
        self.lo.flat[idx] += self.l_occ

        np.clip(self.lo, self.l_min, self.l_max, out=self.lo)
        self.scans_integres += 1
        self.version += 1
        return pose

    def _indices_uniques(self, l, c):
        ok = (l >= 0) & (l < self.n) & (c >= 0) & (c < self.n)
        return np.unique(l[ok] * self.n + c[ok])

    # ------------------------------------------------------------------
    def etats(self):
        """Tableau d'états : 0 inconnu, 1 libre, 2 occupé."""
        e = np.full(self.lo.shape, INCONNU_UI, dtype=np.uint8)
        e[self.lo < -0.7] = LIBRE_UI
        e[self.lo > 0.7] = OCCUPE_UI
        return e

    def zone_connue(self, marge_cases=10, points=()):
        """Rectangle (l0, l1, c0, c1) couvrant la partie explorée (+ points)."""
        conn = np.argwhere(self.lo != 0)
        ls = list(conn[:, 0]) if len(conn) else []
        cs = list(conn[:, 1]) if len(conn) else []
        for (x, y) in points:
            l, c = self.vers_case(x, y)
            ls.append(int(l))
            cs.append(int(c))
        if not ls:
            m = self.n // 2
            return m - marge_cases, m + marge_cases, m - marge_cases, m + marge_cases
        l0 = max(0, min(ls) - marge_cases)
        l1 = min(self.n, max(ls) + marge_cases + 1)
        c0 = max(0, min(cs) - marge_cases)
        c1 = min(self.n, max(cs) + marge_cases + 1)
        return l0, l1, c0, c1

    def grille_planification(self, points=(), inconnu_libre=True):
        """Grille (format des algos de la phase 1) recadrée sur la zone utile.

        Les cases inconnues sont considérées libres (exploration « optimiste »).
        Le robot replanifie dès que le LiDAR découvre un obstacle sur le chemin.
        """
        l0, l1, c0, c1 = self.zone_connue(marge_cases=6, points=points)
        e = self.etats()[l0:l1, c0:c1]
        d = np.where(e == OCCUPE_UI, OBSTACLE, LIBRE).astype(np.int16)
        if not inconnu_libre:
            d[e == INCONNU_UI] = OBSTACLE
        # Bord de la zone = mur (empêche de « sortir » de la carte recadrée)
        d[0, :] = d[-1, :] = d[:, 0] = d[:, -1] = OBSTACLE
        origine = (self.origine[0] + c0 * self.res, self.origine[1] + l0 * self.res)
        return Grille(d, self.res, origine)

    def pour_affichage(self):
        """Carte recadrée, prête à envoyer au téléphone."""
        l0, l1, c0, c1 = self.zone_connue(marge_cases=4)
        e = self.etats_propres()[l0:l1, c0:c1]
        return {
            "version": self.version,
            "resolution": self.res,
            "origine": [self.origine[0] + c0 * self.res, self.origine[1] + l0 * self.res],
            "largeur": int(e.shape[1]),
            "hauteur": int(e.shape[0]),
            "cases": e,          # encodé par l'appelant
        }

    def reinitialiser(self):
        self.lo[:] = 0
        self.correction = (0.0, 0.0, 0.0)
        self.scans_integres = 0
        self.version += 1

    # ------------------------------------------------------------------
    # Enregistrement et relocalisation globale
    # ------------------------------------------------------------------
    def sauver(self, chemin, infos=None):
        """Écrit la carte dans un fichier (écriture atomique : jamais de fichier à moitié écrit)."""
        os.makedirs(os.path.dirname(os.path.abspath(chemin)), exist_ok=True)
        tmp = chemin + ".tmp.npz"
        np.savez_compressed(tmp, lo=self.lo, res=np.float64(self.res),
                            origine=np.array(self.origine, dtype=np.float64),
                            infos=np.array(json.dumps(infos or {})))
        os.replace(tmp, chemin)

    def charger(self, chemin):
        """Recharge une carte enregistrée. Renvoie les infos associées (base, lieux…)."""
        with np.load(chemin, allow_pickle=False) as f:
            lo = f["lo"].astype(np.float32)
            res = float(f["res"])
            origine = tuple(float(v) for v in f["origine"])
            infos = json.loads(str(f["infos"]))
        if lo.shape != self.lo.shape or abs(res - self.res) > 1e-9 or origine != tuple(self.origine):
            raise ValueError("carte enregistrée incompatible (taille ou résolution différente)")
        self.lo[:] = lo
        self.scans_integres = 1000            # carte déjà riche : le recalage est actif tout de suite
        self.version += 1
        return infos

    def _champ_vraisemblance(self):
        """1 sur les murs, 0,5 juste à côté : un point du LiDAR un peu décalé compte encore."""
        occ = self.lo > 0.7
        champ = occ.astype(np.float32)
        voisin = np.zeros_like(occ)
        voisin[1:, :] |= occ[:-1, :]
        voisin[:-1, :] |= occ[1:, :]
        voisin[:, 1:] |= occ[:, :-1]
        voisin[:, :-1] |= occ[:, 1:]
        champ[voisin & ~occ] = 0.5
        # Un point du LiDAR qui tombe en plein espace libre déjà vu est une preuve CONTRE
        # la pose testée : sans cette pénalité, une mauvaise pose peut « coller » par hasard.
        champ[(self.lo < -0.7) & ~voisin & ~occ] = -0.5
        return champ

    def _scores(self, champ, xs, ys, a, r, th):
        """Score de chaque pose candidate (xs, ys, th), pour les points (a, r) du scan."""
        X = xs[:, None] + r[None, :] * np.cos(th[:, None] + a[None, :])
        Y = ys[:, None] + r[None, :] * np.sin(th[:, None] + a[None, :])
        l, c = self.vers_case(X, Y)
        dedans = (l >= 0) & (l < self.n) & (c >= 0) & (c < self.n)
        vals = np.zeros(l.shape, dtype=np.float32)
        vals[dedans] = champ[l[dedans], c[dedans]]
        return vals.sum(axis=1)

    def localiser_global(self, angles, distances, rmin=0.1, rmax=12.0):
        """Retrouve la position du robot sur la carte, SANS indice de départ.

        On essaie toutes les positions libres connues (tous les 10 cm) et toutes les
        orientations (tous les 4°), on garde celle où le scan colle le mieux aux murs,
        puis on l'affine à 2,5 cm et 1° près. C'est une recherche exhaustive, comme la
        « relocalisation globale » d'AMCL, en version déterministe.

        Renvoie (pose, qualite, ambigu) ou None. `ambigu` = une autre position, loin de
        la première, colle presque aussi bien (pièce symétrique) : il faut une autre vue.
        """
        a, r = self.points_valides(angles, distances, rmin, rmax)
        garde = r < self.portee_max
        a, r = a[garde], r[garde]
        if len(r) < 30:
            return None
        if len(r) > 90:
            pas = int(math.ceil(len(r) / 90))
            a, r = a[::pas], r[::pas]
        champ = self._champ_vraisemblance()
        e = self.etats()
        ll, cc = np.nonzero(e == LIBRE_UI)
        garde = (ll % 2 == 0) & (cc % 2 == 0)              # une case sur deux : 10 cm
        ll, cc = ll[garde], cc[garde]
        if len(ll) == 0:
            return None
        xs = self.origine[0] + (cc + 0.5) * self.res
        ys = self.origine[1] + (ll + 0.5) * self.res
        orientations = np.radians(np.arange(0, 360, 4))
        meilleurs = []                                       # (score, x, y, th) par orientation
        for th in orientations:
            s = self._scores(champ, xs, ys, a, r, np.full(len(xs), th))
            i = int(np.argmax(s))
            meilleurs.append((float(s[i]), float(xs[i]), float(ys[i]), float(th), s))
        k = int(np.argmax([m[0] for m in meilleurs]))
        score, x0, y0, t0, _ = meilleurs[k]
        # Second candidat : la meilleure pose « différente » (à plus de 0,5 m, ou orientation à plus de 25°)
        second = 0.0
        for s_, _, _, th, s in meilleurs:
            dth = abs(math.atan2(math.sin(th - t0), math.cos(th - t0)))
            if dth > math.radians(25):
                second = max(second, s_)
            else:
                loin = (xs - x0) ** 2 + (ys - y0) ** 2 > 0.5 ** 2
                if loin.any():
                    second = max(second, float(s[loin].max()))
        # Affinage autour du meilleur
        dxy = np.arange(-4, 5) * (self.res / 2)
        dth = np.radians(np.arange(-4, 5))
        DX, DY, DT = (v.ravel() for v in np.meshgrid(dxy, dxy, dth, indexing="ij"))
        s = self._scores(champ, x0 + DX, y0 + DY, a, r, t0 + DT)
        i = int(np.argmax(s))
        pose = (x0 + float(DX[i]), y0 + float(DY[i]),
                math.atan2(math.sin(t0 + DT[i]), math.cos(t0 + DT[i])))
        qualite = float(s[i]) / len(r)
        return pose, round(qualite, 3), second > 0.9 * float(s[i])

    # ------------------------------------------------------------------
    # Carte venue de slam_toolbox (option --slam)
    # ------------------------------------------------------------------
    def fusionner_grille(self, res, origine, largeur, hauteur, donnees, decalage=(0.0, 0.0, 0.0)):
        """Recopie une carte nav_msgs/OccupancyGrid (celle de slam_toolbox) dans la nôtre.

        `origine` = (x, y, cap) du coin bas-gauche de la grille ROS ; `donnees` = valeurs
        -1 (inconnu) ou 0..100 (probabilité d'obstacle), ligne par ligne depuis le bas.
        `decalage` = pose du repère de slam_toolbox dans NOTRE repère (identité en temps
        normal ; utile quand slam_toolbox a redémarré sur une carte rechargée).
        Les cases inconnues pour slam_toolbox gardent ce que l'on savait déjà : une carte
        rechargée n'est donc pas effacée par une nouvelle session qui n'a encore rien vu.
        """
        g = np.asarray(donnees, dtype=np.int16).reshape(hauteur, largeur)
        # Coins de la grille ROS dans notre repère -> zone de nos cases à remplir
        T = composer(decalage, origine)
        coins = [composer(T, (cx, cy, 0.0))[:2] for cx in (0, largeur * res) for cy in (0, hauteur * res)]
        xs, ys = [c[0] for c in coins], [c[1] for c in coins]
        l0, c0 = self.vers_case(min(xs), min(ys))
        l1, c1 = self.vers_case(max(xs), max(ys))
        l0, c0 = max(0, int(l0)), max(0, int(c0))
        l1, c1 = min(self.n, int(l1) + 1), min(self.n, int(c1) + 1)
        if l1 <= l0 or c1 <= c0:
            return 0
        L, C = np.mgrid[l0:l1, c0:c1]
        X = self.origine[0] + (C + 0.5) * self.res
        Y = self.origine[1] + (L + 0.5) * self.res
        # Centre de nos cases -> repère de la grille ROS (transformation inverse)
        ix, iy, it = inverser(T)
        cs, sn = math.cos(it), math.sin(it)
        gx = ix + cs * X - sn * Y
        gy = iy + sn * X + cs * Y
        gc = np.floor(gx / res).astype(np.int32)
        gl = np.floor(gy / res).astype(np.int32)
        dedans = (gl >= 0) & (gl < hauteur) & (gc >= 0) & (gc < largeur)
        v = np.full(X.shape, -1, dtype=np.int16)
        v[dedans] = g[gl[dedans], gc[dedans]]
        zone = self.lo[l0:l1, c0:c1]
        zone[(v >= 0) & (v <= 25)] = self.l_min           # libre
        zone[v >= 65] = self.l_max                        # mur
        zone[(v > 25) & (v < 65)] = 0.0                   # incertain
        self.scans_integres = max(self.scans_integres, 1)
        self.version += 1
        return int((v >= 0).sum())

    # ------------------------------------------------------------------
    # Carte « présentable » et formats standards
    # ------------------------------------------------------------------
    def etats_propres(self):
        """États pour l'AFFICHAGE : sans les points de bruit isolés, sans petits trous.

        La planification, elle, garde la carte brute (un pied de chaise reste un obstacle).
        - un point de mur tout seul (aucun voisin mur) au milieu du libre disparaît ;
        - une case inconnue entourée de libre (≥ 6 voisins sur 8) devient libre ;
        - un trou d'une case dans un mur est bouché (mur à gauche et à droite, ou dessus/dessous).
        """
        e = self.etats()
        occ = e == OCCUPE_UI
        lib = e == LIBRE_UI
        inc = e == INCONNU_UI

        def voisins(m):
            p = np.pad(m.astype(np.int8), 1)
            return sum(p[1 + dl:p.shape[0] - 1 + dl, 1 + dc:p.shape[1] - 1 + dc]
                       for dl in (-1, 0, 1) for dc in (-1, 0, 1) if dl or dc)

        n_occ, n_lib = voisins(occ), voisins(lib)
        sortie = e.copy()
        sortie[occ & (n_occ == 0)] = LIBRE_UI
        sortie[inc & (n_lib >= 6)] = LIBRE_UI
        p = np.pad(occ, 1)
        trou = ~occ & ((p[1:-1, :-2] & p[1:-1, 2:]) | (p[:-2, 1:-1] & p[2:, 1:-1]))
        sortie[trou & (n_occ >= 2)] = OCCUPE_UI
        return sortie

    def exporter_pgm(self, chemin_sans_ext, nom=""):
        """Écrit la carte au format standard ROS (map_server / Nav2 / RViz) : .pgm + .yaml."""
        l0, l1, c0, c1 = self.zone_connue(marge_cases=4)
        e = self.etats()[l0:l1, c0:c1][::-1]               # PGM : première ligne = en haut
        img = np.full(e.shape, 205, dtype=np.uint8)        # gris = inconnu
        img[e == LIBRE_UI] = 254
        img[e == OCCUPE_UI] = 0
        with open(chemin_sans_ext + ".pgm", "wb") as f:
            f.write(f"P5\n# {nom or 'carte'} - station TurtleBot3\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
            f.write(img.tobytes())
        ox = self.origine[0] + c0 * self.res
        oy = self.origine[1] + l0 * self.res
        with open(chemin_sans_ext + ".yaml", "w", encoding="utf-8") as f:
            f.write(f"image: {os.path.basename(chemin_sans_ext)}.pgm\nmode: trinary\n"
                    f"resolution: {self.res}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
