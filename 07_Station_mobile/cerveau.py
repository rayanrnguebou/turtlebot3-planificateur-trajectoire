"""Le « cerveau » du robot : carte, mission autonome, sécurité, paroles.

Il ne dépend pas de ROS : il reçoit les mesures (LiDAR, odométrie, batterie)
et renvoie la vitesse à appliquer, 10 fois par seconde. Le même code tourne
donc sur le vrai robot (via pont_ros.py) et dans la simulation.

Machine à états de la mission :

    attente ──(destination)──> planification ──(chemin trouvé)──> en_route
                                     ^                               │
                                     │                 obstacle sur le chemin
                                     │                               v
                                     └──(toujours là après 2,5 s)── obstacle
                                                                     │
                                                  (parti tout seul) └──> en_route
    obstacle collé devant, ou aucun chemin ──> recul (20 cm en arrière) ──> planification
    en_route ──(à moins de 8 cm du but)──> arrive
    planification ──(aucun chemin, même après 3 reculs)──> echec
    joystick touché ──> manuel        bouton ARRÊT ──> arrêt d'urgence (prioritaire)

Carte enregistrée : au démarrage suivant, la carte est rechargée et le robot
retrouve SEUL sa position dessus, où qu'on l'ait posé (relocalisation globale,
voir Carte.localiser_global). Tant qu'il ne s'est pas retrouvé, il ne modifie
pas la carte et refuse les missions ; le joystick reste utilisable.
"""

from __future__ import annotations

import base64
import itertools
import math
import os
import re
import random
import threading
import time
from collections import deque
from datetime import datetime

from cartographie import Carte, composer, inverser
from exploration import prochaine_frontiere
from controle import PurePursuit, densifier
from navigation import (ALGOS, Plan, chemin_traverse_obstacle, longueur,
                        obstacle_sur_chemin, planifier, points_lidar_monde)

# --- Sécurité ---------------------------------------------------------------
V_MAX_DEFAUT = 0.22            # plafond absolu (réglable au lancement, --vmax)
W_MAX_DEFAUT = 1.8
# Vitesses choisies sur la manette : niveau -> (m/s, rad/s)
NIVEAUX_VITESSE = {1: (0.10, 0.7), 2: (0.15, 1.1), 3: (0.22, 1.8)}   # 0.22 m/s = max du Burger
DELAI_HOMME_MORT = 0.5          # s sans commande du joystick -> arrêt
DISTANCE_URGENCE = 0.15         # m : obstacle collé devant -> stop immédiat
RAYON_SECURITE = 0.125          # m : rayon du Burger (≈ 0,105 m) + 2 cm, pour la bulle de sécurité
DISTANCE_MANUEL = 0.20          # m : en manuel, interdiction d'avancer (ou de reculer) vers un obstacle
FREIN_MANUEL = 1.0              # en manuel, vitesse max = (distance - 0.20 m) x 1.0 : freinage progressif
V_SANS_LIDAR = 0.05             # m/s : en manuel sans LiDAR récent, on roule au pas
DEMI_CONE = math.radians(20)
ATTENTE_OBSTACLE = 2.5          # s avant de contourner un obstacle immobile
LIBRE_DEPUIS = 1.0              # s de voie libre avant de repartir
VISEE_OBSTACLE = 0.90           # m de chemin surveillés devant le robot (on prévient tôt)
ALERTE_MANUEL = 0.50            # m : en manuel, on prévient dès qu'un obstacle est à cette distance
RAYON_ROBOT = 0.10              # m : distances annoncées depuis le bord du robot, pas son centre
EN_MISSION = ("planification", "en_route", "obstacle", "recul")

# --- Dégagement : marche arrière automatique quand il est coincé -------------
RECUL_DISTANCE = 0.20           # m reculés avant de recalculer un chemin
RECUL_VITESSE = 0.06            # m/s, doucement
RECUL_PLACE_MINI = 0.35         # m libres derrière (couloir balayé) pour oser reculer
RECULS_MAX = 3                  # par mission
OBSTACLE_PROCHE = 0.25          # m : obstacle trop près devant pour repartir en avant
BLOCAGES_MAX = 4                # obstacles d'affilée sans se rapprocher du but -> on abandonne
PROGRES_MINI = 0.15             # m gagnés vers le but pour compter comme un progrès

# --- Exploration autonome -----------------------------------------------------
EXPLORATION_ETAPES_MAX = 40
EXPLORATION_LIMITE_ZONE = 60.0  # s max pour atteindre une frontière, sinon on passe à la suivante

# --- Relocalisation sur une carte enregistrée --------------------------------
QUALITE_MINI = 0.55             # part (pondérée) des points du LiDAR qui tombent sur un mur
ECART_VUES = math.radians(30)   # deux vues d'accord doivent être prises sous des angles différents
ROTATIONS_LOC_MAX = 8           # 8 x 45° = un tour complet, puis on demande de l'aide
ACCORD_POSITION = (0.15, math.radians(8))   # deux essais « d'accord » : moins de 15 cm et 8° d'écart
PERIODE_LOCALISATION = 1.0      # s entre deux essais
LIEU_NOM_MAX = 24

# --- Batterie LiPo 3S (11,1 V) ----------------------------------------------
TENSION_PLEINE, TENSION_VIDE = 12.6, 10.0
TENSION_ALERTE, TENSION_MINI_MISSION = 11.0, 10.6

PHRASES = {
    "depart": ["C'est parti !", "En route !", "J'y vais !"],
    "obstacle": ["Oh ! Attention, obstacle !", "Oups ! Un obstacle.", "Attention !",
                 "Oh là ! Obstacle devant."],
    "obstacle_loin": ["Attention, obstacle devant. Je ralentis."],
    "reprise": ["C'est bon, la voie est libre.", "Je repars."],
    "contourne": ["Je contourne l'obstacle.", "Je cherche un autre chemin."],
    "itineraire": ["Je change d'itinéraire.", "Je prends un autre chemin."],
    "arrive": ["Arrivé !", "Je suis arrivé !", "Destination atteinte !"],
    "arrive_proche": ["Je suis au plus près du but.", "Je ne peux pas aller plus loin, je suis au plus près."],
    "au_plus_pres": ["Je ne peux pas aller jusque-là. Je vais au plus près.",
                     "Pas de passage jusqu'à ce point : je m'approche au maximum."],
    "etroit": ["Passage étroit, j'y vais doucement."],
    "echec": ["Je suis bloqué, je ne trouve pas de chemin."],
    "batterie": ["Attention, batterie faible."],
    "batterie_vide": ["Batterie trop faible, je ne pars pas."],
    "urgence": ["Arrêt d'urgence !"],
    "reprendre": ["Arrêt d'urgence levé."],
    "manuel": ["Mode manuel."],
    "base": ["Je rentre à la base."],
    "annule": ["Mission annulée."],
    "carte": ["Carte effacée."],
    "cherche_position": ["Carte chargée. Je cherche où je suis."],
    "position_trouvee": ["Je sais où je suis !", "Position retrouvée sur la carte."],
    "tourne_position": ["Je tourne sur moi-même pour me repérer."],
    "aide_position": ["Je ne suis pas sûr de ma position. Fais-moi tourner un peu sur place, ou avance-moi."],
    "inconnu": ["Je ne reconnais pas cet endroit. Si c'est une nouvelle pièce, efface la carte."],
    "pas_localise": ["Attends, je cherche encore où je suis."],
    "carte_enregistree": ["Carte enregistrée."],
    "lieu": ["C'est noté."],
    "recul": ["Je recule pour mieux passer.", "Je me dégage."],
    "exploration": ["J'explore la pièce tout seul."],
    "zone": ["Je vais voir par là.", "J'explore cette zone.", "Zone suivante."],
    "bloque_proche": ["Je n'arrive pas à passer. Je m'arrête au plus près.",
                      "Passage impossible, je reste au plus près."],
    "exploration_finie": ["Exploration terminée, j'ai fait le tour !"],
    "exploration_impossible": ["Je n'ai rien de nouveau à explorer d'ici."],
}


def pourcentage_batterie(tension):
    if tension is None or tension <= 0:
        return None
    p = (tension - TENSION_VIDE) / (TENSION_PLEINE - TENSION_VIDE) * 100.0
    return round(max(0.0, min(100.0, p)))


def angle_normalise(a):
    return math.atan2(math.sin(a), math.cos(a))


def distance_devant(angles, distances, rmin=0.1, demi_cone=DEMI_CONE):
    """Plus petite distance valide dans le cône avant, ou None."""
    meilleur = None
    for a, r in zip(angles, distances):
        if r is None or not math.isfinite(r) or r < max(rmin, 0.05):
            continue
        if abs(angle_normalise(a)) <= demi_cone and (meilleur is None or r < meilleur):
            meilleur = r
    return meilleur


def distance_couloir(angles, distances, rmin=0.1, sens=1, demi_largeur=0.14):
    """Distance libre dans le couloir que le robot va balayer (en avant si sens=1,
    en arrière si sens=-1) : voit aussi un mur longé en biais, pas seulement de face."""
    meilleur = None
    for a, r in zip(angles, distances):
        if r is None or not math.isfinite(r) or r < max(rmin, 0.05):
            continue
        x, y = r * math.cos(a) * sens, r * math.sin(a)
        if x > 0 and abs(y) < demi_largeur and (meilleur is None or x < meilleur):
            meilleur = x
    return meilleur


def decrire_obstacle(distance_centre, angle):
    """« à 35 cm, devant à gauche » : distance depuis le bord du robot, arrondie à 5 cm."""
    cm = max(5, int(round(max(0.0, distance_centre - RAYON_ROBOT) * 100 / 5)) * 5)
    a = math.degrees(angle_normalise(angle))
    cote = "à gauche" if a > 0 else "à droite"
    if abs(a) <= 15:
        ou = "devant"
    elif abs(a) <= 60:
        ou = "devant " + cote
    elif abs(a) <= 120:
        ou = cote
    elif abs(a) <= 165:
        ou = "derrière " + cote
    else:
        ou = "derrière"
    return f"à {cm} cm, {ou}"


def contact_prevu(angles, distances, v, w, rmin=0.1, horizon=0.6, rayon=RAYON_SECURITE):
    """Le mouvement (v, w) va-t-il amener le bord du robot contre un point du LiDAR ?

    On prolonge l'arc de cercle suivi par le robot pendant `horizon` secondes et on mesure,
    à chaque instant, la distance entre le centre du robot et le point du LiDAR le plus proche.
    Contrairement au cône avant, cela voit aussi un coin de meuble frôlé SUR LE CÔTÉ dans un virage
    (c'est là qu'une petite erreur de position fait toucher). Renvoie (distance, angle) du point
    menaçant, ou None. Un mouvement qui ÉLOIGNE du point (pour se dégager) est toujours permis.
    """
    if abs(v) < 1e-3:
        return None
    pts = []
    for a, r in zip(angles, distances):
        if r is not None and math.isfinite(r) and max(rmin, 0.05) <= r < 1.0:
            pts.append((r * math.cos(a), r * math.sin(a), r, a))
    if not pts:
        return None
    marge_maintenant = min(p[2] for p in pts)
    pire = None
    for k in range(1, 7):
        s = horizon * k / 6
        if abs(w) < 1e-3:
            x, y = v * s, 0.0
        else:
            x, y = v / w * math.sin(w * s), v / w * (1 - math.cos(w * s))
        for px, py, r, a in pts:
            d = math.hypot(px - x, py - y)
            if d < rayon and d < marge_maintenant - 0.005 and (pire is None or d < pire[0]):
                pire = (d, r, a)
    return None if pire is None else (pire[1], pire[2])


def obstacle_le_plus_proche(angles, distances, rmin=0.1, sens=1, demi_largeur=0.14):
    """(distance, angle) du point le plus proche dans le couloir avant (sens=1) ou arrière."""
    meilleur = None
    for a, r in zip(angles, distances):
        if r is None or not math.isfinite(r) or r < max(rmin, 0.05):
            continue
        x, y = r * math.cos(a) * sens, r * math.sin(a)
        if x > 0 and abs(y) < demi_largeur and (meilleur is None or r < meilleur[0]):
            meilleur = (r, a)
    return meilleur


def nom_de_carte(nom):
    """« Salle 330 ! » -> « Salle_330 » : un nom de fichier sûr (lettres, chiffres, - et _)."""
    nom = re.sub(r"[^\w\-]+", "_", str(nom).strip(), flags=re.UNICODE).strip("_")
    return nom[:40]


def lire_infos_carte(chemin):
    import json
    import numpy as np
    with np.load(chemin, allow_pickle=False) as f:
        return json.loads(str(f["infos"]))


def memoriser_derniere_carte(chemin):
    """Retient la dernière carte utilisée : c'est elle qui revient au prochain démarrage."""
    try:
        with open(os.path.join(os.path.dirname(os.path.abspath(chemin)), "derniere_carte.txt"),
                  "w", encoding="utf-8") as f:
            f.write(os.path.basename(chemin))
    except OSError:
        pass


class Cerveau:
    def __init__(self, v_max=V_MAX_DEFAUT, w_max=W_MAX_DEFAUT, mode="robot",
                 recalage=True, son=None):
        self.verrou = threading.RLock()
        self.plafond_v, self.plafond_w, self.mode_station = v_max, w_max, mode
        self.decalage_lidar = 0.0        # rad, option --lidar-decalage (LiDAR monté tourné)
        self.miroir_lidar = False        # option --lidar-miroir (gauche et droite inversées)
        self.niveau_vitesse = 2
        self.son = son or (lambda type_evenement: None)     # buzzer du robot
        self.carte = Carte(recalage=recalage)
        # Capteurs
        self.scan = None                 # (angles, distances, rmin, rmax)
        self.scan_nouveau = False
        self.instant_scan = None
        self.pose_odom = None
        self.instant_odom = None
        self.vitesse = (0.0, 0.0)
        self.tension = None
        self.instant_batterie = None
        self.alerte_batterie_donnee = False
        self.derniere_integration = 0.0
        # Mission
        self.etat = "attente"
        self.arret_urgence = False
        self.algo = "A*"
        self.but = None
        self.plan: Plan | None = None
        self.plan_dense = []
        self.suivi = PurePursuit()
        self._appliquer_vitesse()
        self.fil_planif = None
        self.resultat_planif = None
        self.echecs_planif = 0
        self.raison_planif = ""
        self.instant_obstacle = None
        self.instant_libre = None
        self.instant_verif = 0.0
        self.mission = None              # statistiques de la mission en cours
        self.bilan = None                # bilan de la dernière mission
        self.comparaison = None
        self.depart_base = (0.0, 0.0)
        # Manuel
        self.demande = (0.0, 0.0)
        self.instant_demande = None
        # Affichage
        self.trace = []
        self.commande = (0.0, 0.0)
        self.obstacle_devant = None
        self.couloir_avant = self.couloir_arriere = None
        self.instant_alerte_manuel = 0.0
        self.alerte_proche_donnee = False   # manuel : « obstacle à 45 cm » déjà annoncé pour cette approche
        self.marge_chemin = 0.12
        # Carte enregistrée, relocalisation, lieux
        self.fichier_carte = None        # chemin du fichier (donné par la station)
        # slam_toolbox (option --slam) : None = notre cartographie maison
        #   T : pose du repère de slam_toolbox dans NOTRE repère (None = à calculer au 1er TF)
        #   S : dernière transformation map -> odom publiée par slam_toolbox
        self.slam = None
        self.demandes_slam = []          # actions pour pont_ros : ("lancer", fichier, pose) ...
        self.carte_enregistree = None    # date du dernier enregistrement
        self.lieux = {}                  # nom -> (x, y) sur la carte
        self.localise = True             # False après le chargement d'une carte
        self.fil_loc = None
        self.resultat_loc = None
        self.generation_loc = 0
        self.vues_loc = []               # [(correction, cap odométrie)] des vues déjà analysées
        self.rotation_loc = None         # cap visé pendant un quart de tour de repérage
        self.tours_loc = 0
        self.aide_loc_donnee = False
        self.essais_loc = 0
        self.instant_loc = 0.0
        self.derniere_loc = None         # infos du dernier essai (pour l'écran)
        # Dégagement et exploration
        self.recul = None                # {"depart": (x, y), "debut": t}
        self.exploration = None          # {"interdits": [...], "etapes": n, "suite": t}
        self.evenements = deque(maxlen=40)
        self._ids = itertools.count(1)
        self._cache_carte = None

    # ===================================================================
    # Mesures reçues des capteurs
    # ===================================================================
    def recevoir_scan(self, angle_min, increment, rmin, rmax, distances):
        n = len(distances)
        sens = -1.0 if self.miroir_lidar else 1.0
        angles = [sens * (angle_min + i * increment) + self.decalage_lidar for i in range(n)]
        with self.verrou:
            self.scan = (angles, [float(d) for d in distances], float(rmin), float(rmax))
            self.scan_nouveau = True
            self.instant_scan = time.monotonic()

    def recevoir_odom(self, x, y, theta, v, w):
        with self.verrou:
            self.pose_odom = (float(x), float(y), float(theta))
            self.vitesse = (float(v), float(w))
            self.instant_odom = time.monotonic()

    def recevoir_batterie(self, tension):
        with self.verrou:
            self.tension = float(tension)
            self.instant_batterie = time.monotonic()
            if 0 < self.tension < TENSION_ALERTE and not self.alerte_batterie_donnee:
                self.alerte_batterie_donnee = True
                self._dire("batterie")
            elif self.tension > TENSION_ALERTE + 0.3:
                self.alerte_batterie_donnee = False

    # ===================================================================
    # Ordres venant du téléphone
    # ===================================================================
    def nouvelle_mission(self, x, y, algo=None, phrase="depart", interne=False):
        with self.verrou:
            if not interne:
                self.exploration = None      # une destination choisie à la main arrête l'exploration
            if algo in ALGOS:
                self.algo = algo
            if self.arret_urgence:
                return False, "arrêt d'urgence actif"
            if self.pose_odom is None:
                return False, "pas d'odométrie (robot démarré ?)"
            if not self.localise:
                self._dire("pas_localise")
                return False, "je cherche encore ma position sur la carte"
            if self.tension is not None and 0 < self.tension < TENSION_MINI_MISSION:
                self._dire("batterie_vide")
                return False, "batterie trop faible"
            self.but = (float(x), float(y))
            self.echecs_planif = 0
            self.mission = {"debut": time.monotonic(), "tension": self.tension,
                            "distance": 0.0, "obstacles": 0, "replanifications": 0,
                            "algo": self.algo, "phrase": phrase, "reculs": 0,
                            "meilleure_d": math.inf, "sans_progres": 0}
            self.bilan = None
            self._lancer_planification()
            return True, ""

    def retour_base(self, algo=None):
        return self.nouvelle_mission(*self.depart_base, algo=algo, phrase="base")

    def annuler(self):
        with self.verrou:
            self.exploration = None
            if self.etat in EN_MISSION:
                self._dire("annule")
            self._terminer("attente")

    def commande_manuelle(self, v, w):
        with self.verrou:
            if self.etat != "manuel" and (abs(v) > 1e-3 or abs(w) > 1e-3):
                self.exploration = None
                if self.etat in EN_MISSION:
                    self._terminer("manuel")
                self.etat = "manuel"
                self._dire("manuel")
            self.demande = (float(v), float(w))
            self.instant_demande = time.monotonic()

    def regler_arret(self, actif):
        with self.verrou:
            if actif and not self.arret_urgence:
                self._dire("urgence", son="urgence")
            elif not actif and self.arret_urgence:
                self._dire("reprendre")
            self.arret_urgence = bool(actif)
            if actif:
                self.exploration = None
            self.demande = (0.0, 0.0)
            self.instant_demande = None
            if actif and self.etat in EN_MISSION:
                self._terminer("attente")

    def _appliquer_vitesse(self):
        v, w = NIVEAUX_VITESSE[self.niveau_vitesse]
        self.v_max = min(v, self.plafond_v)
        self.w_max = min(w, self.plafond_w)
        # En autonome : rotations un peu plus vives que la vitesse max manuelle
        self.suivi.v_max = self.v_max
        self.suivi.w_max = min(self.plafond_w * 1.2, self.w_max * 1.3)

    def regler_vitesse(self, niveau):
        """Vitesse choisie sur la manette (1 lent, 2 normal, 3 rapide), même en mission."""
        with self.verrou:
            if niveau in NIVEAUX_VITESSE:
                self.niveau_vitesse = niveau
                self._appliquer_vitesse()
            return self.niveau_vitesse, self.v_max

    def choisir_algo(self, algo):
        with self.verrou:
            if algo in ALGOS:
                self.algo = algo

    def effacer_carte(self):
        with self.verrou:
            self._terminer("attente")
            self.carte.reinitialiser()
            self.trace.clear()
            self.lieux.clear()
            self.exploration = None
            self.localise = True
            if self.slam is not None:
                self.demandes_slam += [("arreter",), ("lancer", None, None)]
                self.slam.update(T=None, S=None, actif=True, reprise=None)
            self.generation_loc += 1
            self.derniere_loc = None
            if self.pose_odom:
                self.depart_base = self.pose_odom[:2]
            self._dire("carte")

    # ---------------- carte enregistrée et lieux ----------------
    def enregistrer_carte(self, chemin=None, annoncer=True, nom=None, complet=None):
        """Écrit la carte, la base et les lieux dans un fichier.

        `nom` : enregistre sous ce nom (cartes/<nom>.npz) et en fait la carte courante.
        `complet` : écrit aussi le plan standard ROS (.pgm/.yaml) et, avec slam_toolbox, son
        graphe de poses (pour reprendre la cartographie un autre jour). Par défaut : seulement
        quand on l'annonce (bouton ENREGISTRER), pas à chaque lieu mémorisé.
        """
        with self.verrou:
            if complet is None:
                complet = annoncer
            if nom is not None:
                nom = nom_de_carte(nom)
                if not nom:
                    return False, "nom de carte vide"
                dossier = os.path.dirname(os.path.abspath(self.fichier_carte or "cartes/x.npz"))
                chemin = os.path.join(dossier, nom + ".npz")
                self.fichier_carte = chemin
            chemin = chemin or self.fichier_carte
            if not chemin:
                return False, "aucun fichier de carte"
            if not self.localise:
                return False, "position pas encore retrouvée : enregistrer abîmerait la carte"
            if self.carte.scans_integres == 0:
                return False, "la carte est vide"
            date = datetime.now().strftime("%d/%m/%Y %H:%M")
            infos = {"base": list(self.depart_base), "date": date,
                     "lieux": {n: list(p) for n, p in self.lieux.items()},
                     "nom": os.path.splitext(os.path.basename(chemin))[0],
                     "surface_m2": round(float((self.carte.etats() == 1).sum()) * self.carte.res ** 2, 1)}
            base = os.path.splitext(chemin)[0]
            if self.slam is not None and self.slam.get("T") is not None:
                infos["slam"] = {"T": list(self.slam["T"])}
                if complet and self.slam.get("actif"):
                    self.demandes_slam.append(("serialiser", base))
            self.carte.sauver(chemin, infos)
            if complet:
                try:
                    self.carte.exporter_pgm(base, infos["nom"])
                except OSError:
                    pass
                memoriser_derniere_carte(chemin)
            self.carte_enregistree = date
            if annoncer:
                self._dire("carte_enregistree")
            return True, date

    def charger_carte(self, chemin):
        """Recharge une carte. Le robot peut être posé N'IMPORTE OÙ dessus : il se retrouve seul."""
        with self.verrou:
            infos = self.carte.charger(chemin)
            self.exploration = None
            self._terminer("attente")
            if self.slam is not None:
                self.slam["actif"] = False    # l'ancienne session n'est pas à garder
            self.fichier_carte = chemin
            self.carte_enregistree = infos.get("date")
            self.depart_base = tuple(infos.get("base", (0.0, 0.0)))
            self.lieux = {n: tuple(p) for n, p in infos.get("lieux", {}).items()}
            self._chercher_position()
            if self.slam is not None:
                # Une fois la position retrouvée, slam_toolbox reprendra son graphe enregistré
                base = os.path.splitext(chemin)[0]
                graphe = os.path.isfile(base + ".posegraph") and "slam" in infos
                self.slam.update(T=tuple(infos["slam"]["T"]) if graphe else None,
                                 reprise=base if graphe else None)
            memoriser_derniere_carte(chemin)
            return infos

    def charger_carte_nommee(self, nom):
        """Bouton CHARGER : remplace la carte courante par une carte enregistrée."""
        nom = nom_de_carte(nom)
        dossier = os.path.dirname(os.path.abspath(self.fichier_carte or "cartes/x.npz"))
        chemin = os.path.join(dossier, nom + ".npz")
        if not nom or not os.path.isfile(chemin):
            return False, "carte introuvable"
        try:
            self.charger_carte(chemin)
        except (OSError, ValueError, KeyError) as exc:
            return False, f"carte illisible ({exc})"
        return True, ""

    def liste_cartes(self):
        """Cartes enregistrées dans le dossier cartes/ (pour le panneau CARTES)."""
        dossier = os.path.dirname(os.path.abspath(self.fichier_carte or "cartes/x.npz"))
        cartes = []
        try:
            noms = sorted(os.listdir(dossier))
        except OSError:
            return []
        for f in noms:
            if not f.endswith(".npz") or f.endswith((".precedente.npz", ".tmp.npz")) or f.startswith("_"):
                continue
            chemin = os.path.join(dossier, f)
            try:
                infos = lire_infos_carte(chemin)
            except Exception:
                continue
            nom = f[:-4]
            cartes.append({"nom": nom, "date": infos.get("date"), "lieux": len(infos.get("lieux", {})),
                           "surface_m2": infos.get("surface_m2"),
                           "slam": os.path.isfile(os.path.join(dossier, nom + ".posegraph")),
                           "courante": os.path.abspath(chemin) == os.path.abspath(self.fichier_carte or ""),
                           "modifie": os.path.getmtime(chemin)})
        cartes.sort(key=lambda c: -c["modifie"])
        return cartes

    def relocaliser(self):
        """Bouton « Où suis-je ? » : le robot a été déplacé à la main, il se retrouve."""
        with self.verrou:
            if self.carte.scans_integres == 0:
                return False, "la carte est vide"
            self.exploration = None
            self._terminer("attente")
            self._chercher_position()
            return True, ""

    # ---------------- exploration autonome ----------------
    def explorer(self, algo=None):
        """Le robot découvre seul la pièce : il va de frontière en frontière."""
        with self.verrou:
            if self.arret_urgence:
                return False, "arrêt d'urgence actif"
            if not self.localise:
                return False, "je cherche encore ma position sur la carte"
            if self.pose() is None:
                return False, "pas d'odométrie (robot démarré ?)"
            if algo in ALGOS:
                self.algo = algo
            self._terminer("attente")
            self.exploration = {"interdits": [], "etapes": 0, "suite": None, "debut": time.monotonic()}
            self._dire("exploration")
            return self._etape_exploration(premiere=True)

    def _etape_exploration(self, premiere=False):
        ex = self.exploration
        pose = self.pose()
        cible = None
        if ex["etapes"] < EXPLORATION_ETAPES_MAX and pose is not None:
            cible = prochaine_frontiere(self.carte, pose, interdits=ex["interdits"])
        if cible is None:
            self.exploration = None
            self._dire("exploration_impossible" if premiere else "exploration_finie", son="arrive")
            if not premiere and self.fichier_carte:
                self.enregistrer_carte(annoncer=False)
            return (not premiere), ("" if not premiere else "rien de nouveau à explorer d'ici")
        ex["etapes"] += 1
        ex["interdits"].append(cible)
        ok, raison = self.nouvelle_mission(*cible, phrase="zone", interne=True)
        if ok:
            self.mission["limite_s"] = EXPLORATION_LIMITE_ZONE
        if not ok:
            self.exploration = None
        return ok, raison

    def _chercher_position(self):
        if self.slam is not None:
            # slam_toolbox continuerait à cartographier avec une fausse position :
            # on garde sa carte de côté (fichier temporaire) et on l'arrête le temps de se retrouver.
            if self.slam.get("actif") and self.fichier_carte:
                tmp = os.path.join(os.path.dirname(os.path.abspath(self.fichier_carte)), "_reprise_slam")
                self.demandes_slam.append(("serialiser", tmp))
                self.slam["reprise"] = tmp
            self.demandes_slam.append(("arreter",))
            self.slam["actif"] = False
        self.carte.correction = (0.0, 0.0, 0.0)
        self.localise = False
        self.generation_loc += 1
        self.vues_loc = []
        self.rotation_loc = None
        self.tours_loc = 0
        self.aide_loc_donnee = False
        self.essais_loc = 0
        self.instant_loc = 0.0
        self.derniere_loc = None
        self.trace.clear()
        self._dire("cherche_position")

    def memoriser_lieu(self, nom):
        with self.verrou:
            nom = " ".join(str(nom).split())[:LIEU_NOM_MAX]
            pose = self.pose()
            if not nom:
                return False, "nom vide"
            if pose is None:
                return False, "position inconnue"
            self.lieux[nom] = (round(pose[0], 3), round(pose[1], 3))
            self._dire("lieu")
            if self.fichier_carte:
                self.enregistrer_carte(annoncer=False)
            return True, nom

    def supprimer_lieu(self, nom):
        with self.verrou:
            if self.lieux.pop(nom, None) is None:
                return False, "lieu inconnu"
            if self.fichier_carte:
                self.enregistrer_carte(annoncer=False)
            return True, ""

    def aller_lieu(self, nom, algo=None):
        with self.verrou:
            if nom not in self.lieux:
                return False, "lieu inconnu"
            return self.nouvelle_mission(*self.lieux[nom], algo=algo)

    def comparer(self, x, y):
        """Lance A*, Dijkstra et RRT sur la carte actuelle (pour le rapport)."""
        with self.verrou:
            pose = self.pose()
            if pose is None:
                return None
            grille = self.carte.grille_planification(points=[pose[:2], (x, y)])
            prudente = self.carte.grille_planification(points=[pose[:2], (x, y)], inconnu_libre=False)
        lignes = []
        for algo in ALGOS:
            p = planifier(grille, pose[:2], (x, y), algo, cap=pose[2], grille_prudente=prudente)
            lignes.append(p.resume())
        with self.verrou:
            self.comparaison = {"but": [x, y], "resultats": lignes, "id": next(self._ids)}
        return self.comparaison

    # ===================================================================
    # Boucle de contrôle (10 Hz)
    # ===================================================================
    def cycle(self):
        """Renvoie la commande (v, w) à envoyer aux moteurs."""
        with self.verrou:
            maintenant = time.monotonic()
            self._cartographier(maintenant)
            v, w = self._decider(maintenant)
            self.commande = (v, w)
            return v, w

    def _cartographier(self, maintenant):
        if not self.localise:
            self._localiser(maintenant)       # carte figée tant qu'on ne sait pas où on est
            return
        if self.slam is not None:
            return                            # c'est slam_toolbox qui construit la carte
        if not (self.scan_nouveau and self.pose_odom) or \
                maintenant - self.derniere_integration < 0.2:          # 5 Hz max
            return
        angles, dist, rmin, rmax = self.scan
        self.carte.integrer(self.pose_odom, angles, dist, rmin, rmax)
        self.scan_nouveau = False
        self.derniere_integration = maintenant

    def _localiser(self, maintenant):
        """Relocalisation globale en tâche de fond.

        Une seule vue peut tromper (deux coins de la pièce qui se ressemblent). On exige
        donc deux vues PRISES SOUS DES ANGLES DIFFÉRENTS (au moins 30° d'écart) qui donnent
        la même position. Entre deux vues, le robot tourne seul de 45° sur place (rotation
        sans avancer : sans risque), sauf si quelqu'un le pilote au joystick.
        """
        if self.fil_loc is not None:
            if self.fil_loc.is_alive():
                return
            gen, odom, resultat = self.resultat_loc
            self.fil_loc = None
            if gen == self.generation_loc:
                self._resultat_localisation(odom, resultat)
            return
        if self.rotation_loc is not None:
            return                            # on attend la fin du quart de tour
        immobile = abs(self.vitesse[0]) < 0.01 and abs(self.vitesse[1]) < 0.05
        if not (self.scan and self.pose_odom and self.instant_scan) or not immobile \
                or maintenant - self.instant_scan > 0.5 \
                or maintenant - self.instant_loc < PERIODE_LOCALISATION:
            return
        self.instant_loc = maintenant
        angles, dist, rmin, rmax = self.scan
        odom, gen = self.pose_odom, self.generation_loc
        angles, dist = list(angles), list(dist)

        def travail():
            try:
                r = self.carte.localiser_global(angles, dist, rmin, rmax)
            except Exception as exc:          # ne jamais bloquer le robot
                r = None
                print("Relocalisation : erreur", exc)
            self.resultat_loc = (gen, odom, r)

        self.fil_loc = threading.Thread(target=travail, daemon=True)
        self.fil_loc.start()

    def _resultat_localisation(self, odom, resultat):
        self.essais_loc += 1
        accepte = False
        if resultat is not None:
            pose_carte, qualite, ambigu = resultat
            correction = composer(pose_carte, inverser(odom))
            self.derniere_loc = {"qualite": qualite, "ambigu": ambigu}
            if not ambigu and qualite >= QUALITE_MINI:
                for c, cap in self.vues_loc:
                    if math.dist(c[:2], correction[:2]) < ACCORD_POSITION[0] \
                            and abs(angle_normalise(c[2] - correction[2])) < ACCORD_POSITION[1] \
                            and abs(angle_normalise(cap - odom[2])) >= ECART_VUES:
                        accepte = True
                self.vues_loc.append((correction, odom[2]))
        else:
            self.derniere_loc = {"qualite": 0.0, "ambigu": False}
        if accepte:
            self.carte.correction = correction
            self.localise = True
            if self.slam is not None:
                self._relancer_slam_apres_localisation()
            self.vues_loc = []
            self.rotation_loc = None
            self.derniere_integration = 0.0
            self._dire("position_trouvee", son="arrive")
            return
        # Pas encore sûr : quart de tour sur place, puis nouvelle vue
        if self.etat != "manuel" and not self.arret_urgence and self.tours_loc < ROTATIONS_LOC_MAX:
            self.tours_loc += 1
            self.rotation_loc = angle_normalise(odom[2] + math.radians(45))
            if self.tours_loc == 1:
                self._dire("tourne_position")
        elif self.essais_loc >= ROTATIONS_LOC_MAX + 1 and not self.aide_loc_donnee:
            self.aide_loc_donnee = True
            self._dire("aide_position")
        if self.essais_loc == 20:
            self._dire("inconnu")

    # ---------------- slam_toolbox (option --slam) ----------------
    def activer_slam(self):
        """La carte vient de slam_toolbox (avec fermeture de boucle), plus de notre SLAM maison."""
        with self.verrou:
            self.slam = {"T": None, "S": None, "actif": False, "reprise": None,
                         "cartes_recues": 0}

    def demarrer_slam(self):
        """Au démarrage de la station, après le chargement éventuel d'une carte."""
        with self.verrou:
            if self.slam is not None and self.localise:
                self.slam["actif"] = True
                self.demandes_slam.append(("lancer", None, None))

    def _relancer_slam_apres_localisation(self):
        """Position retrouvée sur une carte rechargée : slam_toolbox reprend LA carte, au bon endroit."""
        s = self.slam
        s["actif"] = True
        pose = self.pose()
        if s.get("reprise") and s.get("T") is not None and pose is not None:
            # pose du robot dans le repère de slam_toolbox = T⁻¹ ∘ pose
            self.demandes_slam.append(("lancer", s["reprise"], composer(inverser(s["T"]), pose)))
        else:
            s["T"] = None                     # nouvelle session : T calculé au premier TF
            self.demandes_slam.append(("lancer", None, None))
        s["reprise"] = None
        s["S"] = None

    def recevoir_tf_slam(self, x, y, theta):
        """Transformation map -> odom publiée par slam_toolbox (sa correction de l'odométrie)."""
        with self.verrou:
            s = self.slam
            if s is None or not s.get("actif") or not self.localise:
                return
            S = (x, y, theta)
            if s["T"] is None:
                # Premier TF d'une session : on aligne le repère de slam_toolbox sur le nôtre
                s["T"] = composer(self.carte.correction, inverser(S))
            s["S"] = S
            self.carte.correction = composer(s["T"], S)

    def recevoir_carte_slam(self, res, ox, oy, oth, largeur, hauteur, donnees):
        """Carte /map de slam_toolbox : recopiée dans la nôtre (affichage, planification…)."""
        with self.verrou:
            s = self.slam
            if s is None or not s.get("actif") or not self.localise or s["T"] is None:
                return
            self.carte.fusionner_grille(res, (ox, oy, oth), largeur, hauteur, donnees, s["T"])
            s["cartes_recues"] += 1

    def prendre_demandes_slam(self):
        with self.verrou:
            d, self.demandes_slam = self.demandes_slam, []
            return d

    def pose(self):
        if self.pose_odom is None or not self.localise:
            return None
        return self.carte.pose_carte(self.pose_odom)

    def _decider(self, maintenant):
        pose = self.pose()
        if self.pose_odom is None:
            return 0.0, 0.0
        if pose is not None and (not self.trace or math.dist(pose[:2], self.trace[-1]) > 0.02):
            if self.trace and self.mission:
                self.mission["distance"] += math.dist(pose[:2], self.trace[-1])
            self.trace.append((round(pose[0], 3), round(pose[1], 3)))
            del self.trace[:-3000]

        scan_frais = self.scan and self.instant_scan and maintenant - self.instant_scan < 1.0
        if scan_frais:
            angles, dist, rmin, _ = self.scan
            self.obstacle_devant = distance_devant(angles, dist, rmin)
            self.couloir_avant = distance_couloir(angles, dist, rmin, 1)
            self.couloir_arriere = distance_couloir(angles, dist, rmin, -1)
        else:
            self.obstacle_devant = self.couloir_avant = self.couloir_arriere = None

        if self.arret_urgence:
            return 0.0, 0.0

        # Exploration : la mission vers une frontière est finie, on passe à la suivante
        ex = self.exploration
        if ex and pose is not None and self.etat in ("arrive", "echec", "attente"):
            if ex["suite"] is None:
                ex["suite"] = maintenant + 0.8          # laisse le LiDAR mettre la carte à jour
            elif maintenant >= ex["suite"]:
                ex["suite"] = None
                self._etape_exploration()

        # ---------------- quart de tour de repérage (relocalisation) ----------------
        if self.rotation_loc is not None:
            if self.etat == "manuel" or self.localise:
                self.rotation_loc = None
            else:
                ecart = angle_normalise(self.rotation_loc - self.pose_odom[2])
                if abs(ecart) < math.radians(4):
                    self.rotation_loc = None
                    self.instant_loc = maintenant          # laisse arriver un scan immobile
                    return 0.0, 0.0
                return 0.0, math.copysign(max(0.25, min(0.6, 1.5 * abs(ecart))), ecart)

        # ---------------- manuel ----------------
        if self.etat == "manuel":
            age = None if self.instant_demande is None else maintenant - self.instant_demande
            if age is None or age > DELAI_HOMME_MORT:
                return 0.0, 0.0
            v = max(-self.v_max, min(self.v_max, self.demande[0]))
            w = max(-self.w_max, min(self.w_max, self.demande[1]))
            if not scan_frais:
                return max(-V_SANS_LIDAR, min(V_SANS_LIDAR, v)), w
            # Freinage progressif puis arrêt devant un obstacle, en avant comme en arrière
            d = self.couloir_avant if v > 0 else self.couloir_arriere if v < 0 else None
            if d is None or d > ALERTE_MANUEL + 0.10:
                self.alerte_proche_donnee = False
            if d is not None:
                limite = 0.0 if d < DISTANCE_MANUEL else max(0.03, (d - DISTANCE_MANUEL) * FREIN_MANUEL)
                if abs(v) > limite:
                    v = math.copysign(limite, v)
                angles, dist, rmin, _ = self.scan
                proche = obstacle_le_plus_proche(angles, dist, rmin, 1 if v > 0 else -1)
                ou = decrire_obstacle(*proche) if proche else ""
                if limite == 0.0 and abs(self.demande[0]) > 0.02 \
                        and maintenant - self.instant_alerte_manuel > 4.0:
                    self.instant_alerte_manuel = maintenant
                    self.alerte_proche_donnee = True
                    self._dire("obstacle", texte=f"Stop ! Obstacle {ou}.")
                elif d < ALERTE_MANUEL and not self.alerte_proche_donnee and abs(v) > 0.02:
                    # Prévenir AVANT d'arriver dessus, avec la distance et le côté
                    self.alerte_proche_donnee = True
                    self.instant_alerte_manuel = maintenant
                    self._dire("obstacle_loin", texte=f"Attention, obstacle {ou}. Je ralentis.")
            return v, w

        if pose is None:                      # position pas encore retrouvée
            return 0.0, 0.0

        # ---------------- recul (dégagement) ----------------
        if self.etat == "recul":
            if not scan_frais:
                return 0.0, 0.0
            r = self.recul
            fait = math.dist(pose[:2], r["depart"])
            bloque_derriere = self.couloir_arriere is not None and self.couloir_arriere < 0.15
            if fait >= RECUL_DISTANCE or bloque_derriere or maintenant - r["debut"] > 6.0:
                self.recul = None
                self._lancer_planification()
                return 0.0, 0.0
            return -RECUL_VITESSE, 0.0

        # ---------------- planification ----------------
        if self.etat == "planification":
            if self.fil_planif is not None and not self.fil_planif.is_alive():
                self._fin_planification()
            return 0.0, 0.0

        # ---------------- en route / obstacle ----------------
        if self.mission and self.mission.get("limite_s") and self.etat in EN_MISSION \
                and maintenant - self.mission["debut"] > self.mission["limite_s"]:
            self._terminer("echec")               # exploration : zone trop dure, on passe à la suivante
            return 0.0, 0.0
        if self.etat in ("en_route", "obstacle"):
            if not scan_frais:
                return 0.0, 0.0                   # sans LiDAR, on ne roule pas
            angles, dist, rmin, _ = self.scan
            pts = points_lidar_monde(pose, angles, dist, rmin)
            detail = obstacle_sur_chemin(pts, self.plan_dense, self.suivi._indice,
                                         longueur_avant=VISEE_OBSTACLE,
                                         marge=self.marge_chemin, avec_point=True)
            bloque = detail[0] if detail else None
            ou = ""
            if detail:
                px, py = detail[1]
                ou = decrire_obstacle(math.dist(pose[:2], (px, py)),
                                      math.atan2(py - pose[1], px - pose[0]) - pose[2])
            elif self.obstacle_devant is not None:
                ou = decrire_obstacle(self.obstacle_devant, 0.0)
            colle = self.obstacle_devant is not None and self.obstacle_devant < DISTANCE_URGENCE

            if self.etat == "obstacle":
                # Le chemin repris frôlerait-il encore le coin qui nous a arrêtés ?
                cmd_essai = self.suivi.commande(*pose)
                frole = cmd_essai.v > 0 and contact_prevu(angles, dist, cmd_essai.v, cmd_essai.w, rmin)
                if bloque is None and not colle and not frole:
                    self.instant_libre = self.instant_libre or maintenant
                    if maintenant - self.instant_libre >= LIBRE_DEPUIS:
                        self.etat = "en_route"
                        self._dire("reprise")
                else:
                    self.instant_libre = None
                    if maintenant - self.instant_obstacle >= ATTENTE_OBSTACLE:
                        self.mission["replanifications"] += 1
                        proche = (self.obstacle_devant is not None and self.obstacle_devant < OBSTACLE_PROCHE) \
                            or bool(frole)
                        # Trop près pour tourner proprement : on recule d'abord, puis on recalcule
                        if not (proche and self._commencer_recul(maintenant)):
                            self._dire("contourne")
                            self._lancer_planification()
                return 0.0, 0.0

            # en_route
            cmd = self.suivi.commande(*pose)
            if cmd.arrive:
                self._arrivee()
                return 0.0, 0.0
            # Bulle de sécurité : le virage en cours va-t-il frotter un coin, même sur le côté ?
            contact = contact_prevu(angles, dist, cmd.v, cmd.w, rmin) if cmd.v > 0 else None
            if contact is not None:
                colle = True
                ou = decrire_obstacle(*contact)
            if cmd.v > 0 and not colle and bloque is not None and bloque > 0.35 \
                    and maintenant - self.instant_verif > 1.5:
                # Obstacle vu de loin sur le chemin : on recalcule tout de suite
                # un chemin qui l'évite, sans s'arrêter longtemps.
                if self._abandon_si_bloque(pose):
                    return 0.0, 0.0
                self.mission["replanifications"] += 1
                self._dire("itineraire", texte=f"Obstacle {ou}. Je change d'itinéraire.")
                self._lancer_planification()
                return 0.0, 0.0
            if cmd.v > 0 and (bloque is not None or colle):
                if self._abandon_si_bloque(pose):
                    return 0.0, 0.0
                self.etat = "obstacle"
                self.instant_obstacle = maintenant
                self.instant_libre = None
                self.mission["obstacles"] += 1
                self._dire("obstacle", son="obstacle",
                           texte=f"Attention, obstacle {ou} ! Je m'arrête." if ou else None)
                return 0.0, 0.0
            # Un mur découvert plus loin coupe le chemin ? (vérifié 1 fois/s)
            if maintenant - self.instant_verif > 1.0:
                self.instant_verif = maintenant
                if chemin_traverse_obstacle(self.carte.etats(), self.carte,
                                            self.plan_dense, self.suivi._indice):
                    self.mission["replanifications"] += 1
                    self._dire("itineraire")
                    self._lancer_planification()
                    return 0.0, 0.0
            return cmd.v, cmd.w

        return 0.0, 0.0

    # ===================================================================
    def _abandon_si_bloque(self, pose):
        """Anti-boucle : obstacle après obstacle sans jamais se rapprocher du but -> on s'arrête."""
        m = self.mission
        d = math.dist(pose[:2], self.but)
        if d < m["meilleure_d"] - PROGRES_MINI:
            m["meilleure_d"], m["sans_progres"] = d, 0
            return False
        m["sans_progres"] += 1
        if m["sans_progres"] < BLOCAGES_MAX:
            return False
        m["manque_m"] = d
        self.raison_planif = "passage impossible"
        self._dire("bloque_proche", son="echec")
        self._terminer("echec")
        return True

    def _commencer_recul(self, maintenant):
        """Marche arrière de 20 cm si la place derrière est libre (vue au LiDAR)."""
        if not self.mission or self.mission["reculs"] >= RECULS_MAX:
            return False
        if self.couloir_arriere is not None and self.couloir_arriere < RECUL_PLACE_MINI:
            return False
        pose = self.pose()
        if pose is None:
            return False
        self.mission["reculs"] += 1
        self.recul = {"depart": pose[:2], "debut": maintenant}
        self.etat = "recul"
        self.plan_dense = []
        self._dire("recul")
        return True

    def _lancer_planification(self):
        pose = self.pose()
        but, algo = self.but, self.algo
        grille = self.carte.grille_planification(points=[pose[:2], but])
        prudente = self.carte.grille_planification(points=[pose[:2], but], inconnu_libre=False)
        self.etat = "planification"
        self.resultat_planif = None

        def travail():
            self.resultat_planif = planifier(grille, pose[:2], but, algo, cap=pose[2],
                                             grille_prudente=prudente)

        self.fil_planif = threading.Thread(target=travail, daemon=True)
        self.fil_planif.start()

    def _fin_planification(self):
        plan, self.fil_planif = self.resultat_planif, None
        if plan is None or not plan.succes:
            self.echecs_planif += 1
            self.raison_planif = plan.raison if plan else "erreur"
            self.plan = plan
            if not (plan and plan.raison.startswith("destination")) \
                    and self._commencer_recul(time.monotonic()):
                return                            # on se dégage, puis on recalcule
            if self.echecs_planif >= 3 or (plan and plan.raison.startswith("destination")):
                self._dire("echec", son="echec")
                self._terminer("echec")
            else:
                self.etat = "obstacle"            # on réessaiera dans 2,5 s
                self.instant_obstacle = time.monotonic()
            return
        premiere = not self.mission.get("annonce")
        self.mission["annonce"] = True
        self.echecs_planif = 0
        self.plan = plan
        self.suivi.nouveau_chemin(plan.points)
        self.plan_dense = self.suivi.chemin
        self.etat = "en_route"
        self.instant_verif = time.monotonic()
        # Marge fine : on roule doucement et on tolère des murs plus proches du chemin
        etroit = plan.mode != "normal"
        self.marge_chemin = 0.11 if etroit else 0.12
        self.suivi.v_max = min(self.v_max, 0.10) if etroit else self.v_max
        if premiere:
            self._dire(self.mission.get("phrase", "depart"), son="depart")
        if plan.mode != self.mission.get("mode_annonce", "normal"):
            self.mission["mode_annonce"] = plan.mode
            if plan.mode in ("etroit", "au_plus_pres"):
                self._dire(plan.mode)

    def _arrivee(self):
        proche = self.plan is not None and self.plan.mode == "au_plus_pres"
        self._dire("arrive_proche" if proche else "arrive", son="arrive")
        if proche and self.mission:
            self.mission["manque_m"] = self.plan.manque_m
        self._terminer("arrive")

    def _terminer(self, etat):
        if self.mission:
            m = self.mission
            self.bilan = {
                "resultat": etat,
                "algo": m["algo"],
                "duree_s": round(time.monotonic() - m["debut"], 1),
                "distance_m": round(m["distance"], 2),
                "chemin_prevu_m": round(self.plan.longueur_m, 2) if self.plan and self.plan.succes else None,
                "obstacles": m["obstacles"],
                "replanifications": m["replanifications"],
                "tension_debut": m["tension"],
                "tension_fin": self.tension,
                "au_plus_pres_manque_m": round(m["manque_m"], 2) if m.get("manque_m") else None,
                "reculs": m.get("reculs", 0),
                "exploration": self.exploration is not None,
            }
        self.mission = None
        self.recul = None
        self.etat = etat
        self.plan_dense = []
        self.suivi.nouveau_chemin([])

    def _dire(self, type_evenement, son=None, texte=None):
        texte = texte or random.choice(PHRASES[type_evenement])
        self.evenements.append({"id": next(self._ids), "type": type_evenement,
                                "texte": texte})
        if son:
            try:
                self.son(son)
            except Exception:
                pass

    # ===================================================================
    # Pour le téléphone
    # ===================================================================
    def instantane(self):
        maintenant = time.monotonic()

        def age(t):
            return None if t is None else round(maintenant - t, 2)

        with self.verrou:
            pose = self.pose()
            scan = None
            if self.scan:
                angles, dist, rmin, rmax = self.scan
                pas = max(1, len(dist) // 360)
                scan = [[round(angles[i], 3), round(dist[i], 3)]
                        for i in range(0, len(dist), pas)
                        if math.isfinite(dist[i]) and rmin <= dist[i] <= rmax and dist[i] > 0]
            return {
                "station": self.mode_station,
                "etat": self.etat,
                "arret_urgence": self.arret_urgence,
                "algo": self.algo,
                "algos": ALGOS,
                "but": self.but,
                "plan": self.plan.resume() if self.plan else None,
                "suivi_indice": self.suivi._indice if self.plan_dense else 0,
                "bilan": self.bilan,
                "comparaison": self.comparaison,
                "pose": None if pose is None else [round(p, 3) for p in pose],
                "vitesse": [round(c, 3) for c in self.vitesse],
                "commande": [round(c, 3) for c in self.commande],
                "obstacle_devant": None if self.obstacle_devant is None else round(self.obstacle_devant, 3),
                "scan": scan,
                "age_scan": age(self.instant_scan),
                "age_odom": age(self.instant_odom),
                "trace": self.trace[-800:],
                "tension": None if self.tension is None else round(self.tension, 2),
                "batterie_pct": pourcentage_batterie(self.tension),
                "batterie_alerte": self.tension is not None and 0 < self.tension < TENSION_ALERTE,
                "age_batterie": age(self.instant_batterie),
                "evenements": list(self.evenements),
                "carte_version": self.carte.version,
                "recalage_qualite": self.carte.derniere_qualite,
                "v_max": self.v_max, "w_max": self.w_max,
                "vitesse_niveau": self.niveau_vitesse, "niveaux_vitesse": NIVEAUX_VITESSE,
                "base": list(self.depart_base),
                "localise": self.localise,
                "localisation": None if self.localise else {
                    "essais": self.essais_loc, **(self.derniere_loc or {})},
                "lieux": {n: list(p) for n, p in self.lieux.items()},
                "carte_enregistree": self.carte_enregistree,
                "carte_nom": os.path.splitext(os.path.basename(self.fichier_carte))[0] if self.fichier_carte else None,
                "slam": None if self.slam is None else {
                    "actif": bool(self.slam.get("actif")), "cartes": self.slam["cartes_recues"]},
                "exploration": None if self.exploration is None else {
                    "etapes": self.exploration["etapes"],
                    "duree_s": round(maintenant - self.exploration["debut"], 1)},
            }

    def carte_pour_ui(self):
        with self.verrou:
            if self._cache_carte and self._cache_carte["version"] == self.carte.version:
                return self._cache_carte
            d = self.carte.pour_affichage()
            d["cases"] = base64.b64encode(d["cases"].tobytes()).decode()
            self._cache_carte = d
            return d


def distance_parcourue(trace):
    return longueur(trace)


__all__ = ["Cerveau", "pourcentage_batterie", "distance_devant", "densifier"]
