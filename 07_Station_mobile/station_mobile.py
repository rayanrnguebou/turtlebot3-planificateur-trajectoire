#!/usr/bin/env python3
"""Station de contrôle au sol du TurtleBot3 : console de pilotage sur téléphone.

À lancer SUR LA RASPBERRY PI, pendant que tourne dans un autre terminal :
    ros2 launch turtlebot3_bringup robot.launch.py

    python3 ~/07_Station_mobile/station_mobile.py

Puis, sur le téléphone : http://172.20.10.2:8080

Le robot est AUTONOME : on touche un point sur la carte de l'écran, on appuie
sur GO, et il calcule seul son chemin avec A*, Dijkstra ou RRT (nos algos de la
phase 1), le suit (Pure Pursuit), s'arrête et contourne les obstacles imprévus
détectés au LiDAR, et annonce « Arrivé ! ». La carte de la pièce est construite
en direct par le robot (cartographie + recalage). Le joystick ne sert qu'en
secours (mode manuel).

Sans ROS 2 (sur le PC Windows), le programme démarre en SIMULATION :
    python station_mobile.py --simulation        puis http://localhost:8080
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)

try:
    import numpy  # noqa: F401
except ImportError:
    print("numpy est nécessaire.  Sur la Pi : sudo apt install -y python3-numpy\n"
          "Sur Windows : pip install numpy")
    sys.exit(1)

from cerveau import V_MAX_DEFAUT, W_MAX_DEFAUT, Cerveau  # noqa: E402

DOSSIER_WEB = os.path.join(ICI, "web")
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8", ".png": "image/png",
         ".svg": "image/svg+xml", ".json": "application/json",
         ".webmanifest": "application/manifest+json"}


def fabriquer_gestionnaire(cerveau, simu=None):
    class Gestionnaire(BaseHTTPRequestHandler):
        server_version = "StationTB3/2.0"

        def log_message(self, *args):
            pass

        def _repondre(self, code, corps, type_mime="application/json"):
            if isinstance(corps, (dict, list)) or corps is None:
                corps = json.dumps(corps, default=float).encode()
            elif isinstance(corps, str):
                corps = corps.encode()
            self.send_response(code)
            self.send_header("Content-Type", type_mime)
            self.send_header("Content-Length", str(len(corps)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(corps)

        def do_GET(self):
            chemin = self.path.split("?")[0]
            if chemin == "/api/etat":
                e = cerveau.instantane()
                if simu:
                    e["simulation"] = simu.etat_reel()
                return self._repondre(200, e)
            if chemin == "/api/carte":
                return self._repondre(200, cerveau.carte_pour_ui())
            if chemin == "/api/cartes":
                return self._repondre(200, {"cartes": cerveau.liste_cartes()})
            if chemin == "/":
                chemin = "/index.html"
            fichier = os.path.normpath(os.path.join(DOSSIER_WEB, chemin.lstrip("/")))
            if not fichier.startswith(DOSSIER_WEB + os.sep) or not os.path.isfile(fichier):
                return self._repondre(404, {"erreur": "introuvable"})
            with open(fichier, "rb") as f:
                contenu = f.read()
            self._repondre(200, contenu, TYPES.get(os.path.splitext(fichier)[1],
                                                   "application/octet-stream"))

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0) or 0)
                d = json.loads(self.rfile.read(n) or b"{}") if n else {}
                if not isinstance(d, dict):
                    raise ValueError
            except (ValueError, json.JSONDecodeError):
                return self._repondre(400, {"erreur": "JSON invalide"})

            def nombre(cle, defaut=None):
                v = float(d.get(cle, defaut))
                if not math.isfinite(v):
                    raise ValueError(cle)
                return v

            try:
                p = self.path
                if p == "/api/joystick":
                    cerveau.commande_manuelle(nombre("v", 0), nombre("w", 0))
                elif p == "/api/arret":
                    cerveau.regler_arret(True)
                elif p == "/api/reprendre":
                    cerveau.regler_arret(False)
                elif p == "/api/mission":
                    ok, raison = cerveau.nouvelle_mission(nombre("x"), nombre("y"), d.get("algo"))
                    return self._repondre(200 if ok else 409, {"ok": ok, "raison": raison})
                elif p == "/api/base":
                    ok, raison = cerveau.retour_base(d.get("algo"))
                    return self._repondre(200 if ok else 409, {"ok": ok, "raison": raison})
                elif p == "/api/annuler":
                    cerveau.annuler()
                elif p == "/api/vitesse":
                    niveau, v = cerveau.regler_vitesse(int(nombre("niveau")))
                    return self._repondre(200, {"ok": True, "niveau": niveau, "v_max": v})
                elif p == "/api/algo":
                    cerveau.choisir_algo(d.get("algo"))
                elif p == "/api/comparer":
                    return self._repondre(200, cerveau.comparer(nombre("x"), nombre("y")))
                elif p == "/api/effacer_carte":
                    cerveau.effacer_carte()
                elif p == "/api/carte/enregistrer":
                    nom = d.get("nom")
                    if nom is not None and not isinstance(nom, str):
                        raise ValueError("nom")
                    ok, info = cerveau.enregistrer_carte(nom=nom or None)
                    return self._repondre(200 if ok else 409, {"ok": ok, "raison": "" if ok else info,
                                                               "date": info if ok else None})
                elif p == "/api/carte/charger":
                    if not isinstance(d.get("nom"), str):
                        raise ValueError("nom")
                    ok, raison = cerveau.charger_carte_nommee(d["nom"])
                    return self._repondre(200 if ok else 409, {"ok": ok, "raison": raison})
                elif p == "/api/explorer":
                    ok, raison = cerveau.explorer(d.get("algo"))
                    return self._repondre(200 if ok else 409, {"ok": ok, "raison": raison})
                elif p == "/api/relocaliser":
                    ok, raison = cerveau.relocaliser()
                    return self._repondre(200 if ok else 409, {"ok": ok, "raison": raison})
                elif p in ("/api/lieu", "/api/lieu/aller", "/api/lieu/supprimer"):
                    nom = d.get("nom")
                    if not isinstance(nom, str):
                        raise ValueError("nom")
                    action = {"/api/lieu": cerveau.memoriser_lieu,
                              "/api/lieu/aller": lambda n: cerveau.aller_lieu(n, d.get("algo")),
                              "/api/lieu/supprimer": cerveau.supprimer_lieu}[p]
                    ok, info = action(nom)
                    return self._repondre(200 if ok else 409, {"ok": ok, "raison": "" if ok else info})
                elif p == "/api/sim/obstacle" and simu:
                    return self._repondre(200, {"ok": True, "position": simu.poser_obstacle()})
                elif p == "/api/sim/retirer" and simu:
                    simu.retirer_obstacles()
                else:
                    return self._repondre(404, {"erreur": "introuvable"})
            except (TypeError, ValueError):
                return self._repondre(400, {"erreur": "valeurs invalides"})
            return self._repondre(200, {"ok": True})

    return Gestionnaire


def voix_du_robot(cerveau):
    """Fait parler le haut-parleur de la Pi (option --voix-robot, espeak-ng)."""
    if not shutil.which("espeak-ng"):
        print("espeak-ng absent : sudo apt install -y espeak-ng  (voix sur la Pi désactivée)")
        return
    dernier = max([e["id"] for e in cerveau.evenements] or [0])

    def boucle():
        nonlocal dernier
        while True:
            for e in list(cerveau.evenements):
                if e["id"] > dernier:
                    dernier = e["id"]
                    subprocess.Popen(["espeak-ng", "-v", "fr", "-s", "150", e["texte"]],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.2)

    threading.Thread(target=boucle, daemon=True).start()


def adresses_locales():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    ips.discard("127.0.0.1")
    return sorted(ips)


def main():
    p = argparse.ArgumentParser(description="Station de contrôle mobile TurtleBot3")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--simulation", action="store_true", help="robot virtuel (sans ROS 2)")
    p.add_argument("--vmax", type=float, default=V_MAX_DEFAUT, help="plafond de vitesse en m/s (défaut 0.22, max du Burger)")
    p.add_argument("--wmax", type=float, default=W_MAX_DEFAUT, help="plafond de rotation en rad/s (défaut 1.8)")
    p.add_argument("--sans-recalage", action="store_true",
                   help="cartographie avec l'odométrie seule (sans scan matching)")
    p.add_argument("--lidar-decalage", type=float, default=0.0, metavar="DEGRES",
                   help="corrige un LiDAR monté tourné (ex. 180 si la main devant apparaît derrière)")
    p.add_argument("--lidar-miroir", action="store_true",
                   help="corrige un LiDAR dont la gauche et la droite sont inversées")
    p.add_argument("--voix-robot", action="store_true",
                   help="le robot parle aussi par son haut-parleur (espeak-ng)")
    p.add_argument("--carte", metavar="FICHIER",
                   help="fichier de la carte enregistrée (défaut : cartes/carte_robot.npz)")
    p.add_argument("--nouvelle-carte", action="store_true",
                   help="ignore la carte enregistrée et repart d'une carte vide")
    p.add_argument("--slam", action="store_true",
                   help="cartographie par slam_toolbox (fermeture de boucle), lancé par la station")
    args = p.parse_args()
    vmax = max(0.02, min(0.22, args.vmax))
    wmax = max(0.2, min(2.84, args.wmax))

    simulation = args.simulation
    if not simulation:
        try:
            import rclpy  # noqa: F401
        except ImportError:
            print("ROS 2 introuvable : démarrage en mode SIMULATION.")
            simulation = True

    cerveau = Cerveau(vmax, wmax, "simulation" if simulation else "robot",
                      recalage=not args.sans_recalage)
    cerveau.decalage_lidar = math.radians(args.lidar_decalage)
    cerveau.miroir_lidar = args.lidar_miroir
    dossier_cartes = os.path.join(ICI, "cartes")
    fichier = args.carte or os.path.join(
        dossier_cartes, "carte_simulation.npz" if simulation else "carte_robot.npz")
    if not args.carte and not simulation:
        # Par défaut : la dernière carte utilisée (enregistrée ou chargée depuis le téléphone)
        try:
            with open(os.path.join(dossier_cartes, "derniere_carte.txt"), encoding="utf-8") as f:
                derniere = os.path.join(dossier_cartes, os.path.basename(f.read().strip()))
            if derniere.endswith(".npz") and os.path.isfile(derniere):
                fichier = derniere
        except OSError:
            pass
    if args.slam:
        if simulation:
            print("--slam demande ROS 2 et slam_toolbox : ignoré en simulation.")
        else:
            try:
                import slam_toolbox.srv  # noqa: F401
                cerveau.activer_slam()
            except ImportError:
                print("=" * 62)
                print("  slam_toolbox n'est pas installé sur ce robot : la station utilise")
                print("  sa cartographie maison (tout le reste fonctionne).")
                print("  Pour l'installer (une seule fois, robot connecté à Internet) :")
                print("    sudo apt update && sudo apt install -y ros-humble-slam-toolbox")
                print("=" * 62)
    cerveau.fichier_carte = fichier
    carte_chargee = False
    if os.path.isfile(fichier) and not args.nouvelle_carte:
        try:
            infos = cerveau.charger_carte(fichier)
            shutil.copyfile(fichier, fichier[:-4] + ".precedente.npz")   # copie de secours
            carte_chargee = True
            print(f"Carte enregistrée rechargée ({infos.get('date', '?')}, "
                  f"{len(infos.get('lieux', {}))} lieu(x)) : le robot cherche sa position.")
        except (OSError, ValueError, KeyError) as exc:
            print(f"Carte enregistrée illisible ({exc}) : on repart d'une carte vide.")
    if cerveau.slam is not None:
        cerveau.demarrer_slam()          # sans carte rechargée : slam_toolbox démarre tout de suite
    simu = None
    if simulation:
        import random
        from robot_simule import DEPARTS_LIBRES, RobotSimule
        # Avec une carte enregistrée, le robot virtuel est posé à un endroit au hasard :
        # il doit se retrouver tout seul, comme le vrai.
        depart = DEPARTS_LIBRES[0]
        if carte_chargee:
            e = cerveau.carte.etats()
            connus = [d for d in DEPARTS_LIBRES[1:]
                      if e[tuple(int(v) for v in cerveau.carte.vers_case(d[0], d[1]))] == 1]
            depart = random.choice(connus or DEPARTS_LIBRES[:1])
            print(f"Robot virtuel posé en ({depart[0]:.1f} ; {depart[1]:.1f}) : à lui de se retrouver.")
        simu = RobotSimule(cerveau, depart=depart)
        simu.start()
        arreter = simu.arreter.set
    else:
        import pont_ros
        arreter = pont_ros.demarrer(cerveau)
    if args.voix_robot:
        voix_du_robot(cerveau)

    serveur = ThreadingHTTPServer(("0.0.0.0", args.port), fabriquer_gestionnaire(cerveau, simu))
    serveur.daemon_threads = True
    print("=" * 62)
    print(f"  Station de contrôle prête  (mode {cerveau.mode_station.upper()})")
    print(f"  Sur cet appareil   : http://localhost:{args.port}")
    for ip in adresses_locales():
        print(f"  Sur le téléphone   : http://{ip}:{args.port}")
    print("  Ctrl+C pour arrêter (le robot s'arrête aussi).")
    print("=" * 62)
    def fin_demandee(*_):
        raise KeyboardInterrupt           # fenêtre SSH fermée, kill… : on enregistre quand même
    for sig in ("SIGTERM", "SIGHUP"):
        if hasattr(signal, sig):
            signal.signal(getattr(signal, sig), fin_demandee)
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cerveau.regler_arret(True)
        ok, info = cerveau.enregistrer_carte(annoncer=False, complet=True)
        print(f"Carte enregistrée : {fichier}" if ok else f"Carte non enregistrée ({info}).")
        serveur.server_close()
        arreter()
        print("Station arrêtée.")


if __name__ == "__main__":
    main()
