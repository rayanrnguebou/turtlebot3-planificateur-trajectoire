"""slam_toolbox piloté par la station (option --slam).

La station lance elle-même slam_toolbox (un seul terminal à ouvrir) et le relance
quand il faut :
  - au démarrage sans carte : nouvelle carte ;
  - carte rechargée : la station retrouve d'abord le robot sur la carte (LiDAR),
    PUIS relance slam_toolbox sur le graphe enregistré, à cette position
    (paramètres map_file_name + map_start_pose) : la cartographie continue sur
    l'ancienne carte, avec fermeture de boucle ;
  - CARTE RAZ : slam_toolbox repart de zéro.
L'enregistrement appelle le service /slam_toolbox/serialize_map (fichiers
<nom>.posegraph et <nom>.data, à côté du <nom>.npz de la station).
"""

from __future__ import annotations

import os
import signal
import subprocess
import tempfile

ICI = os.path.dirname(os.path.abspath(__file__))
REGLAGES = os.path.join(ICI, "config", "slam_station.yaml")


def ecrire_reglages(fichier=None, pose=None):
    """Fichier de paramètres temporaire = nos réglages + la carte à reprendre."""
    with open(REGLAGES, encoding="utf-8") as f:
        texte = f.read().rstrip() + "\n"
    if fichier:
        texte += f"    map_file_name: {os.path.abspath(fichier)}\n"
        x, y, th = pose or (0.0, 0.0, 0.0)
        texte += f"    map_start_pose: [{x:.4f}, {y:.4f}, {th:.4f}]\n"
    fd, chemin = tempfile.mkstemp(prefix="slam_station_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(texte)
    return chemin


class LanceurSlam:
    def __init__(self, journal=print):
        self.processus = None
        self.journal = journal
        self.reglages = None

    def lancer(self, fichier=None, pose=None):
        self.arreter()
        self.reglages = ecrire_reglages(fichier, pose)
        cmd = ["ros2", "launch", "slam_toolbox", "online_async_launch.py",
               f"slam_params_file:={self.reglages}", "use_sim_time:=false"]
        sortie = open(os.path.join(tempfile.gettempdir(), "slam_toolbox_station.log"), "ab")
        self.processus = subprocess.Popen(cmd, stdout=sortie, stderr=subprocess.STDOUT,
                                          start_new_session=True)
        self.journal("slam_toolbox lancé" + (f" (reprise de {os.path.basename(fichier)})" if fichier else
                                             " (nouvelle carte)"))

    def arreter(self):
        p, self.processus = self.processus, None
        if p is None or p.poll() is not None:
            return
        try:
            os.killpg(p.pid, signal.SIGINT)        # comme Ctrl+C : arrêt propre de tout le launch
            p.wait(timeout=8)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if self.reglages:
            try:
                os.remove(self.reglages)
            except OSError:
                pass

    def vivant(self):
        return self.processus is not None and self.processus.poll() is None
