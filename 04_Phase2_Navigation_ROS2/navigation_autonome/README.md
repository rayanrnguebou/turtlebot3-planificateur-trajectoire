# Phase 2 : navigation autonome du TurtleBot3 (ROS 2 Humble, Python)

Ce paquet ROS 2 branche les algorithmes de la phase 1 (A*, Dijkstra, RRT) sur le robot :

- **`planificateur`** lit la carte (`/map`) et la position du robot, puis calcule
  un chemin vers le but que tu cliques dans RViz. Il publie ce chemin sur `/plan`.
- **`suivi`** fait rouler le robot le long de `/plan` avec un contrôleur
  **Pure Pursuit** (`/cmd_vel`). Il s'arrête si le LiDAR voit un obstacle à moins
  de 20 cm devant, et redemande un chemin s'il reste bloqué 3 s.

Prérequis : WSL2 + ROS 2 Humble installés (étape 3 du guide), et la commande
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` qui fonctionne.

## 1. Installer le paquet (une seule fois)

Ouvre **Ubuntu** (WSL) et tape (les guillemets sont indispensables à cause des espaces dans le chemin) :

```bash
cp -r "/mnt/c/Users/Rayan/Documents/ESIEE-IT 2026-2027/Projets Transverses/Projet_TurtleBot3/04_Phase2_Navigation_ROS2/navigation_autonome" ~/turtlebot3_ws/src/
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select navigation_autonome
source install/setup.bash
```

Le build doit finir par `1 package finished`.

Facultatif : lance les tests (sans ROS, quelques secondes) :

```bash
sudo apt install -y python3-pytest
cd ~/turtlebot3_ws/src/navigation_autonome && python3 -m pytest test
```

## 2. Tester en simulation (Gazebo)

Ouvre **3 terminaux Ubuntu**. Dans chacun, les variables `ROS_DOMAIN_ID=30`
et `TURTLEBOT3_MODEL=burger` doivent déjà être dans `~/.bashrc` (étape 3 du guide).

**Terminal 1 : le monde simulé**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 : le SLAM (construit la carte et localise le robot), avec RViz**
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

**Terminal 3 : notre navigation**
```bash
ros2 launch navigation_autonome navigation.launch.py use_sim_time:=true
```

Dans **RViz** :

1. Clique **Add** (en bas à gauche) → onglet **By topic** → `/plan` → **Path** → OK.
   Le chemin calculé s'affichera en couleur.
2. Clique **2D Goal Pose** dans la barre du haut, puis clique dans une zone blanche
   (libre) de la carte en faisant glisser pour choisir l'orientation.
3. Le terminal 3 affiche par exemple :
   `A* : chemin de 1.84 m, 9 points, 412 nœuds explorés, 38 ms`,
   et le robot part tout seul. À l'arrivée : `Arrivé ! 1.91 m parcourus en 14.2 s.`

Les zones grises (pas encore vues par le LiDAR) sont interdites par défaut. Si le
but est dans le gris, déplace d'abord le robot au clavier pour compléter la carte :
`ros2 run turtlebot3_teleop teleop_keyboard`.

## 3. Comparer les algorithmes

```bash
ros2 launch navigation_autonome navigation.launch.py use_sim_time:=true algorithme:=dijkstra
ros2 launch navigation_autonome navigation.launch.py use_sim_time:=true algorithme:=rrt
```

Ou sans relancer : `ros2 param set /planificateur algorithme rrt`.
Note les lignes de résultat (longueur, nœuds, temps, durée du trajet) pour chaque
algorithme et le même but : ce sont les mesures du rapport sur le « vrai » problème.

Tous les réglages sont dans `config/parametres.yaml` (vitesse, marge de sécurité,
distance d'arrêt d'urgence…).

## 4. Sur le vrai robot

1. **Robot** (SSH depuis PowerShell) : `ros2 launch turtlebot3_bringup robot.launch.py`
2. **PC, terminal 2** : `ros2 launch turtlebot3_cartographer cartographer.launch.py`
3. **PC, terminal 3** : `ros2 launch navigation_autonome navigation.launch.py`
4. Clique un but proche (1 m) dans RViz pour le premier essai, la main prête à
   faire **Ctrl+C dans le terminal 3**, ce qui arrête le robot.

Commence avec `vitesse_max: 0.10` dans `config/parametres.yaml` pour les premiers essais.

## Fichiers

| Fichier | Rôle |
| --- | --- |
| `navigation_autonome/algos/` | Les algorithmes de la phase 1 (copie) |
| `navigation_autonome/carte_ros.py` | Carte ROS (OccupancyGrid) → notre grille |
| `navigation_autonome/controle.py` | Contrôleur Pure Pursuit et détection d'obstacle (Python pur) |
| `navigation_autonome/noeud_planificateur.py` | Nœud ROS 2 : `/map` + `/goal_pose` → `/plan` |
| `navigation_autonome/noeud_suivi.py` | Nœud ROS 2 : `/plan` + `/scan` → `/cmd_vel` |
| `launch/navigation.launch.py` | Lance les deux nœuds |
| `config/parametres.yaml` | Tous les réglages |
| `test/test_navigation.py` | Tests, dont un robot simulé qui suit un chemin A* sans toucher les murs |
