# Station de contrôle mobile : robot autonome piloté depuis le téléphone

C'est le démonstrateur réel du sujet, avec sa **station de contrôle au sol**.
Tout tourne **sur la Raspberry Pi du robot** : le téléphone ouvre simplement une
page web. Il n'y a pas d'appli à installer, ni Ubuntu ou ROS sur le PC.

## Ce que fait le robot, tout seul

1. **Il cartographie la pièce en direct** à partir du LiDAR et de l'odométrie, avec
   un recalage par corrélation de scans qui corrige la dérive des roues (un SLAM léger).
   → `cartographie.py`
   **Option `--slam` (recommandée pour la démo)** : la carte est construite par
   **slam_toolbox**, le SLAM standard de ROS 2, avec **fermeture de boucle** (quand le
   robot repasse au même endroit, il recolle la carte : les murs restent nets, un seul
   trait). La station lance et relance slam_toolbox elle-même. → `slam_lanceur.py`,
   `config/slam_station.yaml`
2. **Tu touches un point sur l'écran, puis tu appuies sur A (GO)** : il calcule
   lui-même le chemin avec **A\*, Dijkstra ou RRT**. Ce sont exactement les algos de la
   phase 1 (dossier `algos/`, copie de `03_Phase1_Algorithmes/algos`). Les obstacles
   sont gonflés du rayon du robot, puis le chemin est lissé. S'il n'y a pas de chemin, il
   réessaie avec une marge plus fine (passage étroit, zone déjà vue seulement), puis va au
   point atteignable le plus proche (« Je vais au plus près »). → `navigation.py`
3. **Il suit le chemin** avec le contrôleur Pure Pursuit de la phase 2. → `controle.py`
4. **Obstacle imprévu** (une personne, un carton…) : il le voit au LiDAR, s'arrête et dit
   « Oh ! Attention, obstacle ! ». Si l'obstacle part, il repart (« Je repars »). S'il
   reste plus de 2,5 s, il recalcule un chemin qui le contourne (« Je contourne
   l'obstacle »). Un mur découvert plus loin déclenche aussi un recalcul. S'il est collé à
   l'obstacle, il **recule de 20 cm** (si le LiDAR voit la place libre derrière) avant de
   recalculer. Après 4 blocages sans se rapprocher du but, il s'arrête au plus près au lieu
   de tourner en rond. Une **bulle de sécurité** prolonge le virage en cours sur 0,6 s : si le
   bord du robot allait frôler un coin de meuble, même sur le côté, il s'arrête avant. Il **annonce la distance et le côté** de l'obstacle dès qu'il le voit
   (« Obstacle à 45 cm, devant à gauche »), jusqu'à 90 cm devant en mission et 50 cm en manuel.
   → `cerveau.py`
5. **Il se souvient de la pièce.** La carte, la base et des **lieux nommés** (« Bureau »,
   « Porte ») s'enregistrent (bouton **LIEUX**, et automatiquement à l'arrêt). Au démarrage
   suivant, tout revient, et **le robot peut être posé n'importe où** : il retrouve seul sa
   position en comparant son LiDAR à la carte, à toutes les positions et orientations
   possibles (relocalisation globale). Pour ne jamais se tromper de coin, il tourne seul
   d'un quart de tour et confirme avec une deuxième vue, puis annonce « Je sais où je suis ! ».
   → `cartographie.py` (`localiser_global`), `cerveau.py`
   **Plusieurs cartes** : LIEUX → **SOUS UN NOM…** (ex. « Salle 330 »), puis un autre jour
   **MES CARTES…** → CHARGER. La dernière carte utilisée revient seule au démarrage.
   Chaque carte enregistrée donne aussi le plan au format ROS (`.pgm` + `.yaml`, lisible
   par RViz et Nav2) et, avec `--slam`, le graphe de slam_toolbox (`.posegraph` + `.data`) :
   rechargée, slam_toolbox **continue la même carte** à l'endroit où le robot s'est retrouvé.
   **PLAN EN IMAGE** : un PNG propre (fond blanc, repère en mètres, échelle, légende, base
   et lieux) à mettre dans le rapport ou à imprimer.
   **EXPLORER SEUL** (bouton LIEUX) : il découvre une nouvelle zone sans aide, en allant de
   frontière en frontière (bord entre le connu et l'inconnu), puis enregistre la carte.
   → `exploration.py`
6. **Arrivé à moins de 8 cm du but**, il annonce « Arrivé ! » et affiche le **bilan
   de mission** : durée, chemin prévu, distance parcourue, obstacles, recalculs, et
   tension de la batterie au début et à la fin.

La **voix** sort du téléphone (synthèse vocale en français, avec des bips). Le
**buzzer** du robot joue en plus une mélodie (OpenCR). Avec un haut-parleur branché sur
la Pi, l'option `--voix-robot` le fait aussi parler lui-même.

**Les données affichées sont réelles sur le robot** : la batterie vient de
`/battery_state` (tension mesurée par l'OpenCR), le LiDAR de `/scan`, la position de
`/odom`, et les temps de calcul sont mesurés pendant le calcul. Seul le mode
SIMULATION (badge jaune) utilise un robot virtuel.

## La manette

La carte est un **repère orthonormé gradué en mètres** : origine (0 ; 0) = base H,
axe X en rouge vers la droite, axe Y en vert vers le haut, grille tous les 25 cm à 2 m
selon le zoom et valeurs écrites le long des bords. La position du robot (X, Y, cap)
est affichée en haut à gauche.

| Commande | Rôle |
|---|---|
| Toucher l'écran | choisir la destination : elle se cale sur 5 cm près, une fiche donne X, Y, distance et direction ; ◀ ▲ ▼ ▶ la décalent de 5 cm |
| **XY** (à droite de l'écran) | taper les coordonnées exactes du point (en mètres), puis PLACER LA CIBLE ou PLACER ET GO |
| Glisser sur l'écran, L / R | déplacer la vue, zoomer ; ◎ recentre sur le robot |
| **A · GO** | lancer la mission autonome vers la cible |
| **B · ANNUL** | annuler la mission ; libère aussi l'arrêt d'urgence |
| **X · BASE** | retour automatique au point de départ (H) |
| **Y · VUE** | CARTE (fond sombre) → PLAN (fond blanc, style plan papier) → RADAR LiDAR |
| **VITESSE** | 1 lent (0,10 m/s) → 2 normal (0,15) → 3 rapide (0,22, le maximum du Burger) ; marche aussi pendant une mission |
| **ALGO** | change l'algorithme : A\* → Dijkstra → RRT |
| **COMPARER** | calcule les 3 chemins sur la carte réelle et affiche longueur, temps, nœuds et virages |
| **LIEUX** | enregistrer la carte, mémoriser l'endroit actuel sous un nom, y retourner (ALLER), « Où suis-je ? », EXPLORER SEUL |
| **CARTE RAZ** (maintenir 1 s) | efface la carte et les lieux ; le point actuel devient la base |
| **ARRÊT D'URGENCE** | stoppe tout ; appuie à nouveau (ou B) pour libérer |
| Joystick « MANUEL » | secours uniquement : il reprend la main et annule la mission |
| **? GUIDE** (en haut de l'écran) | ouvre le mode d'emploi déroulant : commandes, lecture de l'écran, sécurité, dépannage et **réglages de la voix** |

**La voix** : la meilleure voix française installée sur le téléphone est choisie
automatiquement. On peut en changer, régler le débit et le ton, ou la couper dans
**GUIDE → Voix du robot**. Sur iPhone, une voix beaucoup plus naturelle se télécharge dans
Réglages → Accessibilité → Contenu énoncé → Voix → Français → « Amélie » ou « Thomas »
(version **améliorée**).

Sécurités côté robot : vitesse plafonnée (0,22 m/s maximum, réglable avec --vmax) ; arrêt si le téléphone n'envoie plus
rien en manuel (0,5 s) ; en manuel, freinage progressif puis arrêt à 20 cm d'un obstacle, **en avant comme en
arrière** (le LiDAR surveille tout le couloir que le robot va balayer), et marche au pas si le LiDAR ne répond plus ; pas de mission sans LiDAR récent ; arrêt immédiat si un objet se
trouve à moins de 15 cm devant ; mission refusée sous 10,6 V ; alerte vocale sous 11,0 V.
Si l'algorithme plante, le robot reçoit « stop ».

---

## A. Aujourd'hui, sans le robot : simulation sur le PC

Double-clique sur **`1_essayer_sur_le_PC.bat`**. Le navigateur s'ouvre sur un robot
virtuel dans une salle de 5 m × 4 m, avec une table, une armoire, un carton et un muret.
Clique sur **DÉMARRER**, touche un point derrière la table, puis appuie sur **A**. Le
bouton **+▣** pose un obstacle surprise devant le robot.

Pour l'essayer sur le téléphone, le PC et le téléphone doivent être sur le même Wi-Fi.
Ouvre l'adresse `Sur le téléphone : http://…:8080` affichée dans la fenêtre noire.
Tourne le téléphone pour passer en mode console horizontale.

## B. Demain, sur le vrai robot

**1. Copier le dossier sur la Pi** : dans **Windows PowerShell**, robot allumé et
connecté au partage iPhone.

```powershell
scp -r "C:\Users\Rayan\Documents\ESIEE-IT 2026-2027\Projets Transverses\Projet_TurtleBot3\07_Station_mobile" ubuntu@172.20.10.2:~/
```

**2. Vérifier numpy** : dans le **terminal SSH**.

```bash
python3 -c "import numpy; print('numpy OK', numpy.__version__)"
```

Si c'est une erreur : `sudo apt install -y python3-numpy`.

**3. Terminal SSH n° 1** : démarrer le robot, puis laisser tourner.

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**4. Terminal SSH n° 2** : démarrer la station.

```bash
python3 ~/07_Station_mobile/station_mobile.py --slam
```

`--slam` : la carte est faite par slam_toolbox (plus propre). **Une seule fois**, installe-le
dans le terminal SSH (le robot doit avoir Internet via le partage iPhone) :
`sudo apt update && sudo apt install -y ros-humble-slam-toolbox`. Sans `--slam`, la station
utilise sa cartographie maison (rien à installer). Le journal de slam_toolbox est dans
`/tmp/slam_toolbox_station.log`.

Attendu : `Station de contrôle prête (mode ROBOT)` puis `Station mobile connectée à ROS 2`.
Si `SIMULATION` s'affiche à la place, ROS n'est pas chargé : tape `source ~/.bashrc` et
relance.

**5. Sur l'iPhone** (celui qui partage la connexion), dans Safari : `http://172.20.10.2:8080`,
puis touche **DÉMARRER**. Le son du téléphone doit être activé, et le bouton silencieux
désactivé.

**6. Premiers essais, dans cet ordre :**
1. **Robot posé au sol, immobile.** L'écran doit montrer les murs en cyan, les points
   rouges du LiDAR et la batterie réelle. Approche ta main : les points bougent.
2. **Fais un tour de la pièce au joystick MANUEL**, lentement. La carte se construit.
3. **Mission courte** : touche un point libre à 1 m, puis A. Garde le doigt près de
   l'ARRÊT D'URGENCE.
4. **Obstacle imprévu** : pendant une mission, pose ton pied ou un carton devant le robot.
5. **COMPARER**, puis teste les 3 algos sur la même destination. Filme l'écran et le
   robot pour la vidéo.

**7. Enregistrer la pièce** : LIEUX → **SOUS UN NOM…** → « Salle 330 », puis mémorise 2 ou
3 lieux (amène le robot, tape un nom, + MÉMORISER ICI). Dans `~/07_Station_mobile/cartes/` :
`Salle_330.npz` (carte de la station), `Salle_330.pgm` + `.yaml` (plan ROS) et, avec
`--slam`, `Salle_330.posegraph` + `.data`. **PLAN EN IMAGE** télécharge le plan en PNG.

**8. Le lendemain** : relance les deux terminaux (la dernière carte revient seule ; une autre
salle : LIEUX → MES CARTES… → CHARGER), pose le robot **où tu veux** dans la pièce.
L'écran affiche « Où suis-je ? » puis le robot dit « Je sais où je suis ! ». S'il hésite,
fais-le tourner un peu sur place au joystick. Ensuite LIEUX → ALLER.

**9. Arrêter** : Ctrl+C dans le terminal 2 (le robot reçoit « stop », la carte et le graphe slam_toolbox sont enregistrés ; marche aussi si la fenêtre SSH se ferme), puis Ctrl+C dans
le terminal 1, puis `sudo poweroff`.

## Options

```bash
python3 ~/07_Station_mobile/station_mobile.py --vmax 0.10      # plafond plus bas (même en vitesse 3)
python3 ~/07_Station_mobile/station_mobile.py --sans-recalage  # carte avec l'odométrie seule
python3 ~/07_Station_mobile/station_mobile.py --voix-robot     # voix sur la Pi (sudo apt install espeak-ng)
python3 ~/07_Station_mobile/station_mobile.py --lidar-decalage 180  # main devant vue derrière (90 : vue à droite, -90 : à gauche)
python3 ~/07_Station_mobile/station_mobile.py --lidar-miroir       # gauche et droite inversées sur le radar
python3 ~/07_Station_mobile/station_mobile.py --nouvelle-carte     # ignore la carte enregistrée (nouvelle salle)
python3 ~/07_Station_mobile/station_mobile.py --carte cartes/salle_B12.npz   # démarrer sur cette carte
python3 ~/07_Station_mobile/station_mobile.py --slam               # carte par slam_toolbox (fermeture de boucle)
```

À chaque démarrage, l'ancienne carte est copiée dans `cartes/carte_robot.precedente.npz`
(copie de secours).

## Tests automatiques (sans ROS, environ 1 min 30)

```bash
python -m pytest -q tests
```

Ces 37 tests vérifient : la carte retrouve les murs, le recalage corrige une erreur
d'odométrie de 6 cm et 3°, et les 3 algos contournent la table. A\* explore moins de
cases que Dijkstra. Les sécurités fonctionnent : plafonnement, homme mort, arrêt
d'urgence, batterie. Trois missions complètes en simulation (A\*, Dijkstra avec obstacle
surprise, RRT) arrivent au but **sans aucune collision**. La carte enregistrée est rechargée
et le robot, posé ailleurs (odométrie à zéro), se retrouve à moins de 10 cm et 5° près, puis
va à un lieu mémorisé sans collision. Le robot coincé devant une boîte recule puis arrive ;
l'exploration autonome cartographie toute la salle simulée sans collision. Le serveur web est aussi testé.
`tests/test_cartes_slam.py` vérifie les cartes nommées, le nettoyage d'affichage, l'export
`.pgm`/`.yaml` et le suivi de slam_toolbox (TF `map→odom`, carte `/map`, reprise du graphe).

**Essai complet avec slam_toolbox, sans le robot** (PC ou VM avec ROS 2 Humble et
slam_toolbox) : `python3 tests/robot_simule_ros.py` dans un terminal (faux robot qui publie
`/scan`, `/odom` et les TF comme le vrai), `python3 station_mobile.py --slam` dans un autre.
C'est ainsi que l'option `--slam` a été validée : exploration complète de la salle simulée,
enregistrement, redémarrage avec le robot posé ailleurs, relocalisation, reprise du graphe
et mission vers un lieu mémorisé.

## Limites honnêtes (à citer dans le rapport)

- Sans `--slam`, le recalage corrige les petites dérives (± 10 cm, ± 4° entre deux scans),
  sans fermeture de boucle. Avec `--slam`, c'est slam_toolbox qui s'en charge.
- L'affichage nettoie la carte (points isolés, trous d'une case) pour qu'elle soit
  lisible ; la planification, elle, utilise la carte brute (un pied de chaise reste un obstacle).
- La relocalisation suppose que la pièce n'a pas trop changé depuis l'enregistrement
  (meubles déplacés, nombreuses personnes). Dans une pièce très symétrique, elle peut
  hésiter : il suffit de faire tourner le robot pour qu'il voie un détail qui tranche.
- Les zones inconnues sont supposées libres (exploration « optimiste ») : le robot
  replanifie quand il y découvre un mur. C'est voulu.
- Le code ROS (`pont_ros.py`) n'a pas pu être exécuté hors du robot : c'est l'essai de
  demain qui le valide. Le reste du code (cerveau, carte, navigation) est le même qu'en
  simulation, et il est testé.
