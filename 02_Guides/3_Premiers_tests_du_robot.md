# Guide 3 : premiers tests du vrai robot

Prérequis : Guide 1 et Guide 2 terminés, PC et robot sur le **même Wi-Fi**.

1. **Robot** (PowerShell, en SSH) :
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
   Le LiDAR se met à tourner. Laisse ce terminal ouvert.

2. **PC** (Ubuntu) :
   ```bash
   ros2 topic list
   ```
   Tu dois voir `/scan`, `/odom`, `/imu`, `/cmd_vel`. Liste vide = problème réseau :
   même Wi-Fi ? même `ROS_DOMAIN_ID=30` des deux côtés ? fichier `.wslconfig` en place ?

3. **Conduire au clavier** (PC, nouveau terminal Ubuntu) :
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
   Touches W, A, S, D pour bouger, X pour freiner.

4. **Voir le LiDAR** (PC, nouveau terminal Ubuntu) :
   ```bash
   ros2 launch turtlebot3_bringup rviz2.launch.py
   ```
   Les points rouges dessinent les murs autour du robot.

- [ ] `ros2 topic list` affiche `/scan` depuis le PC
- [ ] Le robot obéit au clavier
- [ ] RViz affiche les murs

Ensuite : Phase 2, `04_Phase2_Navigation_ROS2\navigation_autonome\README.md`.
