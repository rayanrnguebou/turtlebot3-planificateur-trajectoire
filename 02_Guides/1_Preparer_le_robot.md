# Guide 1 : préparer le robot (Raspberry Pi + OpenCR)

Robot : TurtleBot3 Burger, Raspberry Pi 4 (4 Go), LiDAR LDS-03 probable.
Logiciel : Ubuntu Server 22.04 + ROS 2 Humble. Coche chaque case avant de passer à la suite.

## Étages du robot (du bas vers le haut)

| Étage | Contenu |
| --- | --- |
| 1er (en bas) | Moteurs Dynamixel, roues, batterie Li-Po |
| 2e | Carte OpenCR (boutons SW1, SW2, interrupteur, prise d'alimentation 12 V) |
| 3e | **Raspberry Pi 4** (fente microSD sur la face du dessous, côté opposé aux ports USB) |
| 4e (en haut) | LiDAR |

Carte microSD : contacts dorés tournés **vers la Pi**, étiquette vers le bas. Toujours robot éteint.

## Étape A : écrire Ubuntu sur la microSD (PC Windows + lecteur USB microSD)

1. Installe **Raspberry Pi Imager** : https://www.raspberrypi.com/software/
2. Branche le lecteur USB avec la microSD. Si Windows propose de formater : **Annuler**.
3. Dans Imager :
   - Appareil : **Raspberry Pi 4**
   - Système : *Other general-purpose OS* → *Ubuntu* → **Ubuntu Server 22.04.5 LTS (64-bit)**
   - Stockage : la microSD (vérifie la taille, environ 16 ou 32 Go)
4. Personnalisation (« Modifier les réglages ») :

| Réglage | Valeur |
| --- | --- |
| Nom d'hôte | `turtlebot` |
| Utilisateur | `ubuntu` |
| Mot de passe | simple, sans accent, noté sur papier |
| Wi-Fi SSID | `robotwifi` (renomme l'iPhone ainsi, voir Dépannage Wi-Fi) |
| Wi-Fi mot de passe | `MOT_DE_PASSE_WIFI` (celui du partage de connexion) |
| Pays Wi-Fi | FR |
| Services | **SSH activé**, avec mot de passe |

5. Écris la carte, attends la fin de la vérification, remets la carte dans la Pi.

- [ ] Carte écrite et remise dans la Pi

## Étape B : premier démarrage et connexion SSH

1. Branche l'adaptateur **12 V** sur l'OpenCR (2e étage), interrupteur sur **ON**.
2. Attends **5 minutes**. Sur la Pi, la LED rouge doit être fixe, la verte doit clignoter.
3. Trouve l'adresse IP du robot et connecte-toi depuis PowerShell :
   `ssh ubuntu@ADRESSE` (le mot de passe ne s'affiche pas pendant la frappe, c'est normal).
   Avec un iPhone, ou si le robot reste introuvable : suis **Depannage_WiFi.md**.
4. Une fois connecté (`ubuntu@turtlebot:~$`), désactive les mises à jour automatiques :

```bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```
Remplace les deux `"1"` par `"0"`, puis Ctrl+O, Entrée, Ctrl+X. Ensuite :

```bash
sudo systemctl mask systemd-networkd-wait-online.service
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo reboot
```

- [ ] Je suis connecté en SSH au robot

## Étape C : installer ROS 2 Humble et TurtleBot3 sur la Pi (1 h 30 à 2 h, sur secteur)

Ouvre une session qui survit aux coupures : `tmux new -s install`
(si tu es déconnecté : reconnecte-toi puis `tmux attach -t install`).

**C.1 Dépôt ROS 2**
```bash
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

**C.2 ROS 2 Humble**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base ros-dev-tools
```

**C.3 Paquets TurtleBot3**
```bash
sudo apt install -y python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
sudo apt install -y ros-humble-hls-lfcd-lds-driver ros-humble-turtlebot3-msgs ros-humble-dynamixel-sdk ros-humble-xacro libudev-dev
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
git clone -b humble https://github.com/ROBOTIS-GIT/coin_d4_driver
cd ~/turtlebot3_ws/src/turtlebot3 && rm -r turtlebot3_cartographer turtlebot3_navigation2
```

**C.4 Compilation (plus d'une heure, ne débranche rien)**
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1
```
Réussi si la fin affiche `Summary: X packages finished` sans `failed`.

**C.5 Configuration permanente** (remplace `LDS-03` si l'étiquette du LiDAR dit LDS-01 ou LDS-02)
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-03' >> ~/.bashrc
source ~/.bashrc
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

- [ ] Compilation sans `failed`

## Étape D : firmware de l'OpenCR (10 min)

Le câble USB doit relier la Pi (3e étage) au port micro-USB de l'OpenCR (2e étage).
`ls /dev/ttyACM*` doit afficher `/dev/ttyACM0`.

```bash
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install -y libc6:armhf
cd ~
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```
Réussi si la sortie se termine par `jump_to_fw`. En cas d'échec : maintiens SW2, appuie sur Reset,
relâche Reset puis SW2, et relance `./update.sh`.

**Test :** robot au sol, 1 m libre devant. **SW1** : il avance d'environ 30 cm. **SW2** : il tourne sur place.

- [ ] Le robot bouge avec SW1 et SW2

Source : manuel officiel ROBOTIS, https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/
