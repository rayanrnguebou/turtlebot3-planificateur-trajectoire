# Guide 2 : installer WSL2 et ROS 2 Humble sur le PC (Windows 11)

Environ 1 h. ROS 2 tourne dans un Ubuntu intégré à Windows (WSL2). Ton PC est sous
Windows 11 25H2 : le mode réseau « mirrored » fonctionne.

## 1. Installer Ubuntu 22.04 dans Windows

PowerShell **en administrateur** (clic droit sur Démarrer → Terminal (administrateur)) :

```powershell
wsl --install -d Ubuntu-22.04
```

Redémarre le PC. Ubuntu s'ouvre et demande un nom d'utilisateur et un mot de passe : choisis-les simples.

## 2. Partager le réseau de Windows avec Ubuntu

Crée le fichier `C:\Users\Rayan\.wslconfig` (avec le Bloc-notes, « Tous les fichiers », sans `.txt`) :

```
[wsl2]
networkingMode=mirrored
```

Puis, en PowerShell administrateur :

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
wsl --shutdown
```

## 3. Installer ROS 2 Humble Desktop (dans Ubuntu)

Ouvre **Ubuntu** depuis le menu Démarrer :

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools
```

## 4. Installer les outils TurtleBot3 (simulation, SLAM, navigation)

```bash
sudo apt install -y ros-humble-gazebo-* ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-dynamixel-sdk ros-humble-turtlebot3-msgs ros-humble-turtlebot3 python3-pytest
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

## 5. Test

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Gazebo doit s'ouvrir avec un Burger virtuel au milieu d'obstacles (la première ouverture
peut prendre 1 à 2 minutes).

- [ ] Gazebo s'ouvre avec le Burger

Conseil : installe l'extension **WSL** dans VS Code pour éditer le code Ubuntu depuis Windows.
