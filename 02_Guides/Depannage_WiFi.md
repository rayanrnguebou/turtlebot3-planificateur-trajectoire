# Dépannage : le robot ne se connecte pas au Wi-Fi

Le robot ne crée **pas** son propre Wi-Fi : c'est lui qui se connecte à ton partage de
connexion ou à ta box, avec le nom et le mot de passe saisis dans Raspberry Pi Imager.

## 0. Vérifier que la Pi démarre

Sur la Pi (3e étage), la LED rouge doit être fixe et la verte clignoter au démarrage.
Si rien ne s'allume : problème d'alimentation (adaptateur 12 V, interrupteur OpenCR,
câble entre OpenCR et Pi) ou de carte microSD mal insérée. Pas un problème de Wi-Fi.

## 1. Avec un iPhone

1. Réglages → Général → Informations → **Nom** : renomme l'iPhone en `robotwifi`.
2. Réglages → Partage de connexion :
   - « Autoriser d'autres utilisateurs » : activé
   - **« Maximiser la compatibilité » : activé** (indispensable)
   - Mot de passe : `MOT_DE_PASSE_WIFI`
   - reste sur cet écran tant que le robot se connecte
3. Si la carte a été écrite avec un autre nom ou mot de passe, réécris-la avec Imager
   (Guide 1, étape A) avec exactement `robotwifi` / `MOT_DE_PASSE_WIFI` / pays FR.
4. Allume le robot, attends 5 minutes, connecte le PC au même partage de connexion.
5. L'iPhone n'affiche pas les adresses : scanne-les depuis PowerShell :

```powershell
ipconfig | findstr IPv4
1..14 | ForEach-Object { if (Test-Connection "172.20.10.$_" -Count 1 -Quiet) { "172.20.10.$_ répond" } }
```

`172.20.10.1` = l'iPhone, l'adresse donnée par `ipconfig` = ton PC.
Une **troisième** adresse = le robot : `ssh ubuntu@172.20.10.X`.
Si seules deux adresses répondent, le robot n'est pas connecté : passe à la méthode 2.

## 2. Méthode sûre : câble Ethernet sur une box

1. Relie le port Ethernet de la Pi (3e étage) à un port libre de la box avec un câble RJ45.
2. Allume le robot, attends 5 minutes.
3. Trouve l'appareil `turtlebot` ou `ubuntu` dans la page d'administration de la box
   (Freebox, Livebox, Bbox, SFR…), liste des appareils connectés.
4. PC sur le Wi-Fi de la box, puis : `ssh ubuntu@ADRESSE`.
5. Configure le Wi-Fi à la main :

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Remplace tout le contenu par celui du fichier `netplan_wifi_exemple.yaml` de ce dossier
(en adaptant nom et mot de passe, et en gardant exactement les espaces). Puis :

```bash
sudo netplan apply
ip a show wlan0
```

Réussi si une ligne `inet 172.20.10.x` (iPhone) ou `inet 192.168.x.x` (box) apparaît.

## 3. Méthode écran + clavier

Il faut un écran, un câble **micro-HDMI** (la Pi 4 n'a pas de HDMI normal) et un clavier USB.
Branche-les sur la Pi, allume, connecte-toi (`ubuntu` + mot de passe), puis fais l'étape 5
de la méthode 2. `hostname -I` affiche ensuite l'adresse IP.

## Messages fréquents

| Message | Signification | Solution |
| --- | --- | --- |
| `ssh: connect to host ... timed out` | PC et robot pas sur le même réseau, ou mauvaise IP | Même Wi-Fi pour les deux, revérifie l'IP |
| `Permission denied` | Mauvais mot de passe ou utilisateur | `ubuntu@...` et le mot de passe d'Imager |
| `Could not get lock /var/lib/dpkg/lock` | Ubuntu fait ses mises à jour | Attends 10 min, relance |
