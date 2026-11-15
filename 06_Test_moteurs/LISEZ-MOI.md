# Faire tourner les roues

## Test 1 : sans ROS, avec les boutons (dès que le firmware OpenCR est envoyé)

Après l'étape D du Guide 1 (`./update.sh ... ` qui finit par `jump_to_fw`) :
robot au sol, 1 m libre devant.

- **SW1** (petit bouton sur l'OpenCR, 2e étage) : le robot avance d'environ 30 cm.
- **SW2** : il tourne sur place.

Ce test ne demande ni ROS 2 ni compilation : seulement le firmware.

## Test 2 : en Python, avec ROS 2 (après l'étape C du Guide 1)

1. Copie le programme sur la Pi, depuis **PowerShell sur le PC** (remplace l'adresse) :

```powershell
scp "C:\Users\Rayan\Documents\ESIEE-IT 2026-2027\Projets Transverses\Projet_TurtleBot3\06_Test_moteurs\test_roues.py" ubuntu@ADRESSE_IP:~/
```

2. **Terminal SSH n° 1** : démarre le robot et laisse tourner.

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

3. **Terminal SSH n° 2** :

```bash
python3 ~/test_roues.py
```

Le robot avance 2 s, s'arrête, tourne 2 s, s'arrête, recule 2 s, puis s'arrête.
`Ctrl+C` l'arrête immédiatement.

## Éteindre proprement

```bash
sudo poweroff
```

Attends que la LED verte de la Pi ne clignote plus (environ 20 s), puis mets
l'interrupteur de l'OpenCR sur OFF. Tout ce qui est installé reste sur la carte.
