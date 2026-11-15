"""Test des roues du TurtleBot3 Burger en Python (ROS 2 Humble).

À lancer SUR LA RASPBERRY PI, dans un 2e terminal SSH, pendant que le 1er
terminal fait tourner :  ros2 launch turtlebot3_bringup robot.launch.py

    python3 test_roues.py

Séquence (vitesses faibles) :
    0-2 s  avance à 0,05 m/s  (environ 10 cm)
    2-3 s  arrêt
    3-5 s  tourne sur place à gauche
    5-6 s  arrêt
    6-8 s  recule à 0,05 m/s
    puis arrêt et fin du programme.
Ctrl+C arrête le robot immédiatement.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (durée en s, vitesse linéaire en m/s, vitesse angulaire en rad/s, description)
SEQUENCE = [
    (2.0, 0.05, 0.0, "avance"),
    (1.0, 0.0, 0.0, "arrêt"),
    (2.0, 0.0, 0.5, "tourne à gauche"),
    (1.0, 0.0, 0.0, "arrêt"),
    (2.0, -0.05, 0.0, "recule"),
]


class TestRoues(Node):
    def __init__(self):
        super().__init__("test_roues")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.debut = self.get_clock().now()
        self.etape_affichee = -1
        self.fini = False
        self.create_timer(0.1, self.boucle)        # 10 commandes par seconde
        self.get_logger().info("Test des roues : départ dans 0,1 s")

    def envoyer(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def boucle(self):
        t = (self.get_clock().now() - self.debut).nanoseconds / 1e9
        fin = 0.0
        for i, (duree, v, w, nom) in enumerate(SEQUENCE):
            fin += duree
            if t < fin:
                if i != self.etape_affichee:
                    self.get_logger().info(f"Étape {i + 1}/{len(SEQUENCE)} : {nom}")
                    self.etape_affichee = i
                self.envoyer(v, w)
                return
        self.envoyer(0.0, 0.0)
        self.get_logger().info("Séquence terminée, robot arrêté.")
        self.fini = True


def main():
    rclpy.init()
    noeud = TestRoues()
    try:
        while rclpy.ok() and not noeud.fini:
            rclpy.spin_once(noeud, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Toujours arrêter les moteurs, même après Ctrl+C.
        try:
            for _ in range(3):
                noeud.envoyer(0.0, 0.0)
        except Exception:
            pass
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
