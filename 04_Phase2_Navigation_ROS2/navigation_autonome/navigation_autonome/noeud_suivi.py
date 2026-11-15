"""Nœud ROS 2 : fait suivre au robot le chemin publié sur /plan.

Abonnements :
    /plan   nav_msgs/Path          chemin à suivre (du planificateur)
    /scan   sensor_msgs/LaserScan  LiDAR, pour l'arrêt d'urgence
Publications :
    /cmd_vel      geometry_msgs/Twist  vitesses envoyées au robot
    /replanifier  std_msgs/Empty       demande de nouveau chemin si le robot reste bloqué

La position du robot est lue dans TF (map -> base_footprint), 20 fois par seconde.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .controle import PurePursuit, obstacle_devant


class NoeudSuivi(Node):
    def __init__(self) -> None:
        super().__init__("suivi_trajectoire")
        self.declare_parameter("distance_visee", 0.25)
        self.declare_parameter("vitesse_max", 0.15)
        self.declare_parameter("rotation_max", 1.5)
        self.declare_parameter("tolerance_arrivee", 0.08)
        self.declare_parameter("distance_securite", 0.20)       # m devant le robot
        self.declare_parameter("delai_replanification", 3.0)    # s bloqué avant de redemander un chemin
        self.declare_parameter("twist_stamped", False)          # True pour ROS 2 Jazzy
        self.declare_parameter("repere_carte", "map")
        self.declare_parameter("repere_robot", "base_footprint")

        self.ctrl = PurePursuit(
            distance_visee=self.get_parameter("distance_visee").value,
            v_max=self.get_parameter("vitesse_max").value,
            w_max=self.get_parameter("rotation_max").value,
            tolerance_arrivee=self.get_parameter("tolerance_arrivee").value)

        qos_plan = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "plan", self._sur_plan, qos_plan)
        self.create_subscription(LaserScan, "scan", self._sur_scan, qos_profile_sensor_data)
        self.stamped = self.get_parameter("twist_stamped").value
        self.pub_cmd = self.create_publisher(TwistStamped if self.stamped else Twist, "cmd_vel", 10)
        self.pub_replan = self.create_publisher(Empty, "replanifier", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan: LaserScan | None = None
        self.actif = False
        self.bloque_depuis: float | None = None
        self.replan_envoye = False
        # Mesures pour le rapport
        self.debut_mission = 0.0
        self.distance_parcourue = 0.0
        self.derniere_pose: tuple[float, float] | None = None

        self.create_timer(0.05, self._boucle)
        self.get_logger().info("Suiveur de trajectoire prêt, en attente d'un chemin sur /plan.")

    def _sur_plan(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.ctrl.nouveau_chemin(points)
        self.bloque_depuis = None
        self.replan_envoye = False
        if points:
            if not self.actif:
                self.debut_mission = time.monotonic()
                self.distance_parcourue = 0.0
                self.derniere_pose = None
            self.actif = True
            self.get_logger().info(f"Nouveau chemin de {len(points)} points : départ.")
        else:
            self.actif = False
            self._envoyer(0.0, 0.0)
            self.get_logger().warn("Chemin vide reçu : arrêt.")

    def _sur_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform(self.get_parameter("repere_carte").value,
                                                self.get_parameter("repere_robot").value,
                                                rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        cap = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, cap

    def _boucle(self) -> None:
        if not self.actif:
            return
        pose = self._pose()
        if pose is None:
            self._envoyer(0.0, 0.0)
            self.get_logger().warn("Position du robot inconnue (TF) : arrêt.", throttle_duration_sec=2.0)
            return
        x, y, cap = pose
        if self.derniere_pose is not None:
            self.distance_parcourue += math.dist((x, y), self.derniere_pose)
        self.derniere_pose = (x, y)

        cmd = self.ctrl.commande(x, y, cap)
        if cmd.arrive:
            self._envoyer(0.0, 0.0)
            self.actif = False
            duree = time.monotonic() - self.debut_mission
            self.get_logger().info(
                f"Arrivé ! {self.distance_parcourue:.2f} m parcourus en {duree:.1f} s.")
            return

        if cmd.v > 0 and self.scan is not None and obstacle_devant(
                list(self.scan.ranges), self.scan.angle_min, self.scan.angle_increment,
                self.get_parameter("distance_securite").value):
            self._envoyer(0.0, 0.0)
            self.get_logger().warn("Obstacle devant : arrêt d'urgence.", throttle_duration_sec=1.0)
            maintenant = time.monotonic()
            if self.bloque_depuis is None:
                self.bloque_depuis = maintenant
            elif (not self.replan_envoye and
                  maintenant - self.bloque_depuis > self.get_parameter("delai_replanification").value):
                self.pub_replan.publish(Empty())
                self.replan_envoye = True
            return

        self.bloque_depuis = None
        self.replan_envoye = False
        self._envoyer(cmd.v, cmd.w)

    def _envoyer(self, v: float, w: float) -> None:
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter("repere_robot").value
            msg.twist.linear.x, msg.twist.angular.z = v, w
        else:
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
        self.pub_cmd.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    noeud = NoeudSuivi()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            noeud._envoyer(0.0, 0.0)    # toujours arrêter le robot en quittant
        except Exception:
            pass
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
