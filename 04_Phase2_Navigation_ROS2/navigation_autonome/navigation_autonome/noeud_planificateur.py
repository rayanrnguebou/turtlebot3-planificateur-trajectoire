"""Nœud ROS 2 : calcule un chemin avec nos algorithmes (A*, Dijkstra ou RRT).

Abonnements :
    /map          nav_msgs/OccupancyGrid   carte (SLAM ou map_server)
    /goal_pose    geometry_msgs/PoseStamped  but cliqué dans RViz (« 2D Goal Pose »)
    /replanifier  std_msgs/Empty           demande de recalcul (envoyée par le suiveur)
Publication :
    /plan         nav_msgs/Path            chemin en mètres, repère « map »

La position du robot est lue dans TF (map -> base_footprint).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .algos import astar, dijkstra, lisser, rrt
from .algos.rrt import longueur
from .carte_ros import chemin_en_metres, grille_depuis_occupancy

ALGOS = {"astar": astar, "dijkstra": dijkstra, "rrt": rrt}


class NoeudPlanificateur(Node):
    def __init__(self) -> None:
        super().__init__("planificateur")
        self.declare_parameter("algorithme", "astar")          # astar | dijkstra | rrt
        self.declare_parameter("rayon_securite", 0.15)         # m : rayon du Burger + marge
        self.declare_parameter("lissage", True)
        self.declare_parameter("inconnu_libre", False)
        self.declare_parameter("carte_transient_local", False)  # True avec map_server
        self.declare_parameter("repere_carte", "map")
        self.declare_parameter("repere_robot", "base_footprint")

        durabilite = (DurabilityPolicy.TRANSIENT_LOCAL
                      if self.get_parameter("carte_transient_local").value
                      else DurabilityPolicy.VOLATILE)
        qos_carte = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=durabilite)
        qos_plan = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, "map", self._sur_carte, qos_carte)
        self.create_subscription(PoseStamped, "goal_pose", self._sur_but, 10)
        self.create_subscription(Empty, "replanifier", self._sur_replanifier, 10)
        self.pub_plan = self.create_publisher(Path, "plan", qos_plan)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.carte: OccupancyGrid | None = None
        self.but: tuple[float, float] | None = None
        self.get_logger().info("Planificateur prêt : clique un but avec « 2D Goal Pose » dans RViz.")

    # ------------------------------------------------------------------ entrées

    def _sur_carte(self, msg: OccupancyGrid) -> None:
        if self.carte is None:
            self.get_logger().info(
                f"Carte reçue : {msg.info.width}×{msg.info.height} cases de {msg.info.resolution:.3f} m")
        self.carte = msg

    def _sur_but(self, msg: PoseStamped) -> None:
        self.but = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Nouveau but : x={self.but[0]:.2f} m, y={self.but[1]:.2f} m")
        self.planifier()

    def _sur_replanifier(self, _msg: Empty) -> None:
        if self.but is not None:
            self.get_logger().info("Replanification demandée par le suiveur.")
            self.planifier()

    # ------------------------------------------------------------------ calcul

    def _position_robot(self) -> tuple[float, float] | None:
        repere_carte = self.get_parameter("repere_carte").value
        repere_robot = self.get_parameter("repere_robot").value
        try:
            t = self.tf_buffer.lookup_transform(repere_carte, repere_robot, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"Position du robot inconnue ({repere_carte} -> {repere_robot}) : {e}")
            return None
        return t.transform.translation.x, t.transform.translation.y

    def planifier(self) -> None:
        if self.carte is None:
            self.get_logger().warn("Pas encore de carte sur /map : lance le SLAM ou map_server.")
            return
        depart_m = self._position_robot()
        if depart_m is None or self.but is None:
            return

        info = self.carte.info
        if abs(info.origin.orientation.z) > 1e-3:
            self.get_logger().warn("Carte tournée par rapport au repère map : non géré, résultat faux possible.")
        grille = grille_depuis_occupancy(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, self.carte.data,
            inconnu_libre=self.get_parameter("inconnu_libre").value)
        grille = grille.gonfler(self.get_parameter("rayon_securite").value)

        case_but = grille.metres_vers_case(*self.but)
        if not grille.dans_carte(case_but):
            self.get_logger().error("Le but est en dehors de la carte.")
            return
        # Le robot ou le but peuvent tomber dans la marge de sécurité : on prend la case libre la plus proche.
        depart = grille.case_libre_proche(grille.metres_vers_case(*depart_m))
        arrivee = grille.case_libre_proche(case_but)

        nom = self.get_parameter("algorithme").value
        if nom not in ALGOS:
            self.get_logger().error(f"Algorithme inconnu « {nom} » (choix : {', '.join(ALGOS)})")
            return
        res = ALGOS[nom](grille, depart, arrivee)
        if not res.trouve:
            self.get_logger().error(f"{res.algo} : aucun chemin trouvé ({res.temps_s * 1000:.0f} ms).")
            self.pub_plan.publish(Path(header=self._entete()))     # chemin vide = arrêt
            return

        chemin = lisser(grille, res.chemin) if self.get_parameter("lissage").value else res.chemin
        points = chemin_en_metres(grille, chemin)
        if arrivee == case_but:
            points[-1] = self.but        # finir exactement sur le point cliqué
        self.get_logger().info(
            f"{res.algo} : chemin de {longueur(points):.2f} m, {len(points)} points, "
            f"{res.noeuds_explores} nœuds explorés, {res.temps_s * 1000:.0f} ms")
        self.pub_plan.publish(self._message_chemin(points))

    # ------------------------------------------------------------------ sortie

    def _entete(self):
        msg = Path()
        msg.header.frame_id = self.get_parameter("repere_carte").value
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg.header

    def _message_chemin(self, points) -> Path:
        msg = Path(header=self._entete())
        for i, (x, y) in enumerate(points):
            p = PoseStamped(header=msg.header)
            p.pose.position.x, p.pose.position.y = x, y
            # Orientation : direction vers le point suivant (utile pour l'affichage RViz).
            if i + 1 < len(points):
                cap = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
                p.pose.orientation.z, p.pose.orientation.w = math.sin(cap / 2), math.cos(cap / 2)
            else:
                p.pose.orientation.w = 1.0
            msg.poses.append(p)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    noeud = NoeudPlanificateur()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
