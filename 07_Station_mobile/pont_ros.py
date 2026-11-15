"""Lien entre le cerveau et ROS 2 Humble, sur la Raspberry Pi.

Topics utilisés (publiés par `ros2 launch turtlebot3_bringup robot.launch.py`) :
    /scan           sensor_msgs/LaserScan     LiDAR LDS-03, 10 Hz
    /odom           nav_msgs/Odometry         odométrie des roues + IMU
    /battery_state  sensor_msgs/BatteryState  tension RÉELLE de la batterie
    /cmd_vel        geometry_msgs/Twist       vitesses envoyées aux moteurs
    /sound          turtlebot3_msgs/srv/Sound mélodies du buzzer de l'OpenCR
Avec l'option --slam (slam_toolbox, lancé par la station) :
    /map            nav_msgs/OccupancyGrid    carte de slam_toolbox
    TF map -> odom                            sa correction de l'odométrie
    /slam_toolbox/serialize_map               enregistrement du graphe de poses
"""

from __future__ import annotations

import math
import os
import threading
import time

# Mélodies du buzzer OpenCR (turtlebot3_msgs/srv/Sound) :
# 0 OFF, 1 ON, 2 LOW_BATTERY, 3 ERROR, 4 BUTTON1, 5 BUTTON2
SONS_OPENCR = {"depart": 4, "arrive": 1, "obstacle": 3, "urgence": 3,
               "echec": 2, "batterie": 2}


def demarrer(cerveau):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, LaserScan
    try:
        from turtlebot3_msgs.srv import Sound
    except ImportError:
        Sound = None

    class StationMobile(Node):
        def __init__(self):
            super().__init__("station_mobile")
            self.slam = None
            if cerveau.slam is not None:
                self._preparer_slam()
            self.pub = self.create_publisher(Twist, "cmd_vel", 10)
            self.create_subscription(LaserScan, "scan", self.cb_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, "odom", self.cb_odom, qos_profile_sensor_data)
            self.create_subscription(BatteryState, "battery_state", self.cb_batterie,
                                     qos_profile_sensor_data)
            self.client_son = self.create_client(Sound, "sound") if Sound else None
            self.zeros_restants = 0
            self.create_timer(0.1, self.boucle)               # 10 Hz
            self.get_logger().info("Station mobile connectée à ROS 2")

        def cb_scan(self, m):
            cerveau.recevoir_scan(m.angle_min, m.angle_increment, m.range_min,
                                  m.range_max, list(m.ranges))

        def cb_odom(self, m):
            q = m.pose.pose.orientation
            theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            cerveau.recevoir_odom(m.pose.pose.position.x, m.pose.pose.position.y, theta,
                                  m.twist.twist.linear.x, m.twist.twist.angular.z)

        def cb_batterie(self, m):
            cerveau.recevoir_batterie(m.voltage)

        def envoyer(self, v, w):
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self.pub.publish(msg)

        def boucle(self):
            try:
                v, w = cerveau.cycle()
            except Exception as e:                      # jamais de robot « fou »
                self.get_logger().error(f"erreur dans le cerveau : {e!r}")
                v, w = 0.0, 0.0
            if v != 0.0 or w != 0.0:
                self.envoyer(v, w)
                self.zeros_restants = 5
            elif self.zeros_restants > 0:
                # Après un mouvement : quelques « zéro », puis silence, pour ne
                # pas gêner un autre programme (téléopération clavier...).
                self.envoyer(0.0, 0.0)
                self.zeros_restants -= 1

        # ---------------- slam_toolbox ----------------
        def _preparer_slam(self):
            from nav_msgs.msg import OccupancyGrid
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from slam_toolbox.srv import SerializePoseGraph
            from tf2_ros import Buffer, TransformListener
            from slam_lanceur import LanceurSlam
            qos_carte = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, "map", self.cb_carte, qos_carte)
            self.tf = Buffer()
            self.ecoute_tf = TransformListener(self.tf, self)
            self.client_serialiser = self.create_client(SerializePoseGraph, "slam_toolbox/serialize_map")
            self.SerializePoseGraph = SerializePoseGraph
            self.slam = LanceurSlam(journal=self.get_logger().info)
            self.verrou_slam = threading.Lock()
            self.debut_session = 0.0           # instant (horloge ROS) du dernier lancement
            self.dernier_tf = 0.0
            self.create_timer(0.1, self.boucle_slam)
            threading.Thread(target=self.travail_slam, daemon=True).start()

        def cb_carte(self, m):
            if self.debut_session == 0.0 or \
                    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 < self.debut_session:
                return                         # carte d'une session précédente
            q = m.info.origin.orientation
            th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            cerveau.recevoir_carte_slam(m.info.resolution, m.info.origin.position.x,
                                        m.info.origin.position.y, th, m.info.width,
                                        m.info.height, m.data)

        def boucle_slam(self):
            from rclpy.time import Time
            if self.debut_session == 0.0:
                return
            try:
                tr = self.tf.lookup_transform("map", "odom", Time())
            except Exception:
                return
            st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if st < self.debut_session or st <= self.dernier_tf:
                return                         # transformation périmée (ancienne session)
            self.dernier_tf = st
            q = tr.transform.rotation
            th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            cerveau.recevoir_tf_slam(tr.transform.translation.x, tr.transform.translation.y, th)

        def travail_slam(self):
            """Exécute dans l'ordre les demandes du cerveau (lancer, enregistrer, arrêter)."""
            while rclpy.ok():
                with self.verrou_slam:
                    self._executer(cerveau.prendre_demandes_slam())
                time.sleep(0.2)

        def _executer(self, demandes):
                for d in demandes:
                    try:
                        if d[0] == "lancer":
                            self.debut_session = 0.0
                            self.slam.lancer(d[1], d[2])
                            self.debut_session = self.get_clock().now().nanoseconds * 1e-9
                        elif d[0] == "arreter":
                            self.debut_session = 0.0
                            self.slam.arreter()
                        elif d[0] == "serialiser":
                            self._serialiser(d[1])
                    except Exception as e:
                        self.get_logger().error(f"slam_toolbox ({d[0]}) : {e!r}")

        def _serialiser(self, base):
            if not self.client_serialiser.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn("slam_toolbox ne répond pas : graphe non enregistré")
                return
            req = self.SerializePoseGraph.Request()
            req.filename = os.path.abspath(base)
            fut = self.client_serialiser.call_async(req)
            fin = time.monotonic() + 20
            while not fut.done() and time.monotonic() < fin:
                time.sleep(0.05)
            r = fut.result() if fut.done() else None
            if r is None or r.result != 0:
                self.get_logger().warn(f"graphe slam_toolbox non enregistré ({base})")
            else:
                self.get_logger().info(f"graphe slam_toolbox enregistré : {base}.posegraph")

        def jouer(self, type_son):
            if not self.client_son or type_son not in SONS_OPENCR:
                return
            if not self.client_son.service_is_ready():
                return
            req = Sound.Request()
            req.value = SONS_OPENCR[type_son]
            self.client_son.call_async(req)

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    noeud = StationMobile()
    cerveau.son = noeud.jouer
    threading.Thread(target=rclpy.spin, args=(noeud,), daemon=True).start()

    def arreter():
        try:
            for _ in range(3):
                noeud.envoyer(0.0, 0.0)
                time.sleep(0.05)
        except Exception:
            pass
        finally:
            if noeud.slam is not None:
                # Dernier enregistrement du graphe demandé par la station, PUIS arrêt
                with noeud.verrou_slam:
                    noeud._executer([d for d in cerveau.prendre_demandes_slam() if d[0] == "serialiser"])
                    noeud.slam.arreter()
            try:
                noeud.destroy_node()
            finally:
                rclpy.try_shutdown()

    return arreter
