"""Robot virtuel PUBLIÉ SOUS ROS 2 : permet d'essayer `station_mobile.py --slam`
(et slam_toolbox) sans le vrai TurtleBot3, sur un PC où ROS 2 Humble est installé.

    python3 tests/robot_simule_ros.py [x y cap]      # terminal 1 : le faux robot
    python3 station_mobile.py --slam --port 8080      # terminal 2 : la station

Publie /scan, /odom, /battery_state, les TF odom -> base_footprint -> base_scan,
comme turtlebot3_bringup ; obéit à /cmd_vel. La salle est celle de robot_simule.py.
"""

import math
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from robot_simule import RobotSimule


def quaternion(th):
    return 0.0, 0.0, math.sin(th / 2), math.cos(th / 2)


class FauxRobot(Node):
    """Se fait passer pour le « cerveau » auprès de RobotSimule, et publie tout sous ROS."""

    def __init__(self):
        super().__init__("robot_simule")
        self.cmd = (0.0, 0.0)
        self.p_scan = self.create_publisher(LaserScan, "scan", 10)
        self.p_odom = self.create_publisher(Odometry, "odom", 10)
        self.p_bat = self.create_publisher(BatteryState, "battery_state", 10)
        self.tf = TransformBroadcaster(self)
        statique = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "base_footprint", "base_scan"
        t.transform.translation.z = 0.17
        t.transform.rotation.w = 1.0
        statique.sendTransform(t)
        self.create_subscription(Twist, "cmd_vel", self.cb_cmd, 10)

    def cb_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    # --- interface attendue par RobotSimule ---
    def cycle(self):
        return self.cmd

    def recevoir_odom(self, x, y, th, v, w):
        now = self.get_clock().now().to_msg()
        q = quaternion(th)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, "odom", "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.p_odom.publish(o)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, "odom", "base_footprint"
        t.transform.translation.x, t.transform.translation.y = x, y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)

    def recevoir_scan(self, angle_min, inc, rmin, rmax, d):
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.get_clock().now().to_msg(), "base_scan"
        s.angle_min, s.angle_increment = angle_min, inc
        s.angle_max = angle_min + inc * (len(d) - 1)
        s.range_min, s.range_max = rmin, rmax
        s.scan_time, s.time_increment = 0.1, 0.0
        s.ranges = [float(r) for r in d]
        self.p_scan.publish(s)

    def recevoir_batterie(self, tension):
        b = BatteryState()
        b.voltage = float(tension)
        self.p_bat.publish(b)


def main():
    rclpy.init()
    noeud = FauxRobot()
    depart = tuple(float(v) for v in sys.argv[1:4]) if len(sys.argv) >= 4 else (0.0, 0.0, 0.0)
    simu = RobotSimule(noeud, depart=depart)
    simu.start()
    noeud.get_logger().info(f"Robot virtuel ROS prêt, posé en {depart}")
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        simu.arreter.set()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
