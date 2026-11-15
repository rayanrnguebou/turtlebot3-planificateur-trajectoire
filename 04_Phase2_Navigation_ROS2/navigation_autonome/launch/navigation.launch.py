"""Lance le planificateur et le suiveur de trajectoire.

    ros2 launch navigation_autonome navigation.launch.py use_sim_time:=true   # Gazebo
    ros2 launch navigation_autonome navigation.launch.py                      # vrai robot
    ros2 launch navigation_autonome navigation.launch.py algorithme:=rrt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parametres = os.path.join(get_package_share_directory("navigation_autonome"), "config", "parametres.yaml")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true avec Gazebo, false sur le vrai robot"),
        DeclareLaunchArgument("algorithme", default_value="astar",
                              description="astar, dijkstra ou rrt"),
        Node(package="navigation_autonome", executable="planificateur", name="planificateur",
             output="screen",
             parameters=[parametres, {"use_sim_time": use_sim_time,
                                      "algorithme": LaunchConfiguration("algorithme")}]),
        Node(package="navigation_autonome", executable="suivi", name="suivi_trajectoire",
             output="screen",
             parameters=[parametres, {"use_sim_time": use_sim_time}]),
    ])
