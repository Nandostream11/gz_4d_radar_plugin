"""Example: radar on a car driving down a street, shown in Gazebo and RViz.

    ros2 launch gazebo_4d_radar_plugin radar_street_demo.launch.py
    ros2 launch gazebo_4d_radar_plugin radar_street_demo.launch.py gui:=false    # RViz only
    ros2 launch gazebo_4d_radar_plugin radar_street_demo.launch.py follow:=false # fixed Gazebo camera

RViz shows the live radar scan coloured by Doppler (red = approaching, blue =
receding; the static world has the ego vehicle's own closing speed) and a
4-second trail in the world frame, so the street builds up as the car drives.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    ex = os.path.join(get_package_share_directory("gazebo_4d_radar_plugin"), "examples")
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    follow = LaunchConfiguration("follow").perform(context).lower() in ("1", "true", "yes")

    gz_cmd = ["gz", "sim", "-r", os.path.join(ex, "worlds", "radar_street_demo.sdf")]
    if not gui:
        gz_cmd.append("-s")
    driver_cmd = ["python3", os.path.join(ex, "scripts", "demo_driver.py")]
    if gui and follow:
        driver_cmd.append("--follow")

    return [
        ExecuteProcess(cmd=gz_cmd, output="screen"),
        ExecuteProcess(cmd=driver_cmd, output="screen",
                       condition=IfCondition(LaunchConfiguration("drive"))),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", os.path.join(ex, "rviz", "radar_street_demo.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Show the Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="true", description="Show RViz"),
        DeclareLaunchArgument("drive", default_value="true", description="Run the looping scenario"),
        DeclareLaunchArgument("follow", default_value="true",
                              description="Gazebo camera follows the ego vehicle"),
        OpaqueFunction(function=_setup),
    ])
