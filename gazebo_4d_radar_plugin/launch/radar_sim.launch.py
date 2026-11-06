"""Launch Gazebo with a radar world, plus optional RViz and the ego-velocity estimator.

    ros2 launch gazebo_4d_radar_plugin radar_sim.launch.py                      # demo world, GUI
    ros2 launch gazebo_4d_radar_plugin radar_sim.launch.py world:=radar_terrain_world.sdf
    ros2 launch gazebo_4d_radar_plugin radar_sim.launch.py headless:=true rviz:=false

Data is published in the radar frame (frame_id radar_link), so RViz uses it as
the fixed frame and no TF tree is required.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gz(context):
    pkg = get_package_share_directory("gazebo_4d_radar_plugin")
    world = LaunchConfiguration("world").perform(context)
    if not os.path.isabs(world):
        world = os.path.join(pkg, "worlds", world)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    cmd = ["gz", "sim", "-r", world] + (["-s"] if headless else [])
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_4d_radar_plugin")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="radar_demo_world.sdf",
                              description="World file name in share/worlds, or an absolute path"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run the Gazebo server only (no GUI)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("ego_velocity", default_value="true",
                              description="Run the RANSAC ego-velocity estimator node"),
        OpaqueFunction(function=_gz),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", os.path.join(pkg, "rviz", "radar_view.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="gazebo_4d_radar_plugin", executable="radar_ego_velocity_estimator.py",
             name="radar_ego_velocity_estimator", output="screen",
             condition=IfCondition(LaunchConfiguration("ego_velocity"))),
    ])
