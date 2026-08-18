import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_radar = get_package_share_directory('gazebo_4d_radar_plugin')

    # Path to demo world
    world_path = os.path.join(pkg_radar, 'worlds', 'radar_demo_world.sdf')
    rviz_config_path = os.path.join(pkg_radar, 'rviz', 'radar_view.rviz')

    # Gazebo Sim launch execution
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # RViz2 Node for 4D PointCloud and Doppler visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Static transform publisher for sensor frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_static_tf',
        arguments=['0', '0', '1.5', '0', '0.785398', '0', 'world', 'radar_link'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        static_tf,
        rviz_node
    ])
