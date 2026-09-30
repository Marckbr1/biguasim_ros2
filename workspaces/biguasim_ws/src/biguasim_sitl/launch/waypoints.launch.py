"""Sobe o nó de waypoints.

O sim_vehicle.py, o biguasim_sitl e o MAVROS precisam estar no ar.

  ros2 launch biguasim_sitl waypoints.launch.py
  ros2 launch biguasim_sitl waypoints.launch.py modo:=mission
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('biguasim_sitl')
    params_padrao = os.path.join(share, 'config', 'waypoints.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=params_padrao),
        DeclareLaunchArgument('modo', default_value='guided'),
    ]

    wp_node = Node(
        package='biguasim_sitl',
        executable='waypoint_node',
        name='biguasim_waypoints',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'modo': LaunchConfiguration('modo')},
        ],
    )

    return LaunchDescription(args + [wp_node])
