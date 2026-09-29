"""Sobe o nó SITL do BiguaSim e, opcionalmente, o MAVROS.

O ArduPilot SITL (sim_vehicle.py) e o QGroundControl sobem por fora deste
launch -- ver README.

  ros2 launch biguasim_sitl sitl.launch.py
  ros2 launch biguasim_sitl sitl.launch.py enable_sonar:=true use_mavros:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('biguasim_sitl')
    params_padrao = os.path.join(share, 'config', 'sitl_params.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=params_padrao),
        DeclareLaunchArgument('use_mavros', default_value='true'),
        # Porta do MAVROS: a 14550 fica para o QGroundControl.
        DeclareLaunchArgument('fcu_url', default_value='udp://:14551@'),
        DeclareLaunchArgument('enable_sonar', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
    ]

    sitl_node = Node(
        package='biguasim_sitl',
        executable='sitl_node',
        name='biguasim_sitl',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'enable_sonar': LaunchConfiguration('enable_sonar'),
                'enable_camera': LaunchConfiguration('enable_camera'),
            },
        ],
    )

    # MAVROS falando com o ArduPilot. Se a sua instalação exigir os arquivos
    # de plugins do apm.launch, rode o MAVROS por fora (use_mavros:=false).
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='mavros',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mavros')),
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
        }],
    )

    return LaunchDescription(args + [sitl_node, mavros_node])
