"""Igual que trajectory_launch.py (cuadrado/circulo, cmd_pos_yaw) pero ademas
activa el sonar: biguasim_bridge normaliza el ImagingSonar del agente a
sensor_msgs/Image (/son) y, por default, se abre RViz2 con ese display ya
armado (biguasim_bringup/config/biguasim.rviz).

  ros2 launch biguasim_trajectory trajectory_sonar_launch.py
  ros2 launch biguasim_trajectory trajectory_sonar_launch.py trajectory_type:=circle circle_radius:=8.0
  ros2 launch biguasim_trajectory trajectory_sonar_launch.py rviz:=false   # solo bridge, sin ventana
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    trajectory_share = get_package_share_directory('biguasim_trajectory')
    bridge_share = get_package_share_directory('biguasim_bridge')
    bringup_share = get_package_share_directory('biguasim_bringup')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    # Declarado tambien aqui (ademas de en trajectory_launch.py) para poder
    # usarlo al armar el topico del sonar_bridge mas abajo.
    agent_ros_id_arg = DeclareLaunchArgument('agent_ros_id', default_value='auv0_id0')
    agent_ros_id = LaunchConfiguration('agent_ros_id')

    # El resto de los argumentos de trajectory_launch.py (trajectory_type,
    # square_side, circle_radius, ...) se heredan de la linea de comandos tal
    # cual -- no hace falta redeclararlos aqui.
    trajectory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(trajectory_share, 'launch', 'trajectory_launch.py')),
        launch_arguments={'agent_ros_id': agent_ros_id}.items(),
    )

    sonar_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'biguasim_bridge.launch.py')),
        launch_arguments={
            'odom_topic': PathJoinSubstitution(['/biguasim', agent_ros_id, 'DynamicsSensor', 'Odom']),
            'sonar_topic': PathJoinSubstitution(['/biguasim', agent_ros_id, 'ImagingSonar']),
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_biguasim_trajectory', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(bringup_share, 'config', 'biguasim.rviz')],
    )

    return LaunchDescription([rviz_arg, agent_ros_id_arg, trajectory, sonar_bridge, rviz])
