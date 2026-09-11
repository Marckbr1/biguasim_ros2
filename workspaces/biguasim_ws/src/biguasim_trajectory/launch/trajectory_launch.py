"""Levanta BiguaSim (escenario de 1 agente, cmd_pos_yaw) y le hace recorrer
una trayectoria de cuadrado o circulo. Sin sonar/RViz -- ver
trajectory_sonar_launch.py para eso.

  ros2 launch biguasim_trajectory trajectory_launch.py
  ros2 launch biguasim_trajectory trajectory_launch.py trajectory_type:=circle circle_radius:=8.0
  ros2 launch biguasim_trajectory trajectory_launch.py trajectory_type:=square square_side:=15.0
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    trajectory_share = Path(get_package_share_directory('biguasim_trajectory'))
    scenario_file = trajectory_share / 'config' / 'trajectory_scenario.yaml'
    main_share = get_package_share_directory('biguasim_main')

    biguasim_namespace = 'biguasim'

    declare_args = [
        DeclareLaunchArgument('agent_ros_id', default_value='auv0_id0'),
        DeclareLaunchArgument('trajectory_type', default_value='square',
                              description="'square' o 'circle'"),
        DeclareLaunchArgument('square_side', default_value='10.0'),
        DeclareLaunchArgument('circle_radius', default_value='5.0'),
        DeclareLaunchArgument('num_loops', default_value='1'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        DeclareLaunchArgument('dive_speed', default_value='0.3'),
        DeclareLaunchArgument('target_depth', default_value='-3.0'),
        DeclareLaunchArgument('corner_wait_time', default_value='3.0'),
        DeclareLaunchArgument('control_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('loop_trajectory', default_value='true'),
        DeclareLaunchArgument('startup_timeout_sec', default_value='10.0'),
    ]

    biguasim_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(main_share, 'launch', 'biguasim_main.launch.py')),
        launch_arguments={'params_file': str(scenario_file)}.items(),
    )

    trajectory_generator_node = Node(
        name='trajectory_generator', package='biguasim_trajectory', executable='trajectory_generator',
        namespace=biguasim_namespace, output='screen', emulate_tty=True,
        parameters=[{
            'agent_ros_id': LaunchConfiguration('agent_ros_id'),
            'trajectory_type': LaunchConfiguration('trajectory_type'),
            'square_side': ParameterValue(LaunchConfiguration('square_side'), value_type=float),
            'circle_radius': ParameterValue(LaunchConfiguration('circle_radius'), value_type=float),
            'num_loops': ParameterValue(LaunchConfiguration('num_loops'), value_type=int),
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'dive_speed': ParameterValue(LaunchConfiguration('dive_speed'), value_type=float),
            'target_depth': ParameterValue(LaunchConfiguration('target_depth'), value_type=float),
            'corner_wait_time': ParameterValue(LaunchConfiguration('corner_wait_time'), value_type=float),
            'control_rate_hz': ParameterValue(LaunchConfiguration('control_rate_hz'), value_type=float),
            'loop_trajectory': ParameterValue(LaunchConfiguration('loop_trajectory'), value_type=bool),
            'startup_timeout_sec': ParameterValue(LaunchConfiguration('startup_timeout_sec'), value_type=float),
        }])

    # Espera a que biguasim_node termine de levantar el mundo antes de
    # empezar a comandar (ademas de su propio timeout interno vía
    # startup_timeout_sec, como respaldo).
    delayed_trajectory_node = TimerAction(period=5.0, actions=[trajectory_generator_node])

    return LaunchDescription(declare_args + [biguasim_main, delayed_trajectory_node])
