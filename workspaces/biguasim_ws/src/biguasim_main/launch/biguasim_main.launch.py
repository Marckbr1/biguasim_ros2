## BiguaSim core launch file
# Port of holoocean_main/launch/holoocean_launch.py, adapted to biguasim_main's
# single `params_file` parameter (a YAML with a nested `biguasim_scenario` key)
# instead of HoloOcean's `scenario_path` + `relative_path` pair.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=str(
            Path(os.path.join(
                get_package_share_directory('biguasim_main'),
                'config', 'config.yaml'
            ))
        ),
        description='Full path to the BiguaSim scenario/parameters YAML file',
    )

    params_file = LaunchConfiguration('params_file')

    biguasim_namespace = 'biguasim'

    biguasim_main_node = launch_ros.actions.Node(
        name='biguasim_node',
        package='biguasim_main',
        executable='biguasim_node',
        namespace=biguasim_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': params_file}],
    )

    return LaunchDescription([
        declare_params_file,
        biguasim_main_node,
    ])
