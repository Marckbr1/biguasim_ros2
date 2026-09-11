## BiguaSim waypoint follower launch file
# Port of holoocean_examples/launch/waypoint_launch.py.
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    examples_share = Path(get_package_share_directory('biguasim_examples'))
    waypoint_params = examples_share / 'config' / 'waypoint_config.yaml'

    biguasim_share = Path(get_package_share_directory('biguasim_main'))
    biguasim_params = biguasim_share / 'config' / 'config.yaml'

    biguasim_namespace = 'biguasim'

    biguasim_main_node = launch_ros.actions.Node(
        name='biguasim_node',
        package='biguasim_main',
        executable='biguasim_node',
        namespace=biguasim_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(biguasim_params)}],
    )

    waypoint_node = launch_ros.actions.Node(
        name='waypoint_follower',
        package='biguasim_examples',
        namespace=biguasim_namespace,
        executable='waypoint_follower',
        output='screen',
        parameters=[str(waypoint_params)],
    )

    return LaunchDescription([
        biguasim_main_node,
        waypoint_node,
    ])
