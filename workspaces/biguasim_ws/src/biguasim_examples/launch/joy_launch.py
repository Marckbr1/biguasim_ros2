## BiguaSim joystick teleop launch file
# Port of holoocean_examples/launch/joy_launch.py: launches biguasim_main +
# joy_linux + joy_biguasim (+ camera_hud) under the `biguasim` namespace.
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    base = Path(get_package_share_directory('biguasim_examples'))
    params_file = base / 'config' / 'joy_config.yaml'

    # joy_config.yaml sets params_file as a relative path ('config/config.yaml')
    # for documentation purposes, but biguasim_node/interface.py never resolves
    # relative paths -- it opens whatever string it gets as-is. Override it here
    # with the real, resolved path, same as biguasim_main.launch.py does.
    main_config = Path(get_package_share_directory('biguasim_main')) / 'config' / 'config.yaml'

    biguasim_namespace = 'biguasim'

    biguasim_main_node = launch_ros.actions.Node(
        name='biguasim_node',
        package='biguasim_main',
        executable='biguasim_node',
        namespace=biguasim_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[str(params_file), {'params_file': str(main_config)}],
    )

    joy_node = launch_ros.actions.Node(
        package='joy_linux',
        executable='joy_linux_node',
        namespace=biguasim_namespace,
        name='joy_node',
        output='screen',
        parameters=[str(params_file)],
    )

    joy_biguasim_node = launch_ros.actions.Node(
        name='joy_biguasim',
        package='biguasim_examples',
        namespace=biguasim_namespace,
        executable='joy_biguasim',
        output='screen',
        parameters=[str(params_file)],
    )

    camera_hud_node = launch_ros.actions.Node(
        name='camera_hud',
        package='biguasim_examples',
        namespace=biguasim_namespace,
        executable='camera_hud',
        output='screen',
        parameters=[str(params_file)],
    )

    return LaunchDescription([
        biguasim_main_node,
        joy_node,
        joy_biguasim_node,
        camera_hud_node,
    ])
