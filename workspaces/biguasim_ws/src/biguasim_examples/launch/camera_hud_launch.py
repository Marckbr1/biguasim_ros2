## BiguaSim camera HUD launch file (standalone)
# Also included inside joy_launch.py; this one is for running the HUD on
# its own against an already-running biguasim_main / recorded bag.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    agent_name_arg = DeclareLaunchArgument('agent_name', default_value='auv0')

    camera_hud_node = launch_ros.actions.Node(
        name='camera_hud',
        package='biguasim_examples',
        namespace='biguasim',
        executable='camera_hud',
        output='screen',
        parameters=[{'agent_name': LaunchConfiguration('agent_name')}],
    )

    return LaunchDescription([
        agent_name_arg,
        camera_hud_node,
    ])
