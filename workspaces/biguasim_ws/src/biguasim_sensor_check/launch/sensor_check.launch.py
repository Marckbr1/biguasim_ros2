"""Runs the BiguaSim sensor checker.

  ros2 launch biguasim_sensor_check sensor_check.launch.py

Override the topic list or timings:

  ros2 launch biguasim_sensor_check sensor_check.launch.py report_period:=1.0
  ros2 run biguasim_sensor_check sensor_check_node --ros-args \
      -p 'sensors:=["/my/topic|sensor_msgs/msg/Imu|30"]'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    report_period_arg = DeclareLaunchArgument("report_period", default_value="2.0")
    exit_after_arg = DeclareLaunchArgument("exit_after", default_value="0.0")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    node = Node(
        package="biguasim_sensor_check",
        executable="sensor_check_node",
        name="biguasim_sensor_check",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "report_period": LaunchConfiguration("report_period"),
            "exit_after": LaunchConfiguration("exit_after"),
        }],
    )

    return LaunchDescription([
        report_period_arg, exit_after_arg, use_sim_time_arg, node,
    ])
