"""Runs both BiguaSim normalization nodes.

  odom_bridge_node : /biguasim/rov0_id0/DynamicsSensor/Odom -> /pose_gt + /cmd_vel
  sonar_bridge_node: /biguasim/rov0_id0/ImagingSonar        -> /son (mono8 fan)

  ros2 launch biguasim_bridge biguasim_bridge.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="/biguasim/rov0_id0/DynamicsSensor/Odom")
    sonar_topic_arg = DeclareLaunchArgument(
        "sonar_topic", default_value="/biguasim/rov0_id0/ImagingSonar")
    intensity_scale_arg = DeclareLaunchArgument("intensity_scale", default_value="600.0")
    output_height_arg = DeclareLaunchArgument("output_height", default_value="512")

    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    odom_bridge = Node(
        package="biguasim_bridge",
        executable="odom_bridge_node",
        name="odom_bridge_node",
        output="screen",
        parameters=[use_sim_time, {
            "input_topic": LaunchConfiguration("odom_topic"),
            "pose_topic": "/pose_gt",
            "cmd_vel_topic": "/cmd_vel",
            "pose_frame_id": "odom",
            "cmd_vel_frame_id": "base_link",
        }],
    )

    sonar_bridge = Node(
        package="biguasim_bridge",
        executable="sonar_bridge_node",
        name="sonar_bridge_node",
        output="screen",
        parameters=[use_sim_time, {
            "input_topic": LaunchConfiguration("sonar_topic"),
            "output_topic": "/son",
            "output_frame_id": "son",
            "source_image": "raw_image",
            "intensity_scale": LaunchConfiguration("intensity_scale"),
            "output_height": LaunchConfiguration("output_height"),
            "range_axis_near_first": True,
            "azimuth_flip": False,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg, odom_topic_arg, sonar_topic_arg,
        intensity_scale_arg, output_height_arg,
        odom_bridge, sonar_bridge,
    ])
