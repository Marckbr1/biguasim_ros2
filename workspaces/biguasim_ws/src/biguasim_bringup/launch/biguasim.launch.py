"""Bring up a BiguaSim world + its ROS 2 sensor stack.

    ros2 launch biguasim_bringup biguasim.launch.py

Pieces (each toggleable):
  1. BiguaSim itself      run_sim:=true   -- how it starts is `biguasim_cmd`
  2. biguasim_bridge      run_bridge:=true
  3. biguasim_sensor_check run_check:=true
  4. RViz2                rviz:=false

------------------------------------------------------------------------
HOW BIGUASIM IS STARTED  (the one thing you must configure)
------------------------------------------------------------------------
This launch does not know BiguaSim's command line -- it depends on how
`install_biguasim.sh` installed it. It runs, as a plain subprocess:

    <biguasim_cmd>  [--world <world>]  [<biguasim_extra_args> ...]

Set the command one of these ways (first non-empty wins):
  * ros2 launch ... biguasim_cmd:="ros2 run <pkg> <exe>"
  * export BIGUASIM_CMD="..."   (put it in ~/biguasim/setup.bash)
  * edit DEFAULT_BIGUASIM_CMD below

If BiguaSim runs elsewhere (e.g. on a Windows host, publishing over the
network), start this launch with run_sim:=false and just use the bridge +
checker.
------------------------------------------------------------------------
"""
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Last-resort default if neither the launch arg nor $BIGUASIM_CMD is set.
DEFAULT_BIGUASIM_CMD = ""


def _launch_setup(context, *args, **kwargs):
    actions = []

    run_sim = LaunchConfiguration("run_sim").perform(context).lower() in ("1", "true", "yes")
    cmd_str = (LaunchConfiguration("biguasim_cmd").perform(context)
               or os.environ.get("BIGUASIM_CMD", "")
               or DEFAULT_BIGUASIM_CMD).strip()
    world = LaunchConfiguration("world").perform(context).strip()
    extra = LaunchConfiguration("biguasim_extra_args").perform(context).strip()

    if run_sim:
        if not cmd_str:
            actions.append(LogInfo(msg=(
                "[biguasim_bringup] run_sim:=true but no BiguaSim command is set. "
                "Pass biguasim_cmd:=..., export BIGUASIM_CMD=..., or edit "
                "DEFAULT_BIGUASIM_CMD in biguasim.launch.py. Skipping the sim.")))
        else:
            cmd = shlex.split(cmd_str)
            if world:
                cmd += ["--world", world]
            if extra:
                cmd += shlex.split(extra)
            actions.append(LogInfo(msg=f"[biguasim_bringup] starting BiguaSim: {' '.join(cmd)}"))
            actions.append(ExecuteProcess(cmd=cmd, output="screen", respawn=False))

    return actions


def generate_launch_description():
    check_share = get_package_share_directory("biguasim_sensor_check")
    bridge_share = get_package_share_directory("biguasim_bridge")
    bringup_share = get_package_share_directory("biguasim_bringup")

    args = [
        DeclareLaunchArgument("run_sim", default_value="true"),
        DeclareLaunchArgument("biguasim_cmd", default_value="",
                              description="Command that starts BiguaSim (see file header)."),
        DeclareLaunchArgument("world", default_value="",
                              description="World name/path passed as `--world <world>`."),
        DeclareLaunchArgument("biguasim_extra_args", default_value=""),
        DeclareLaunchArgument("run_bridge", default_value="true"),
        DeclareLaunchArgument("run_check", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bridge_share, "launch", "biguasim_bridge.launch.py")),
        condition=IfCondition(LaunchConfiguration("run_bridge")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    check = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(check_share, "launch", "sensor_check.launch.py")),
        condition=IfCondition(LaunchConfiguration("run_check")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz_biguasim", output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(bringup_share, "config", "biguasim.rviz")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup), bridge, check, rviz])
