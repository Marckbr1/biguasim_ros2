"""Bring up a BiguaSim world + its ROS 2 sensor stack.

    ros2 launch biguasim_bringup biguasim.launch.py

Pieces (each toggleable):
  1. BiguaSim itself      run_sim:=true   -- see "HOW BIGUASIM IS STARTED" below
  2. biguasim_bridge      run_bridge:=true
  3. biguasim_sensor_check run_check:=true
  4. RViz2                rviz:=false

------------------------------------------------------------------------
HOW BIGUASIM IS STARTED
------------------------------------------------------------------------
As of the biguasim_main merge, the default is to launch biguasim_main's own
launch file (biguasim_node -- the ROS-owned biguasim.make(...) bridge),
using `biguasim_params_file` for its scenario. This replaced the previous
"you must tell it the command line" behavior, which is now a fallback for
the case where BiguaSim is instead driven by an external process you don't
control from this workspace (e.g. a standalone script, or a different
package installed by `install_biguasim.sh`):

  * Preferred: biguasim_main is used automatically when
    use_biguasim_main:=true (default) and biguasim_cmd is NOT set.
  * Fallback / override: set biguasim_cmd:="ros2 run <pkg> <exe>" (or
    export BIGUASIM_CMD=...) to run an external command instead. This still
    takes priority over biguasim_main if set.
  * If BiguaSim runs elsewhere entirely (e.g. on a Windows host, publishing
    over the network), start this launch with run_sim:=false and just use
    the bridge + checker.
------------------------------------------------------------------------
"""
import os
import shlex

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Last-resort default if neither the launch arg nor $BIGUASIM_CMD is set,
# and use_biguasim_main is false (or biguasim_main isn't built).
DEFAULT_BIGUASIM_CMD = ""


def _launch_setup(context, *args, **kwargs):
    actions = []

    run_sim = LaunchConfiguration("run_sim").perform(context).lower() in ("1", "true", "yes")
    use_biguasim_main = LaunchConfiguration("use_biguasim_main").perform(context).lower() in ("1", "true", "yes")
    cmd_str = (LaunchConfiguration("biguasim_cmd").perform(context)
               or os.environ.get("BIGUASIM_CMD", "")
               or DEFAULT_BIGUASIM_CMD).strip()
    world = LaunchConfiguration("world").perform(context).strip()
    extra = LaunchConfiguration("biguasim_extra_args").perform(context).strip()
    biguasim_params_file = LaunchConfiguration("biguasim_params_file").perform(context).strip()

    if not run_sim:
        return actions

    if cmd_str:
        # Explicit override always wins, even if biguasim_main is available.
        cmd = shlex.split(cmd_str)
        if world:
            cmd += ["--world", world]
        if extra:
            cmd += shlex.split(extra)
        actions.append(LogInfo(msg=f"[biguasim_bringup] starting BiguaSim via biguasim_cmd: {' '.join(cmd)}"))
        actions.append(ExecuteProcess(cmd=cmd, output="screen", respawn=False))
        return actions

    if use_biguasim_main:
        try:
            main_share = get_package_share_directory("biguasim_main")
        except PackageNotFoundError:
            actions.append(LogInfo(msg=(
                "[biguasim_bringup] use_biguasim_main:=true but the biguasim_main "
                "package isn't built/sourced. Build it (colcon build) or pass "
                "biguasim_cmd:=... / use_biguasim_main:=false. Skipping the sim.")))
            return actions

        main_launch_args = {"use_sim_time": LaunchConfiguration("use_sim_time")}
        if biguasim_params_file:
            main_launch_args["params_file"] = biguasim_params_file

        actions.append(LogInfo(msg="[biguasim_bringup] starting BiguaSim via biguasim_main"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(main_share, "launch", "biguasim_main.launch.py")),
            launch_arguments=main_launch_args.items(),
        ))
        return actions

    actions.append(LogInfo(msg=(
        "[biguasim_bringup] run_sim:=true but no BiguaSim command is set and "
        "use_biguasim_main:=false. Pass biguasim_cmd:=..., export BIGUASIM_CMD=..., "
        "or leave use_biguasim_main:=true. Skipping the sim.")))
    return actions


def generate_launch_description():
    check_share = get_package_share_directory("biguasim_sensor_check")
    bridge_share = get_package_share_directory("biguasim_bridge")
    bringup_share = get_package_share_directory("biguasim_bringup")

    args = [
        DeclareLaunchArgument("run_sim", default_value="true"),
        DeclareLaunchArgument("use_biguasim_main", default_value="true",
                              description="Start BiguaSim via biguasim_main's own launch file "
                                          "(default). Ignored if biguasim_cmd is set."),
        DeclareLaunchArgument("biguasim_params_file", default_value="",
                              description="Scenario/parameters YAML passed to biguasim_main "
                                          "(default: biguasim_main's own config/config.yaml)."),
        DeclareLaunchArgument("biguasim_cmd", default_value="",
                              description="Explicit override: command that starts BiguaSim "
                                          "as an external process (see file header)."),
        DeclareLaunchArgument("world", default_value="",
                              description="World name/path passed as `--world <world>` (biguasim_cmd only)."),
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
