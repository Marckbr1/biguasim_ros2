"""Standalone glue between `trajectories.py` (pure, ported verbatim from
`biguasim_trajectory` in the ROS 2 workspace -- see
docs/trajectories.md) and a BiguaSim `env.step()` loop that has no ROS 2
underneath it at all.

The ROS node (`trajectory_generator_node.py`) captured the vehicle's spawn
pose from the first `DynamicsSensor/Odom` message it received over a
topic. Here there is no topic: the spawn pose is read directly from the
same `simulator.yaml` that `biguasim.make()` itself uses to place the
vehicle (`agents[].location` / `rotation`), so the very first control
command can already be correct (station-keep at spawn) instead of having
to wait for a first sensor reading -- see docs/trajectories.md for why
this also sidesteps the "stale command from a previous run" issue
documented in docs/TRAYECTORIAS_Y_WAYPOINTS.md section 9 (each standalone
run is a fresh `biguasim.make()` process, nothing to be stale from).
"""

from __future__ import annotations

from dataclasses import dataclass

from .trajectories import (
    TrajectoryPlan,
    build_circle_shape,
    build_figure_eight_shape,
    build_hold_shape,
    build_line_shape,
    build_square_shape,
    build_waypoints_shape,
    load_waypoints_csv,
    make_dive_segment,
    make_transit_segment,
    sample_segment,
)

VALID_TRAJECTORY_TYPES = ('line', 'square', 'circle', 'figure_eight', 'waypoints', 'hold')

# Mirrors trajectory_generator_node.py's declare_parameter() defaults
# exactly (docs/trajectories.md) -- control_rate_hz/startup_timeout_sec are
# dropped because they're artifacts of the ROS timer, not the trajectory
# math: the standalone loop re-samples the plan every simulation tick,
# which is a finer resolution than the ROS node's 10 Hz control timer.
DEFAULT_PARAMS = {
    'line_length': 10.0,
    'square_side': 10.0,
    'circle_radius': 5.0,
    'figure_eight_radius': 5.0,
    'waypoints_csv_path': '',
    'num_loops': 1,
    'speed': 1.0,
    'dive_speed': 0.3,
    'target_depth': -3.0,
    'corner_wait_time': 3.0,
    'figure_eight_transition_time': 2.0,
    'loop_trajectory': False,
    'hold_duration_s': 5.0,
}


@dataclass
class Origin:
    x: float
    y: float
    z: float
    yaw_deg: float = 0.0


def build_plan(trajectory_type: str, params: dict, origin: Origin) -> TrajectoryPlan:
    """Build a TrajectoryPlan for `trajectory_type`, replicating
    trajectory_generator_node.py's `_build_plan`/`_build_waypoints_plan`
    exactly (same segment construction, same origin handling) but taking
    the origin as a plain argument instead of a ROS subscription."""
    if trajectory_type not in VALID_TRAJECTORY_TYPES:
        raise ValueError(
            f"unknown trajectory_type '{trajectory_type}', must be "
            f"one of {VALID_TRAJECTORY_TYPES}")

    p = {**DEFAULT_PARAMS, **params}

    if trajectory_type == 'waypoints':
        if not p['waypoints_csv_path']:
            raise ValueError(
                "trajectory_type='waypoints' requires 'waypoints_csv_path' "
                "in the trajectory config (see "
                "waypoints/example.yaml).")
        waypoints = load_waypoints_csv(p['waypoints_csv_path'])
        first = waypoints[0]
        approach = make_transit_segment(
            x0=origin.x, y0=origin.y, z0=origin.z, yaw0_deg=origin.yaw_deg,
            x1=first.x, y1=first.y, z1=first.z, yaw1_deg=first.yaw_deg,
            speed=first.speed_mps)
        segments = [approach] + build_waypoints_shape(waypoints)
        return TrajectoryPlan(segments, origin_x=0.0, origin_y=0.0)

    if trajectory_type == 'hold':
        # Absolute fixed point (same frame as ground truth) -- if x/y/z/yaw_deg
        # are not given in the YAML, the fixed point is the spawn declared in
        # 'location'/'rotation' of the simulator config.
        x = p.get('x', origin.x)
        y = p.get('y', origin.y)
        z = p.get('z', origin.z)
        yaw_deg = p.get('yaw_deg', origin.yaw_deg)
        segments = build_hold_shape(x, y, z, yaw_deg, p['hold_duration_s'])
        return TrajectoryPlan(segments, origin_x=0.0, origin_y=0.0)

    speed = p['speed']
    depth = p['target_depth']

    if trajectory_type == 'line':
        shape_segments = build_line_shape(p['line_length'], speed, depth)
    elif trajectory_type == 'square':
        shape_segments = build_square_shape(
            p['square_side'], speed, depth, p['corner_wait_time'])
    elif trajectory_type == 'circle':
        shape_segments = build_circle_shape(
            p['circle_radius'], speed, depth, p['num_loops'])
    else:  # figure_eight
        shape_segments = build_figure_eight_shape(
            p['figure_eight_radius'], speed, depth, p['num_loops'],
            p['figure_eight_transition_time'])

    first_sample = sample_segment(shape_segments[0], 0.0)
    dive_duration = max(1.0, abs(origin.z - depth) / p['dive_speed'])
    dive = make_dive_segment(
        x=0.0, y=0.0,
        z_start=origin.z, z_target=depth,
        yaw_start_deg=0.0, yaw_target_deg=first_sample.yaw_deg,
        duration=dive_duration)

    return TrajectoryPlan([dive] + shape_segments, origin_x=origin.x, origin_y=origin.y)
