"""Standalone collection loop: BiguaSim -> trajectory -> sensors + ground
truth -> Collector -> dataset on disk. No ROS 2 import anywhere in this
module or anything it imports -- see docs/architecture.md.

Architecture (see docs/architecture.md for the full diagram)::

    biguasim.make()  ->  env.step(cmd_pos_yaw)  ->  state dict (per tick)
        |                                              |
        |  cmd = TrajectoryPlan.sample(state['t'])     |  timestamp = state['t']
        v                                              v
    trajectories.py (pure, ported from biguasim_trajectory)   Collector.save()
                                                               -> WriterFactory writers
                                                               -> dataset/<name>/runs/<run_id>/
"""

from __future__ import annotations

import csv
import logging
import time
from pathlib import Path
from typing import Optional

import yaml

from .collector import Collector
from .config.dataset_cfg import DatasetConfig
from .config.simulator_cfg import SimulatorConfig
from .factory import WriterFactory
from .metadata import RunMetadata
from .trajectories import ClosedLoopWaypointPlan, Origin, build_plan, load_waypoints_csv
from .trajectories.closed_loop import DEFAULT_TIMEOUT_FACTOR
from .waypoint_watcher import DEFAULT_ARRIVAL_TOLERANCE_M, WaypointArrivalWatcher

logger = logging.getLogger(__name__)

# Mirrored from biguasim_main/interface.py::COMMAND_MAP (docs/architecture.md)
# -- NOT imported from there, because that module pulls in rclpy/sensor_msgs
# transitively (biguasim_main.sensor_data_encode). Only the one entry the
# trajectory system actually drives is needed here.
COMMAND_DIM = {
    'cmd_pos_yaw': 4,
}


class CollectionResult:
    def __init__(self, run_dir: Optional[Path], num_frames: int, duration_s: float,
                 start_time_s: float, end_time_s: float):
        self.run_dir = run_dir
        self.num_frames = num_frames
        self.duration_s = duration_s
        self.start_time_s = start_time_s
        self.end_time_s = end_time_s


def load_yaml(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def run_collection(
    simulator_config_path: str,
    dataset_config_path: str,
    trajectory_config_path: str,
    show_view: bool = False,
    max_duration_s: Optional[float] = None,
    max_ticks: Optional[int] = None,
    loop_trajectory: Optional[bool] = None,
    run_name_override: Optional[str] = None,
    waypoint_tolerance_m: Optional[float] = None,
    stop_at_waypoint: Optional[int] = None,
    dry_run: bool = False,
    ground_truth_log_path: Optional[str] = None,
    closed_loop_waypoints: Optional[bool] = None,
    hold_x: Optional[float] = None,
    hold_y: Optional[float] = None,
    hold_z: Optional[float] = None,
    hold_yaw_deg: Optional[float] = None,
) -> CollectionResult:
    """Run one standalone data-collection session end to end.

    `max_duration_s`/`max_ticks` are safety caps on top of the trajectory's
    own natural length (it stops on its own once `TrajectorySample.finished`
    is True, unless `loop_trajectory` is set) -- see docs/trajectories.md.

    `waypoint_tolerance_m`: only used when `trajectory_type == 'waypoints'`.
    Radius (meters) around each waypoint's real-world position within which
    the vehicle's ACTUAL ground-truth position must fall for that waypoint
    to be logged as reached (see waypoint_watcher.py) -- this is independent
    of, and does not affect, the time-driven `cmd_pos_yaw` the trajectory
    planner sends (which has no arrival detection of its own, see
    docs/TRAYECTORIAS_Y_WAYPOINTS.md section 2). Falls back to the trajectory
    YAML's own 'arrival_tolerance_m' key if set, else
    waypoint_watcher.DEFAULT_ARRIVAL_TOLERANCE_M.

    `stop_at_waypoint`: only used when `trajectory_type == 'waypoints'`. Stop
    the collection as soon as this waypoint (1-indexed, matching the
    "Waypoint N/total" log messages) is REALLY reached (real ground-truth
    position within `waypoint_tolerance_m`) -- an alternative to guessing a
    `max_duration_s` when what you actually care about is "stop once the
    vehicle has truly gotten this far", not a fixed sim-time budget. Keep
    `max_duration_s`/`max_ticks` set too as a safety net: if the vehicle
    gets stuck before reaching this waypoint (see docs/informe_final.md,
    "Limitaciones conocidas"), this condition never fires on its own.
    With `closed_loop_waypoints: true` (see below), the meaning sharpens:
    it stops right when that waypoint is genuinely confirmed, not merely
    when the plan has moved past it.

    `trajectory_params['closed_loop_waypoints']` (bool, YAML-only -- see
    configs/trajectories/*.yaml, default False): only for
    `trajectory_type == 'waypoints'`. When True, uses
    `trajectories.ClosedLoopWaypointPlan` instead of the classic
    `TrajectoryPlan` for commanding -- the commanded target HOLDS at each
    waypoint (does not advance to the next one) until the vehicle's real
    ground-truth position is actually within `waypoint_tolerance_m` of it,
    or until a per-leg timeout (`trajectory_params['closed_loop_timeout_factor']`,
    default `closed_loop.DEFAULT_TIMEOUT_FACTOR`) is exceeded. This is the
    first real modification of the time-driven trajectory-following
    behavior in this project (trajectories.py itself, and the classic
    `waypoints` path, are untouched -- see closed_loop.py). When False
    (default), behavior is identical to before: `WaypointArrivalWatcher` is
    a passive observer that never influences what gets commanded.

    `hold_x`/`hold_y`/`hold_z`/`hold_yaw_deg`: only used when
    `trajectory_type == 'hold'`. Override the fixed point on the fly, from
    the CLI, without editing configs/trajectories/hold.yaml -- meant for
    running many "go to this point and hold" tasks back to back (e.g. a
    drone: task 1 = hover at spawn, task 2 = hover at a new point someone
    just gave you) without a new YAML file per point. Any not given falls
    back to the YAML's own value, or the vehicle's spawn (see
    planner.py::build_plan, trajectory_type == 'hold').

    `dry_run`: skip Collector/WriterFactory/RunMetadata entirely -- runs the
    EXACT same engine+trajectory loop (same simulator_cfg, same
    build_plan(), same cmd_pos_yaw commands, same env.step() calls) but
    writes no dataset to disk. Useful to iterate on a trajectory/waypoint
    CSV quickly, or to check whether writing sensors to disk has any effect
    on the resulting motion -- it shouldn't, since env.step() advances by a
    fixed simulation tick regardless of how long the Python-side write
    takes (docs/synchronization.md), but this makes that verifiable instead
    of assumed. `CollectionResult.run_dir` is None in this mode.

    `ground_truth_log_path`: if given (in EITHER mode), write a lightweight
    CSV (frame_id,tick,timestamp,px,py,pz) with just the ground-truth
    position -- so a dry run and a real collection of the same trajectory
    can be diffed directly, without parsing the full 19-column
    ground_truth.csv for the non-dry-run case.
    """
    import biguasim  # imported lazily: only needed on the collection side,
    # not by tools that merely read an already-collected dataset (validator,
    # rosbag2 converter) -- keeps `import biguasim_dataset_standalone` cheap
    # and avoids requiring the biguasim SDK/engine on a pure-ROS2 machine.

    sim_raw = load_yaml(simulator_config_path)
    trajectory_raw = load_yaml(trajectory_config_path)

    simulator_cfg = SimulatorConfig(sim_raw)
    dataset_cfg = None
    if not dry_run:
        dataset_raw = load_yaml(dataset_config_path)
        dataset_cfg = DatasetConfig(dataset_raw)

    trajectory_type = trajectory_raw["type"]
    trajectory_params = {k: v for k, v in trajectory_raw.items() if k != "type"}
    if trajectory_params.get("waypoints_csv_path"):
        # Resolve relative to the trajectory YAML's own directory (not the
        # process CWD), so 'configs/trajectories/waypoints.yaml' can point
        # at '../examples/foo.csv' regardless of where collect_dataset.py
        # is invoked from.
        csv_path = Path(trajectory_params["waypoints_csv_path"])
        if not csv_path.is_absolute():
            csv_path = (Path(trajectory_config_path).resolve().parent / csv_path).resolve()
        trajectory_params["waypoints_csv_path"] = str(csv_path)
    if loop_trajectory is not None:
        trajectory_params["loop_trajectory"] = loop_trajectory
    if closed_loop_waypoints is not None:
        trajectory_params["closed_loop_waypoints"] = closed_loop_waypoints
    if hold_x is not None:
        trajectory_params["x"] = hold_x
    if hold_y is not None:
        trajectory_params["y"] = hold_y
    if hold_z is not None:
        trajectory_params["z"] = hold_z
    if hold_yaw_deg is not None:
        trajectory_params["yaw_deg"] = hold_yaw_deg
    loop = bool(trajectory_params.get("loop_trajectory", False))

    agent_name = simulator_cfg.main_agent or next(iter(simulator_cfg.agents))
    agent_cfg = simulator_cfg.get_agent(agent_name)
    control_abstraction = agent_cfg.raw.get("control_abstraction")
    if control_abstraction != "cmd_pos_yaw":
        raise ValueError(
            f"Agent '{agent_name}' declares control_abstraction="
            f"'{control_abstraction}', but the standalone trajectory runner "
            f"can only command 'cmd_pos_yaw' (x,y,z,yaw_deg). "
            f"Fix the simulator config."
        )

    location = agent_cfg.raw.get("location", [0.0, 0.0, 0.0])
    rotation = agent_cfg.raw.get("rotation", [0.0, 0.0, 0.0])
    origin = Origin(x=location[0], y=location[1], z=location[2], yaw_deg=rotation[2])

    plan = build_plan(trajectory_type, trajectory_params, origin)

    if stop_at_waypoint is not None and trajectory_type != "waypoints":
        raise ValueError(
            f"stop_at_waypoint only applies with trajectory_type='waypoints' "
            f"(the current trajectory is '{trajectory_type}')."
        )

    waypoint_watcher: Optional[WaypointArrivalWatcher] = None
    closed_loop_plan: Optional[ClosedLoopWaypointPlan] = None
    if trajectory_type == "waypoints":
        tolerance = (
            waypoint_tolerance_m
            if waypoint_tolerance_m is not None
            else trajectory_params.get("arrival_tolerance_m", DEFAULT_ARRIVAL_TOLERANCE_M)
        )
        closed_loop = bool(trajectory_params.get("closed_loop_waypoints", False))

        if closed_loop:
            timeout_factor = trajectory_params.get("closed_loop_timeout_factor", DEFAULT_TIMEOUT_FACTOR)
            closed_loop_plan = ClosedLoopWaypointPlan(
                load_waypoints_csv(trajectory_params["waypoints_csv_path"]),
                origin.x, origin.y, origin.z,
                tolerance_m=tolerance, timeout_factor=timeout_factor,
            )
            num_waypoints = len(closed_loop_plan.legs)
            logger.info(
                "Waypoint advance CLOSED ON ACTUAL POSITION enabled: %d waypoints, "
                "tolerance=%.2fm, timeout=%.1fx the nominal duration per leg -- the commanded "
                "point now waits for the actual arrival before advancing (see closed_loop.py).",
                num_waypoints, tolerance, timeout_factor,
            )
        else:
            waypoint_watcher = WaypointArrivalWatcher.from_csv(
                trajectory_params["waypoints_csv_path"], tolerance_m=tolerance,
            )
            num_waypoints = len(waypoint_watcher.waypoints)
            logger.info(
                "Actual arrival detection enabled: %d waypoints, tolerance=%.2fm "
                "(by actual GT position, independent of the commanded plan -- see waypoint_watcher.py; "
                "to make the CONTROL wait for the actual arrival, use closed_loop_waypoints: true).",
                num_waypoints, tolerance,
            )

        if stop_at_waypoint is not None:
            if not (1 <= stop_at_waypoint <= num_waypoints):
                raise ValueError(
                    f"stop_at_waypoint={stop_at_waypoint} out of range "
                    f"(this trajectory has {num_waypoints} waypoints, 1..{num_waypoints})."
                )
            logger.info(
                "Will stop as soon as the actual arrival at waypoint %d/%d is confirmed "
                "(--stop-at-waypoint) -- keep max_duration_s/max_ticks as a safety net "
                "in case the vehicle gets stuck before.",
                stop_at_waypoint, num_waypoints,
            )

    # ------------------------------------------------------------------
    # Run directory + metadata (written BEFORE the simulation starts, so a
    # bad config fails fast without ever opening the engine -- same
    # philosophy as trajectory_generator_node validating its CSV at
    # __init__ time, see docs/trajectories.md). Skipped entirely in
    # dry_run: no run_dir, no RunMetadata, no Collector/writers -- the loop
    # below still runs the identical engine+trajectory steps.
    # ------------------------------------------------------------------
    run_dir: Optional[Path] = None
    run_name = "(dry-run)"
    run_metadata = None
    collector = None
    if dry_run:
        logger.info("DRY RUN: no dataset will be created (no Collector/sensor writers).")
    else:
        if run_name_override:
            run_dir = dataset_cfg.output_root / dataset_cfg.dataset_name / "runs" / run_name_override
            run_dir.mkdir(parents=True, exist_ok=True)
            run_name = run_name_override
        else:
            run_dir, run_name = dataset_cfg.create_run()

        run_metadata = RunMetadata(run_dir, run_name)
        run_metadata.save(
            simulator_cfg=simulator_cfg,
            dataset_cfg=dataset_cfg,
            simulator_cfg_path=simulator_config_path,
            dataset_cfg_path=dataset_config_path,
            trajectory_cfg_path=trajectory_config_path,
            trajectory_type=trajectory_type,
            trajectory_params=trajectory_params,
        )

        collector = Collector(output_dir=run_dir)
        for writer in WriterFactory.create_writer(
            dataset_cfg=dataset_cfg,
            simulator_cfg=simulator_cfg,
            dataset_root=run_dir,
        ):
            collector.register_writer(writer)

    logger.info(
        "Run '%s' -> %s | trayectoria=%s | agente=%s spawn=(%.2f, %.2f, %.2f) yaw=%.1f",
        run_name, run_dir, trajectory_type, agent_name, origin.x, origin.y, origin.z, origin.yaw_deg,
    )

    gt_log_file = None
    gt_log_writer = None
    if ground_truth_log_path:
        gt_log_file = open(ground_truth_log_path, "w", newline="")
        gt_log_writer = csv.writer(gt_log_file)
        gt_log_writer.writerow(["frame_id", "tick", "timestamp", "px", "py", "pz"])
        logger.info("Lightweight ground truth log (position only): %s", ground_truth_log_path)

    cmd = [origin.x, origin.y, origin.z, origin.yaw_deg]
    frame_id = 0
    t_start_sim: Optional[float] = None
    last_t = 0.0
    wall_clock_start = time.monotonic()

    try:
        with biguasim.make(scenario_cfg=simulator_cfg.raw, show_viewport=show_view) as env:
            while True:
                # 't' lives at the TOP level of env.step()'s return dict,
                # a sibling of the agent name -- NOT inside state[agent][0]
                # with the per-sensor keys, despite the pseudocode in
                # docs/biguasim_dataset_audit.md section 6 suggesting
                # otherwise. Confirmed empirically against the real engine
                # (biguasim v1.0.0) in this session, not assumed from docs.
                full_state = env.step(cmd)
                t = full_state["t"]
                state = full_state[agent_name][0]
                if t_start_sim is None:
                    t_start_sim = t
                last_t = t

                gt_pos = None
                if "DynamicsSensor" in state:
                    gt_pos = tuple(float(v) for v in state["DynamicsSensor"][6:9])  # px,py,pz
                if waypoint_watcher is not None and gt_pos is not None:
                    waypoint_watcher.update(*gt_pos, t)
                if gt_log_writer is not None and gt_pos is not None:
                    gt_log_writer.writerow([frame_id, env._num_ticks, t, *gt_pos])

                # timestamp is ALWAYS simulation time (state['t']), never
                # wall-clock -- see docs/synchronization.md and the bug
                # this fixes, docs/AUDITORIA_STANDALONE_DATASET.md section K.
                if collector is not None:
                    collector.save(
                        frame_id=frame_id,
                        tick=env._num_ticks,
                        timestamp=t,
                        state=state,
                    )
                frame_id += 1

                elapsed = t - t_start_sim
                if closed_loop_plan is not None:
                    real_pos = gt_pos if gt_pos is not None else (cmd[0], cmd[1], cmd[2])
                    sample = closed_loop_plan.sample(elapsed, real_pos, loop=loop)
                else:
                    sample = plan.sample(elapsed, loop=loop)
                cmd = [sample.x, sample.y, sample.z, sample.yaw_deg]

                if sample.finished and not loop:
                    logger.info("Trajectory '%s' completed at t=%.2fs (sim).", trajectory_type, elapsed)
                    break
                if max_duration_s is not None and elapsed >= max_duration_s:
                    logger.info("max_duration_s=%.2fs reached, stopping.", max_duration_s)
                    break
                if max_ticks is not None and frame_id >= max_ticks:
                    logger.info("max_ticks=%d reached, stopping.", max_ticks)
                    break
                if stop_at_waypoint is not None:
                    reached = (
                        closed_loop_plan.is_reached(stop_at_waypoint)
                        if closed_loop_plan is not None
                        else waypoint_watcher.is_reached(stop_at_waypoint)
                    )
                    if reached:
                        logger.info(
                            "Target waypoint %d/%d confirmed (actual arrival), stopping "
                            "(--stop-at-waypoint).", stop_at_waypoint, num_waypoints,
                        )
                        break
    finally:
        if collector is not None:
            collector.close()
        if gt_log_file is not None:
            gt_log_file.close()
        if waypoint_watcher is not None:
            waypoint_watcher.log_summary()
        elif closed_loop_plan is not None:
            closed_loop_plan.log_summary()
        wall_elapsed = time.monotonic() - wall_clock_start
        duration_s = last_t - (t_start_sim or 0.0)
        logger.info(
            "Collection finished: %d frames, %.2fs of simulation time "
            "(%.2fs of wall clock, factor %.2fx).",
            frame_id, duration_s, wall_elapsed,
            (duration_s / wall_elapsed) if wall_elapsed > 0 else float("nan"),
        )
        if run_metadata is not None:
            run_metadata.update_run_summary(
                duration_s=duration_s,
                start_time_s=t_start_sim or 0.0,
                end_time_s=last_t,
                num_frames=frame_id,
            )

    return CollectionResult(
        run_dir=run_dir,
        num_frames=frame_id,
        duration_s=duration_s,
        start_time_s=t_start_sim or 0.0,
        end_time_s=last_t,
    )
