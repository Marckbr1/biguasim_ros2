#!/usr/bin/env python3
"""CLI entry point for a standalone BiguaSim data-collection run.

No ROS 2 involved anywhere in this path -- BiguaSim -> trajectory ->
sensors + ground truth -> Collector -> dataset on disk (see
docs/architecture.md). Run this on the machine that has the `biguasim`
engine installed (conda env with GPU/engine access); the dataset it
produces can be moved to a different machine that has ROS 2 for the
`dataset_to_rosbag.py` conversion step (docs/rosbag2.md).

Examples
--------
    python3 tools/collect_dataset.py --trajectory-config waypoints/example.yaml --show-view
"""

import argparse
import logging
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from biguasim_dataset_standalone.logger import setup_logger  # noqa: E402
from biguasim_dataset_standalone.runner import run_collection  # noqa: E402
from biguasim_dataset_standalone.trajectories import VALID_TRAJECTORY_TYPES  # noqa: E402

DEFAULT_SIMULATOR_CONFIG = PACKAGE_ROOT / "config" / "simulator_rov.yaml"
DEFAULT_DATASET_CONFIG = PACKAGE_ROOT / "config" / "dataset.yaml"
DEFAULT_TRAJECTORY_DIR = PACKAGE_ROOT / "waypoints"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect a standalone BiguaSim dataset (no ROS 2 required).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    traj_group = parser.add_mutually_exclusive_group()
    traj_group.add_argument(
        "--trajectory",
        choices=VALID_TRAJECTORY_TYPES,
        help=f"Name of a built-in trajectory config in {DEFAULT_TRAJECTORY_DIR} "
             f"(waypoints/<name>.yaml). One of {VALID_TRAJECTORY_TYPES}.",
    )
    traj_group.add_argument(
        "--trajectory-config",
        help="Path to a custom trajectory YAML (overrides --trajectory).",
    )
    parser.add_argument("--simulator-config", default=str(DEFAULT_SIMULATOR_CONFIG))
    parser.add_argument("--dataset-config", default=str(DEFAULT_DATASET_CONFIG))
    parser.add_argument("--show-view", action="store_true", help="Show the BiguaSim/Unreal viewport.")
    parser.add_argument(
        "--max-duration", type=float, default=None,
        help="Safety cap on simulation-time seconds, in addition to the trajectory's own length.",
    )
    parser.add_argument(
        "--max-ticks", type=int, default=None,
        help="Safety cap on number of ticks/frames recorded.",
    )
    parser.add_argument(
        "--loop", action="store_true", default=None,
        help="Loop the trajectory forever (overrides the YAML's loop_trajectory). "
             "Requires --max-duration or --max-ticks to ever stop.",
    )
    parser.add_argument("--run-name", default=None, help="Override the auto-generated run directory name.")
    parser.add_argument(
        "--waypoint-tolerance", type=float, default=None, dest="waypoint_tolerance_m",
        help="Waypoints only: radius (meters) around each waypoint within which the vehicle's "
             "ACTUAL position (ground truth) must fall to log that waypoint as reached -- see "
             "waypoint_watcher.py. If not given, uses 'arrival_tolerance_m' from the trajectory "
             "YAML, or 1.0m by default.",
    )
    parser.add_argument(
        "--stop-at-waypoint", type=int, default=None,
        help="Waypoints only: stop the collection as soon as the ACTUAL (not commanded) "
             "arrival at this waypoint is confirmed (1-indexed, e.g. 10 = stop when really "
             "reaching waypoint #10). An alternative to computing a --max-duration in seconds "
             "when what matters is 'how far it really got', not a fixed time. Keep "
             "--max-duration as a safety net in case the vehicle gets stuck before arriving.",
    )
    parser.add_argument(
        "--closed-loop-waypoints", action="store_true", default=None,
        help="Waypoints only: the COMMANDED point waits for the vehicle to actually reach "
             "each waypoint (within --waypoint-tolerance) before moving to the next one, "
             "instead of advancing purely by time -- see "
             "biguasim_dataset_standalone/trajectories/closed_loop.py. Has a per-leg safety "
             "timeout so it never hangs if the vehicle never arrives. Overrides "
             "'closed_loop_waypoints' from the trajectory YAML.",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Run the same engine+trajectory (same cmd_pos_yaw commands, same ticks) but "
             "WITHOUT creating any Collector/sensor writer -- no dataset is written to disk. "
             "Useful to iterate quickly on a waypoint CSV, or to compare against a recorded "
             "run (with --ground-truth-log in both) and confirm the motion is identical with "
             "or without the overhead of recording sensors.",
    )
    parser.add_argument(
        "--hold-x", type=float, default=None,
        help="Hold trajectory only: x of the fixed point (overrides the YAML) without editing "
             "any file -- meant for giving 'task N: stay at this point' from the command line, "
             "one after another.",
    )
    parser.add_argument("--hold-y", type=float, default=None, help="Hold trajectory only: y of the fixed point.")
    parser.add_argument("--hold-z", type=float, default=None, help="Hold trajectory only: z of the fixed point.")
    parser.add_argument(
        "--hold-yaw", type=float, default=None, dest="hold_yaw_deg",
        help="Hold trajectory only: yaw_deg of the fixed point.",
    )
    parser.add_argument(
        "--ground-truth-log", default=None,
        help="Path to a lightweight CSV (frame_id,tick,timestamp,px,py,pz) with only the "
             "actual position -- written both in normal mode and in --dry-run, so both runs of "
             "the same trajectory can be compared without parsing the full ground_truth.csv.",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    setup_logger(logging.DEBUG if args.verbose else logging.INFO)
    logger = logging.getLogger("collect_dataset")

    if args.trajectory_config:
        trajectory_config_path = args.trajectory_config
    elif args.trajectory:
        trajectory_config_path = str(DEFAULT_TRAJECTORY_DIR / f"{args.trajectory}.yaml")
    else:
        parser_choices = ", ".join(VALID_TRAJECTORY_TYPES)
        logger.error(
            "You must give a trajectory: --trajectory {%s} or --trajectory-config <path>.",
            parser_choices,
        )
        sys.exit(2)

    if args.loop is None and args.max_duration is None and args.max_ticks is None:
        logger.info(
            "No --max-duration/--max-ticks: the collection will stop on its own when "
            "the trajectory ends (or never, if loop_trajectory: true in the YAML)."
        )

    try:
        result = run_collection(
            simulator_config_path=args.simulator_config,
            dataset_config_path=args.dataset_config,
            trajectory_config_path=trajectory_config_path,
            show_view=args.show_view,
            max_duration_s=args.max_duration,
            max_ticks=args.max_ticks,
            loop_trajectory=args.loop,
            run_name_override=args.run_name,
            waypoint_tolerance_m=args.waypoint_tolerance_m,
            stop_at_waypoint=args.stop_at_waypoint,
            dry_run=args.dry_run,
            ground_truth_log_path=args.ground_truth_log,
            closed_loop_waypoints=args.closed_loop_waypoints,
            hold_x=args.hold_x,
            hold_y=args.hold_y,
            hold_z=args.hold_z,
            hold_yaw_deg=args.hold_yaw_deg,
        )
    except KeyboardInterrupt:
        logger.warning("Interrupted by the user (Ctrl-C).")
        sys.exit(130)

    if result.run_dir is not None:
        logger.info(
            "Dataset written to: %s (%d frames, %.2fs of simulation time)",
            result.run_dir, result.num_frames, result.duration_s,
        )
        print(str(result.run_dir))
    else:
        logger.info(
            "DRY RUN finished: %d frames, %.2fs of simulation time -- nothing written to disk.",
            result.num_frames, result.duration_s,
        )


if __name__ == "__main__":
    main()
