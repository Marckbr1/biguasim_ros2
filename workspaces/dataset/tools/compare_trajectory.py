#!/usr/bin/env python3
"""Compares the DESIRED trajectory (waypoint CSV, the commanded plan)
against the ACTUAL trajectory (ground_truth.csv of a waypoints run of
`collect_dataset.py`), and optionally plots both on map/mapa.png.

Reuses the project's own `load_waypoints_csv` (same parser the trajectory
engine uses) instead of re-implementing the format -- see
`biguasim_dataset_standalone/trajectories/trajectories.py`. Complements
(does not replace) `waypoint_watcher.py`: that module runs LIVE during the
collection and only confirms arrival/no-arrival against a tolerance; this
script runs AFTERWARDS, on the recorded CSV, and reports the closest actual
approach to each waypoint (whether or not it got within tolerance), plus
aggregate metrics and a plot.

Example
-------
    python3 tools/compare_trajectory.py \\
        --waypoints-csv waypoints/example.csv \\
        --run-dir dataset/colecta_simple/runs/<run_id> \\
        --map-yaml map/mapa.yaml \\
        --out results/comparison.png
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from biguasim_dataset_standalone.trajectories import load_waypoints_csv  # noqa: E402
from biguasim_dataset_standalone.waypoint_watcher import DEFAULT_ARRIVAL_TOLERANCE_M  # noqa: E402
from biguasim_dataset_standalone.map.utils import load_map_yaml, world_to_map_pixel  # noqa: E402


def load_ground_truth_xyz(csv_path: Path) -> np.ndarray:
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    return np.array([[float(r["px"]), float(r["py"]), float(r["pz"])] for r in rows])


def closest_approach(waypoints, actual_xyz: np.ndarray) -> list[dict]:
    """For each waypoint: minimum actual distance reached (3D) and the index
    of the tick where it happened."""
    results = []
    for i, wp in enumerate(waypoints, start=1):
        target = np.array([wp.x, wp.y, wp.z])
        dists = np.linalg.norm(actual_xyz - target, axis=1)
        idx = int(np.argmin(dists))
        results.append({
            "waypoint": i, "target": target, "min_dist_m": float(dists[idx]),
            "at_tick": idx, "at_pos": actual_xyz[idx],
        })
    return results


def path_length(xyz: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--waypoints-csv", required=True, type=Path, help="DESIRED waypoint CSV (the commanded plan)")
    ap.add_argument("--run-dir", required=True, type=Path, help="ACTUAL run (dataset/.../runs/<id>)")
    ap.add_argument("--tolerance-m", type=float, default=DEFAULT_ARRIVAL_TOLERANCE_M,
                     help=f"'arrival' radius -- same default as waypoint_watcher.py ({DEFAULT_ARRIVAL_TOLERANCE_M}m)")
    ap.add_argument("--map-yaml", type=Path, default=None, help="map/mapa.yaml -- if given together with --out, plots both trajectories on the map")
    ap.add_argument("--out", type=Path, default=None, help="output PNG with the plot (requires --map-yaml)")
    args = ap.parse_args()

    waypoints = load_waypoints_csv(str(args.waypoints_csv))
    actual_xyz = load_ground_truth_xyz(args.run_dir / "ground_truth" / "ground_truth.csv")

    results = closest_approach(waypoints, actual_xyz)
    reached = [r for r in results if r["min_dist_m"] <= args.tolerance_m]

    desired_xyz = np.array([[wp.x, wp.y, wp.z] for wp in waypoints])
    desired_len = path_length(desired_xyz)
    actual_len = path_length(actual_xyz)

    print(f"Waypoints: {len(waypoints)}  |  actual run: {len(actual_xyz)} ticks")
    print(f"'Arrival' tolerance: {args.tolerance_m} m (same default as waypoint_watcher.py)")
    print()
    for r in results:
        flag = "OK " if r["min_dist_m"] <= args.tolerance_m else "!! "
        print(f"  {flag}waypoint {r['waypoint']:>3d}: target=({r['target'][0]:.2f}, {r['target'][1]:.2f}, "
              f"{r['target'][2]:.2f})  min_actual_distance={r['min_dist_m']:.3f} m  (tick {r['at_tick']})")
    print()
    print(f"Reached within tolerance: {len(reached)}/{len(results)}")
    dists = [r["min_dist_m"] for r in results]
    print(f"Approach distance -- mean: {np.mean(dists):.3f} m  max: {np.max(dists):.3f} m")
    print(f"Path length -- desired: {desired_len:.1f} m  actual: {actual_len:.1f} m "
          f"(difference: {actual_len - desired_len:+.1f} m)")

    if args.out is not None:
        if args.map_yaml is None:
            print("WARNING: --out without --map-yaml, no plot generated", file=sys.stderr)
        else:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            meta = load_map_yaml(args.map_yaml)
            map_img_path = args.map_yaml.parent / meta.image
            img = plt.imread(str(map_img_path))

            desired_px = np.array([world_to_map_pixel(x, y, meta) for x, y, _ in desired_xyz])
            actual_px = np.array([world_to_map_pixel(x, y, meta) for x, y, _ in actual_xyz])

            fig, ax = plt.subplots(figsize=(10, 10))
            ax.imshow(img)
            ax.plot(desired_px[:, 0], desired_px[:, 1], "r.-", label="desired (waypoints)", linewidth=1.5, markersize=6)
            ax.plot(actual_px[:, 0], actual_px[:, 1], "c-", label="actual (ground truth)", linewidth=1.5)
            ax.legend(loc="upper right")
            ax.set_title(f"Desired vs actual trajectory -- {len(reached)}/{len(results)} waypoints within {args.tolerance_m}m")
            args.out.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(args.out, dpi=150, bbox_inches="tight")
            print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
