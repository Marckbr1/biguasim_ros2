#!/usr/bin/env python3
"""Draws a waypoint route (CSV) on the georeferenced map
(map/mapa.png + map/mapa.yaml), together with the vehicle spawn, BEFORE
launching the simulation. Does not need BiguaSim: use it to check that the
points fall in water and inside the map.

Example
-------
    python3 tools/plot_route_on_map.py --waypoints-csv waypoints/example.csv \\
        --simulator-config config/simulator_rov.yaml --out results/route_example.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from biguasim_dataset_standalone.map.utils import load_map_yaml, world_to_map_pixel  # noqa: E402
from biguasim_dataset_standalone.trajectories import load_waypoints_csv  # noqa: E402


def read_spawn(simulator_config: Path):
    cfg = yaml.safe_load(simulator_config.read_text())
    main = cfg.get("main_agent")
    for agent in cfg["agents"]:
        if agent["agent_name"] == main:
            return agent["location"]
    return cfg["agents"][0]["location"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--waypoints-csv", required=True, type=Path)
    ap.add_argument("--map-yaml", type=Path, default=PACKAGE_ROOT / "map" / "mapa.yaml")
    ap.add_argument("--simulator-config", type=Path, default=PACKAGE_ROOT / "config" / "simulator_rov.yaml")
    ap.add_argument("--out", type=Path, required=True, help="output PNG")
    ap.add_argument("--margin-m", type=float, default=25.0, help="margin around the route in the crop")
    args = ap.parse_args()

    meta = load_map_yaml(args.map_yaml)
    waypoints = load_waypoints_csv(str(args.waypoints_csv))
    spawn = read_spawn(args.simulator_config)

    x_max = meta.origin_x + meta.width * meta.resolution
    y_max = meta.origin_y + meta.height * meta.resolution
    print(f"Map: x=[{meta.origin_x:.1f}, {x_max:.1f}]  y=[{meta.origin_y:.1f}, {y_max:.1f}]  "
          f"({meta.resolution} m/px)")
    print(f"Spawn: ({spawn[0]:.2f}, {spawn[1]:.2f}, {spawn[2]:.2f})")
    outside = 0
    for i, wp in enumerate(waypoints, start=1):
        inside = meta.origin_x <= wp.x <= x_max and meta.origin_y <= wp.y <= y_max
        outside += not inside
        print(f"  wp {i:>3d}: ({wp.x:8.2f}, {wp.y:8.2f}, {wp.z:6.2f})  yaw={wp.yaw_deg:7.2f}  "
              f"v={wp.speed_mps:.2f} m/s  hold={wp.hold_time_s:.1f} s{'' if inside else '  !! OUTSIDE THE MAP'}")
    if outside:
        print(f"WARNING: {outside} waypoint(s) outside the map", file=sys.stderr)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    img = plt.imread(str(args.map_yaml.parent / meta.image))
    pts = [(spawn[0], spawn[1])] + [(wp.x, wp.y) for wp in waypoints]
    px = [world_to_map_pixel(x, y, meta) for x, y in pts]
    cols = [c for c, _ in px]
    rows = [r for _, r in px]
    margin_px = args.margin_m / meta.resolution

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img)
    ax.plot(cols[:2], rows[:2], "y--", linewidth=1.2, label="TRANSIT (spawn -> wp 1)")
    ax.plot(cols[1:], rows[1:], "r.-", linewidth=1.5, markersize=8, label="route (waypoints)")
    ax.plot(cols[0], rows[0], "g^", markersize=12, label="spawn")
    for i, (c, r) in enumerate(px[1:], start=1):
        ax.annotate(str(i), (c, r), textcoords="offset points", xytext=(4, 4), color="white", fontsize=8)
    ax.set_xlim(max(0, min(cols) - margin_px), min(meta.width, max(cols) + margin_px))
    ax.set_ylim(min(meta.height, max(rows) + margin_px), max(0, min(rows) - margin_px))
    ax.legend(loc="upper right")
    ax.set_title(f"{args.waypoints_csv.name} on {meta.image} ({len(waypoints)} waypoints)")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
