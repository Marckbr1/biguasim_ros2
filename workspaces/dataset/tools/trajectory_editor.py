#!/usr/bin/env python3
"""Trajectory editor: click on map/mapa.png -> waypoints (world x,y) ->
CSV in the format BiguaSim already understands
(`biguasim_dataset_standalone/trajectories/trajectories.py:load_waypoints_csv`,
same columns as `waypoints/example.csv`).

Needs an environment with a display (interactive matplotlib backend) -- run
it on the user's machine, not in a headless session.

Controls
--------
- Left click: add a waypoint at that point.
- Right click / 'z' key: undo the last waypoint.
- 'enter' key: save the CSV and close.
- Closing the window without 'enter': nothing is saved.
- Zoom/pan with the toolbar (magnifier / cross): while one of them is
  active, clicks do NOT add waypoints -- turn it off again to keep clicking.
- --load <csv>: start from an existing route to edit it (add/undo points).
- The bottom bar shows the WORLD x,y under the mouse.

The yaw of each waypoint is computed automatically as the heading towards
the NEXT waypoint; the last one keeps the yaw of the previous leg.

Example
-------
    python3 tools/trajectory_editor.py --map-yaml map/mapa.yaml \\
        --z -1.0 --speed 1.0 --out waypoints/my_route.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from biguasim_dataset_standalone.map.utils import (  # noqa: E402
    MapMeta, load_map_yaml, map_pixel_to_world, world_to_map_pixel)


def compute_yaw_deg(points_world: list[tuple[float, float]]) -> list[float]:
    """Heading (degrees, atan2 in BiguaSim's x,y plane) towards the next
    point; the last one repeats the yaw of the previous leg (or 0.0 if there
    is a single point)."""
    if len(points_world) < 2:
        return [0.0] * len(points_world)
    yaws = []
    for i in range(len(points_world) - 1):
        x0, y0 = points_world[i]
        x1, y1 = points_world[i + 1]
        yaws.append(math.degrees(math.atan2(y1 - y0, x1 - x0)))
    yaws.append(yaws[-1])
    return yaws


def build_waypoints_csv(
    points_map_px: list[tuple[float, float]],
    meta: MapMeta,
    out_path: Path,
    z: float = -1.0,
    speed_mps: float = 1.0,
    hold_time_s: float = 0.0,
) -> list[tuple[float, float]]:
    """points_map_px: list of (col,row) in mapa.png, in click order. Returns
    the list of world (x,y) written (so it can be plotted in the same
    script)."""
    if not points_map_px:
        raise ValueError("No waypoints to export")
    points_world = [map_pixel_to_world(col, row, meta) for col, row in points_map_px]
    yaws = compute_yaw_deg(points_world)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["waypoint_id", "x", "y", "z", "yaw_deg", "speed_mps", "hold_time_s"])
        for i, ((x, y), yaw) in enumerate(zip(points_world, yaws), start=1):
            writer.writerow([i, f"{x:.4f}", f"{y:.4f}", z, f"{yaw:.2f}", speed_mps, hold_time_s])
    return points_world


def run_editor(meta: MapMeta, map_image_path: Path, out_path: Path, z: float, speed_mps: float, hold_time_s: float,
               initial_px: list[tuple[float, float]] | None = None):
    import matplotlib
    import matplotlib.pyplot as plt

    img = plt.imread(str(map_image_path))
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img)
    ax.set_title(
        "Left click: add waypoint | right click / 'z': undo | 'enter': save and exit\n"
        f"z={z}  speed_mps={speed_mps}  hold_time_s={hold_time_s}  ->  {out_path}"
    )

    points_map_px: list[tuple[float, float]] = list(initial_px or [])
    markers = []

    # Status bar: world coordinates under the mouse instead of pixels.
    def format_coord(col, row):
        x, y = map_pixel_to_world(col, row, meta)
        return f"world x={x:.2f}  y={y:.2f}   (px {col:.0f}, {row:.0f})   waypoints: {len(points_map_px)}"
    ax.format_coord = format_coord
    texts = []

    def redraw():
        for m in markers:
            m.remove()
        for t in texts:
            t.remove()
        markers.clear()
        texts.clear()
        xs = [p[0] for p in points_map_px]
        ys = [p[1] for p in points_map_px]
        if xs:
            (line,) = ax.plot(xs, ys, "r.-", markersize=10, linewidth=1.5)
            markers.append(line)
            for i, (x, y) in enumerate(zip(xs, ys), start=1):
                texts.append(ax.annotate(str(i), (x, y), color="yellow", fontsize=9,
                                          xytext=(4, 4), textcoords="offset points"))
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes != ax:
            return
        # Zoom/pan tool active: the click belongs to the toolbar, not to us.
        toolbar = getattr(fig.canvas, "toolbar", None)
        if toolbar is not None and str(getattr(toolbar, "mode", "")):
            return
        if event.button == 1:  # left
            points_map_px.append((event.xdata, event.ydata))
            redraw()
        elif event.button == 3:  # right
            if points_map_px:
                points_map_px.pop()
                redraw()

    def on_key(event):
        if event.key == "z" and points_map_px:
            points_map_px.pop()
            redraw()
        elif event.key == "enter":
            if not points_map_px:
                print("No waypoints, nothing saved.", file=sys.stderr)
            else:
                points_world = build_waypoints_csv(points_map_px, meta, out_path, z, speed_mps, hold_time_s)
                print(f"Saved: {out_path} ({len(points_world)} waypoints)")
                for i, (x, y) in enumerate(points_world, start=1):
                    print(f"  {i}: x={x:.3f} y={y:.3f}")
            plt.close(fig)

    redraw()
    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map-yaml", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path, help="output CSV (waypoints format)")
    ap.add_argument("--z", type=float, default=-1.0, help="fixed depth for all waypoints")
    ap.add_argument("--speed", type=float, default=1.0, dest="speed_mps")
    ap.add_argument("--hold-time", type=float, default=0.0, dest="hold_time_s")
    ap.add_argument("--load", type=Path, default=None,
                    help="existing waypoints CSV to start from (edit a route)")
    args = ap.parse_args()

    meta = load_map_yaml(args.map_yaml)
    map_image_path = args.map_yaml.parent / meta.image
    initial_px = None
    if args.load is not None:
        with open(args.load, newline="") as f:
            initial_px = [world_to_map_pixel(float(r["x"]), float(r["y"]), meta) for r in csv.DictReader(f)]
        print(f"Loaded {len(initial_px)} waypoints from {args.load}")
    run_editor(meta, map_image_path, args.out, args.z, args.speed_mps, args.hold_time_s, initial_px)


if __name__ == "__main__":
    main()
