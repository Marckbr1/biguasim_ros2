#!/usr/bin/env python3
"""Small CLI to convert map pixel <-> BiguaSim world coordinates.

The logic itself (MapMeta, load_map_yaml, map_pixel_to_world,
world_to_map_pixel) lives in `biguasim_dataset_standalone.map.utils`; this
file is only the command-line entry point.

Usage
-----
    python3 tools/map_utils.py --map-yaml map/mapa.yaml --pixel COL ROW
    python3 tools/map_utils.py --map-yaml map/mapa.yaml --world X Y
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

from biguasim_dataset_standalone.map.utils import (  # noqa: E402
    load_map_yaml,
    map_pixel_to_world,
    world_to_map_pixel,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map-yaml", required=True, type=Path)
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--pixel", type=float, nargs=2, metavar=("COL", "ROW"))
    group.add_argument("--world", type=float, nargs=2, metavar=("X", "Y"))
    args = ap.parse_args()

    meta = load_map_yaml(args.map_yaml)
    if args.pixel:
        x, y = map_pixel_to_world(args.pixel[0], args.pixel[1], meta)
        print(f"pixel ({args.pixel[0]}, {args.pixel[1]}) -> world (x={x:.3f}, y={y:.3f})")
    else:
        col, row = world_to_map_pixel(args.world[0], args.world[1], meta)
        print(f"world ({args.world[0]}, {args.world[1]}) -> pixel (col={col:.1f}, row={row:.1f})")


if __name__ == "__main__":
    main()
