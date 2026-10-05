#!/usr/bin/env python3
"""Formal pixel (drone image) -> map (orthorectified mapa.png) -> world
(BiguaSim metric coordinates) transformation.

It is DELIBERATELY a two-step chain, not one:

  pixel (drone perspective image, 2048x2048)
      --[ homography from biguasim_dataset_standalone.georeference (--export-ortho) ]-->
  map (mapa.png, top-down, constant resolution in meters/pixel)
      --[ affine (resolution, origin) from mapa.yaml, THIS function ]-->
  world (x, y meters, same frame as ground_truth.csv)

The first step (pixel->map) was already solved when mapa.png was generated
(no need to repeat it per click). What a trajectory editor needs live,
click by click, is ONLY the second step -- map->world -- which is a plain
affine transformation (no perspective, because mapa.png is already
orthorectified): that is why it lives in its own small module instead of
reusing pixel_to_world (which assumes the original drone perspective image,
not the already-projected map).

mapa.yaml convention (same schema as ROS map_server, adapted):
  origin.x, origin.y = LOWER-LEFT corner of the map in the world
                        (minimum x,y).
  resolution          = meters per pixel (square).
  row 0 of the PNG (top of the image) = MAXIMUM y edge of the world;
  the last row (bottom) = origin.y (MINIMUM y).

LIBRARY module (no CLI): the command-line tool that uses this lives in
`tools/map_utils.py`.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class MapMeta:
    image: str
    resolution: float
    origin_x: float
    origin_y: float
    origin_z: float
    coordinate_system: str
    width: int
    height: int


def load_map_yaml(yaml_path: Path) -> MapMeta:
    """Reads mapa.yaml and the actual size of the associated PNG (same
    directory, 'image' field)."""
    raw = yaml.safe_load(Path(yaml_path).read_text())
    image_path = Path(yaml_path).parent / raw["image"]
    width, height = _png_size(image_path)
    return MapMeta(
        image=raw["image"],
        resolution=float(raw["resolution"]),
        origin_x=float(raw["origin"]["x"]),
        origin_y=float(raw["origin"]["y"]),
        origin_z=float(raw["origin"].get("z", 0.0)),
        coordinate_system=raw.get("coordinate_system", "world"),
        width=width,
        height=height,
    )


def _png_size(path: Path) -> tuple[int, int]:
    try:
        import cv2
        img = cv2.imread(str(path))
        if img is None:
            raise FileNotFoundError(path)
        return img.shape[1], img.shape[0]
    except ImportError:
        import struct
        with open(path, "rb") as f:
            f.seek(16)
            w, h = struct.unpack(">II", f.read(8))
        return w, h


def map_pixel_to_world(col: float, row: float, meta: MapMeta) -> tuple[float, float]:
    """Map (col,row) (mapa.png, top-left origin like any image) -> world
    (x,y)."""
    x = meta.origin_x + col * meta.resolution
    y = meta.origin_y + (meta.height - row) * meta.resolution
    return x, y


def world_to_map_pixel(x: float, y: float, meta: MapMeta) -> tuple[float, float]:
    """Inverse of map_pixel_to_world."""
    col = (x - meta.origin_x) / meta.resolution
    row = meta.height - (y - meta.origin_y) / meta.resolution
    return col, row
