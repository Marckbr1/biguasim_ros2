#!/usr/bin/env python3
"""Pure georeferencing geometry: pixel (drone image) <-> BiguaSim world,
plus the camera rotations/intrinsics needed for it.

Method
------
Geometric (pinhole) ray projection, assuming the water is a horizontal
plane z = z_water:

1. Camera intrinsics from FOV + resolution (assumes horizontal FOV and
   square pixels, same as `CameraInfoEncoder` in
   `src/biguasim_main/biguasim_main/sensor_data_encode.py`).
2. World<-camera orientation = R_world_body (quaternion from
   `ground_truth.csv`, columns qx,qy,qz,qw) composed with R_body_socket
   (the camera socket's `rotation` offset, degrees, [roll,pitch,yaw]
   convention around FIXED x,y,z axes -- see
   `biguasim_docs/usage/units-and-coordinates.rst`) and a fixed
   BODY_TO_OPTICAL matrix that goes from body axes (x=forward, y=left,
   z=up) to optical camera axes (x=right, y=down, z=forward-into-scene).
3. Ray-plane intersection for each pixel (pixel_to_world) or direct
   projection of a world point to a pixel (world_to_pixel, the inverse).

This is exact given the model (no distortion, perfectly horizontal and
known water plane); in practice the error comes from: drone pose noise,
real vs. nominal FOV, and waves (the plane is not perfectly horizontal).

LIBRARY module (no CLI): used by `biguasim_dataset_standalone.sonar.geometry`
(rotation_from_quat) and by the georeferencing tools of ../colecta_biguasim.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


# Fixed matrix: camera axes (x=right, y=down, z=forward-into-scene, standard
# optical convention) expressed in BODY/socket axes (x=forward, y=left,
# z=up) BEFORE applying the socket's own rotation offset (rotation=[0,0,0]).
# Derived from the fact that, looking along body +x with +z up
# (right-handed), "right" = -y, "down" = -z, "forward" = +x:
#   x_cam = -y_body ; y_cam = -z_body ; z_cam = +x_body
BODY_TO_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rotation_xyz_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """[roll,pitch,yaw] rotation in degrees around FIXED x,y,z axes
    (BiguaSim convention, see biguasim_docs/usage/units-and-coordinates.rst).
    Equivalent to R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotation_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion (x,y,z,w) -> rotation matrix (same order as
    `msg.pose.pose.orientation` / columns qx,qy,qz,qw of ground_truth.csv)."""
    q = np.array([qx, qy, qz, qw], dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def camera_intrinsics(width: int, height: int, fov_deg: float) -> tuple[float, float, float, float]:
    """(fx, fy, cx, cy) -- horizontal FOV, square pixels (fx == fy), same
    criterion as CameraInfoEncoder."""
    fov_rad = np.deg2rad(fov_deg)
    fx = width / (2.0 * np.tan(fov_rad / 2.0))
    fy = fx
    return fx, fy, width / 2.0, height / 2.0


def camera_pose_world(
    drone_pos_world: np.ndarray,
    drone_quat_xyzw: tuple[float, float, float, float],
    mount_rpy_deg: tuple[float, float, float] = (0.0, 90.0, 0.0),
    mount_offset_body: tuple[float, float, float] = (0.0, 0.0, -0.5),
) -> tuple[np.ndarray, np.ndarray]:
    """Returns (camera position in the world, R_world_camera) --
    R_world_camera maps a vector in optical camera axes to world axes."""
    R_world_body = rotation_from_quat(*drone_quat_xyzw)
    R_body_socket = rotation_xyz_deg(*mount_rpy_deg)
    R_world_camera = R_world_body @ R_body_socket @ BODY_TO_OPTICAL
    cam_pos_world = drone_pos_world + R_world_body @ np.array(mount_offset_body)
    return cam_pos_world, R_world_camera


def pixel_to_world(
    u: float, v: float,
    cam_pos_world: np.ndarray, R_world_camera: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    z_water: float = 0.0,
) -> Optional[np.ndarray]:
    """Pixel (u,v) -> world point (x,y,z_water), ray-plane intersection.
    None if the ray does not point towards the plane (should not happen in
    a normal nadir/oblique frame)."""
    ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    ray_world = R_world_camera @ ray_cam
    if abs(ray_world[2]) < 1e-9:
        return None
    t = (z_water - cam_pos_world[2]) / ray_world[2]
    if t <= 0:
        return None
    return cam_pos_world + t * ray_world


def world_to_pixel(
    point_world: np.ndarray,
    cam_pos_world: np.ndarray, R_world_camera: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
) -> Optional[tuple[float, float]]:
    """Inverse of pixel_to_world: world point -> pixel (u,v). None if the
    point is behind the camera."""
    v_cam = R_world_camera.T @ (np.asarray(point_world) - cam_pos_world)
    if v_cam[2] <= 0:
        return None
    u = fx * v_cam[0] / v_cam[2] + cx
    v = fy * v_cam[1] / v_cam[2] + cy
    return u, v


def image_footprint(
    width: int, height: int,
    cam_pos_world: np.ndarray, R_world_camera: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    z_water: float = 0.0,
) -> list[Optional[np.ndarray]]:
    """Frame corners [TL, TR, BR, BL] projected onto the water plane."""
    corners_px = [(0, 0), (width, 0), (width, height), (0, height)]
    return [pixel_to_world(u, v, cam_pos_world, R_world_camera, fx, fy, cx, cy, z_water)
            for u, v in corners_px]
