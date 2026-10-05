#!/usr/bin/env python3
"""Imaging sonar (`ImagingSonar`) geometry: the 2D sector (fan) it covers in
the WORLD, to draw it on map/mapa.png -- equivalent to what
`SonShape`/`AerialImageNode` draw in `marckloc_cnn_ws` (ROS 2,
`src/marckloc_map/src/SonShape.cpp`), but geometric and standalone here (no
ROS 2, no RViz), from the sensor's real `location`/config in the simulator
config and the dataset's own ground truth.

Deliberate simplification: the sonar also has an elevation extent
(`Elevation`, 20 degrees by default), but a plan view (top-down map) cannot
show that dimension without collapsing it -- only the horizontal sector
(azimuth x range) is drawn, like any aerial view of a 2D sonar FOV.

LIBRARY module (no CLI): used by the sonar/dashboard tools of
../colecta_biguasim.
"""

from __future__ import annotations

import numpy as np

from biguasim_dataset_standalone.georeference.core import rotation_from_quat


def heading_rad(quat_xyzw: tuple[float, float, float, float]) -> float:
    """Vehicle heading (radians, atan2 in BiguaSim's x,y plane) from its
    quaternion -- rotates the body +x axis (forward) into the world and
    takes its angle, instead of extracting 'yaw' from the matrix (avoids
    Euler decomposition ambiguities with non-zero roll/pitch)."""
    R = rotation_from_quat(*quat_xyzw)
    fwd_world = R @ np.array([1.0, 0.0, 0.0])
    return float(np.arctan2(fwd_world[1], fwd_world[0]))


def sonar_fov_polygon_world(
    rov_xy: tuple[float, float],
    quat_xyzw: tuple[float, float, float, float],
    range_min: float,
    range_max: float,
    azimuth_deg: float,
    mount_yaw_deg: float = 0.0,
    n_arc_points: int = 24,
) -> np.ndarray:
    """(N,2) polygon of the sonar sector [range_min,range_max] x
    [-azimuth/2,+azimuth/2], in world coordinates (x,y) -- outer arc
    (range_max) followed by the inner arc (range_min) reversed, leaving the
    'hole' of the blind zone near the vehicle (the sonar detects nothing
    closer than range_min). `mount_yaw_deg`: mounting offset of the sonar
    socket relative to the body (0 by default -- ImagingSonar in the
    simulator config declares no 'rotation', i.e. aligned with the body,
    facing forward)."""
    x0, y0 = rov_xy
    center = heading_rad(quat_xyzw) + np.radians(mount_yaw_deg)
    half = np.radians(azimuth_deg) / 2.0
    thetas = np.linspace(center - half, center + half, n_arc_points)

    outer = np.stack([x0 + range_max * np.cos(thetas), y0 + range_max * np.sin(thetas)], axis=1)
    inner = np.stack([x0 + range_min * np.cos(thetas[::-1]), y0 + range_min * np.sin(thetas[::-1])], axis=1)
    return np.concatenate([outer, inner], axis=0)


def plot_sonar_polar(ax, sonar_array: np.ndarray, range_min: float, range_max: float,
                      azimuth_deg: float, cmap: str = "viridis"):
    """Draws `sonar_array` (RangeBins, AzimuthBins) on a polar Axes --
    theta=azimuth, r=actual range in meters via RangeMin/RangeMax."""
    ax.clear()
    if sonar_array.size == 0:
        ax.text(0.5, 0.5, "no data", ha="center", va="center", transform=ax.transAxes)
        return
    theta = np.linspace(-np.radians(azimuth_deg) / 2, np.radians(azimuth_deg) / 2, sonar_array.shape[1])
    r = np.linspace(range_min, range_max, sonar_array.shape[0])
    Theta, R = np.meshgrid(theta, r)
    ax.pcolormesh(Theta, R, sonar_array, cmap=cmap, shading="auto")
    ax.set_thetamin(-azimuth_deg / 2)
    ax.set_thetamax(azimuth_deg / 2)
    ax.set_theta_zero_location("N")
    ax.set_title("Sonar (raw, polar)", fontsize=10)
