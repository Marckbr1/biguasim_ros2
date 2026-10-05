"""Detection of ACTUAL arrival at a waypoint -- by position, not by time.

`TrajectoryPlan` (trajectories/trajectories.py, ported verbatim from the
ROS 2 project) has no arrival detection: the commanded target advances
purely by elapsed time, without checking whether the real vehicle is
already there (a real run confirmed the vehicle can get stuck while the
generator keeps "visiting" the remaining waypoints in mid-air). This module
does NOT change that control logic -- it only watches the actual position
(ground truth, `DynamicsSensor`) tick by tick and confirms a waypoint when
the vehicle really enters its tolerance radius, so an honest "reached"
message can be logged instead of one that only reflects the commanded plan.

BUG FOUND AND FIXED (user report -- "it doesn't reach point 5 or 6",
suspected collision): the first version of this watcher confirmed
waypoints in strict order, checking only the "next pending" one. With very
close waypoints and `hold_time_s=0` (the plan never waits for the vehicle
to actually converge before moving on), the minimum actual distance to ONE
intermediate waypoint can stay above the tolerance (the vehicle "cuts the
corner") -- verified empirically: with a CSV of 133 very close waypoints,
the minimum actual distance to waypoint 4 was 1.27m with tolerance=0.5m, so
it was never confirmed. Since the original watcher only looked at the "next
pending" one, it waited for waypoint 4 FOREVER and never checked 5 (which
was reached, at 0.187m) or any later one -- it was not a real collision
(ground truth showed the vehicle moving non-stop, with no frozen-position
stretch). Fix: each waypoint is confirmed INDEPENDENTLY, as soon as the
vehicle enters its radius, without requiring the previous ones to be
confirmed first.
"""

from __future__ import annotations

import logging
import math
from typing import List, Optional

from .trajectories import Waypoint, load_waypoints_csv

logger = logging.getLogger(__name__)

DEFAULT_ARRIVAL_TOLERANCE_M = 1.0

# Index of px/py/pz inside the array returned by DynamicsSensor. Invariant
# with respect to 'UseRPY' (config/sensor_fields.py::TABULAR_FIELD_NAMES): both
# variants (quaternion or roll/pitch/yaw) share
# [ax,ay,az, vx,vy,vz, px,py,pz, ...] and only differ after the position.
_POSITION_SLICE = slice(6, 9)


class WaypointArrivalWatcher:
    """Confirms each waypoint, INDEPENDENTLY, the FIRST time the vehicle's
    actual position gets within `tolerance_m` of its target -- it does not
    require the previous waypoints to be confirmed (an intermediate waypoint
    "cut" by the controller does not block detection of the following ones),
    and never fires the same waypoint twice."""

    def __init__(self, waypoints: List[Waypoint], tolerance_m: float = DEFAULT_ARRIVAL_TOLERANCE_M):
        if not waypoints:
            raise ValueError("At least one waypoint is needed")
        if tolerance_m <= 0.0:
            raise ValueError(f"tolerance_m must be > 0 (value: {tolerance_m})")
        self.waypoints = waypoints
        self.tolerance_m = tolerance_m
        self.reached_at_t: List[Optional[float]] = [None] * len(waypoints)
        self._pending_indices: List[int] = list(range(len(waypoints)))

    @property
    def num_reached(self) -> int:
        return len(self.waypoints) - len(self._pending_indices)

    @property
    def all_reached(self) -> bool:
        return not self._pending_indices

    def is_reached(self, waypoint_number: int) -> bool:
        """`waypoint_number` is 1-indexed, same as in the log messages and in
        --stop-at-waypoint."""
        if not (1 <= waypoint_number <= len(self.waypoints)):
            raise IndexError(
                f"waypoint_number={waypoint_number} out of range "
                f"(1..{len(self.waypoints)})")
        return self.reached_at_t[waypoint_number - 1] is not None

    def update(self, px: float, py: float, pz: float, t: float) -> List[int]:
        """Call once per tick with the actual GT position. Checks ALL the
        waypoints not yet confirmed (not just the next one in order) and
        confirms any that is within tolerance at this tick. Returns the
        (possibly empty) list of (0-based) indices newly confirmed at this
        tick."""
        if not self._pending_indices:
            return []

        newly_reached = []
        still_pending = []
        for idx in self._pending_indices:
            wp = self.waypoints[idx]
            dist = math.sqrt((px - wp.x) ** 2 + (py - wp.y) ** 2 + (pz - wp.z) ** 2)
            if dist <= self.tolerance_m:
                self.reached_at_t[idx] = t
                logger.info(
                    "✓ Waypoint %d/%d REACHED -- target=(%.3f, %.3f, %.3f) "
                    "actual=(%.3f, %.3f, %.3f) distance=%.3fm (tolerance=%.2fm) t=%.2fs",
                    idx + 1, len(self.waypoints), wp.x, wp.y, wp.z, px, py, pz,
                    dist, self.tolerance_m, t,
                )
                newly_reached.append(idx)
            else:
                still_pending.append(idx)
        self._pending_indices = still_pending
        return newly_reached

    def update_from_dynamics_sensor(self, dynamics_sensor_reading, t: float) -> List[int]:
        """Shortcut: takes the raw array returned by state['DynamicsSensor']
        (see _POSITION_SLICE above) instead of separate px/py/pz."""
        px, py, pz = (float(v) for v in dynamics_sensor_reading[_POSITION_SLICE])
        return self.update(px, py, pz, t)

    def log_summary(self) -> None:
        total = len(self.waypoints)
        reached = self.num_reached
        if reached == total:
            logger.info("Waypoints reached (actual, by position): %d/%d -- mission complete.", reached, total)
        else:
            missing = [i + 1 for i, t in enumerate(self.reached_at_t) if t is None]
            shown = missing[:20]
            suffix = f" ... (+{len(missing) - 20} more)" if len(missing) > 20 else ""
            logger.warning(
                "Waypoints reached (actual, by position): %d/%d -- never confirmed within "
                "%.2fm: %s%s (the vehicle may have cut the corner at those points, got "
                "stuck, or the run ended before arriving).",
                reached, total, self.tolerance_m, shown, suffix,
            )

    @classmethod
    def from_csv(cls, csv_path: str, tolerance_m: float = DEFAULT_ARRIVAL_TOLERANCE_M) -> "WaypointArrivalWatcher":
        return cls(load_waypoints_csv(csv_path), tolerance_m=tolerance_m)
