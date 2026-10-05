"""Waypoint advance CLOSED ON THE ACTUAL POSITION, instead of purely by clock.

`TrajectoryPlan.sample()` (trajectories.py, ported verbatim from the ROS 2
project) is a pure function: given an elapsed time, it returns where the
vehicle "should" be according to the plan, without ever knowing where it
really is -- that is why a waypoint with `hold_time_s=0` is "abandoned" as
soon as its nominal time passes, whether or not the real vehicle has
arrived (with `speed_mps=1.0` the real gap was a systematic ~1-3m).

This module does NOT replace or modify `trajectories.py` -- `line`,
`square`, `circle`, `figure_eight` and the classic `waypoints` mode
(`closed_loop_waypoints: false`, the default) behave exactly as before.
`ClosedLoopWaypointPlan` is a new class, opt-in only for
`trajectory_type == 'waypoints'`, that reuses the same segment math
(`make_transit_segment`, `make_turn_segment`, `sample_segment` -- zero
re-implementation of the interpolation) but decides WHEN to move to the
next leg by looking at the vehicle's actual position at every tick,
instead of a fixed clock:

  - While the real vehicle has not entered `tolerance_m` of the current
    waypoint, the commanded point stays at the leg's destination (it does
    not overshoot it) -- the controller gets unlimited time (up to the
    timeout) to converge, instead of the tracked target running away.
  - If the vehicle never arrives (real collision, world geometry), a safety
    timeout (`timeout_factor` x the leg's nominal duration) makes it
    advance anyway, so the run never hangs forever.

Since `sample()` needs the actual position at every tick (not just the
time), it does not share the `TrajectoryPlan.sample(t, loop)` signature --
`runner.py` calls it through a separate branch when `closed_loop_waypoints:
true` is set in the trajectory YAML.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .trajectories import Segment, TrajectorySample, Waypoint, make_transit_segment, make_turn_segment, sample_segment

logger = logging.getLogger(__name__)

# How long to wait, as a multiple of the leg's nominal duration (distance/
# speed_mps), before giving up and advancing anyway. 3x is generous (gives
# the controller three times the "ideal" time to converge) without risking
# hanging the run forever on a real collision.
DEFAULT_TIMEOUT_FACTOR = 3.0


@dataclass
class _Leg:
    transit: Segment
    target: Tuple[float, float, float]
    yaw_deg: float
    waypoint_number: int  # 1-indexed, matches the waypoint_watcher.py messages
    hold: Optional[Segment]  # in-place waiting TURN (hold_time_s > 0), or None


class ClosedLoopWaypointPlan:
    def __init__(
        self,
        waypoints: List[Waypoint],
        origin_x: float,
        origin_y: float,
        origin_z: float,
        tolerance_m: float,
        timeout_factor: float = DEFAULT_TIMEOUT_FACTOR,
    ):
        if not waypoints:
            raise ValueError("At least one waypoint is needed")
        if tolerance_m <= 0.0:
            raise ValueError(f"tolerance_m must be > 0 (value: {tolerance_m})")
        if timeout_factor <= 0.0:
            raise ValueError(f"timeout_factor must be > 0 (value: {timeout_factor})")

        self.tolerance_m = tolerance_m
        self.timeout_factor = timeout_factor

        self.legs: List[_Leg] = []
        prev_x, prev_y, prev_z = origin_x, origin_y, origin_z
        # The approach leg (origin -> first waypoint) starts with a
        # hardcoded yaw=0.0, the same way planner.py::build_plan does it for
        # the classic 'waypoints' mode -- not a new choice of this module,
        # just the existing convention (see planner.py).
        prev_yaw = 0.0
        for i, wp in enumerate(waypoints):
            transit = make_transit_segment(
                prev_x, prev_y, prev_z, prev_yaw,
                wp.x, wp.y, wp.z, wp.yaw_deg, wp.speed_mps,
            )
            hold = None
            if wp.hold_time_s > 0.0:
                hold = make_turn_segment(wp.x, wp.y, wp.z, wp.yaw_deg, wp.yaw_deg, wp.hold_time_s)
            self.legs.append(_Leg(
                transit=transit, target=(wp.x, wp.y, wp.z), yaw_deg=wp.yaw_deg,
                waypoint_number=i + 1, hold=hold,
            ))
            prev_x, prev_y, prev_z, prev_yaw = wp.x, wp.y, wp.z, wp.yaw_deg

        self._final_target = self.legs[-1].target
        self._final_yaw = self.legs[-1].yaw_deg

        self.arrived_by_tolerance: List[bool] = [False] * len(waypoints)
        self.arrived_at_t: List[Optional[float]] = [None] * len(waypoints)

        self._leg_idx = 0
        self._phase = "transit"  # "transit" | "hold"
        self._phase_start_t: Optional[float] = None
        self.finished = False

    def total_duration_nominal(self) -> float:
        """Sum of NOMINAL durations (distance/speed_mps + hold_time_s),
        informative only -- the real duration depends on how long the
        vehicle actually takes to reach each waypoint, it is not fixed."""
        total = 0.0
        for leg in self.legs:
            total += leg.transit.duration
            if leg.hold is not None:
                total += leg.hold.duration
        return total

    def is_reached(self, waypoint_number: int) -> bool:
        """1-indexed. True only if confirmed by actual TOLERANCE -- if the
        leg was abandoned by timeout (the vehicle never arrived), False."""
        if not (1 <= waypoint_number <= len(self.legs)):
            raise IndexError(f"waypoint_number={waypoint_number} out of range (1..{len(self.legs)})")
        return self.arrived_by_tolerance[waypoint_number - 1]

    def sample(self, t: float, real_pos: Tuple[float, float, float], loop: bool = False) -> TrajectorySample:
        if self._phase_start_t is None:
            self._phase_start_t = t

        if self.finished:
            if not loop:
                return TrajectorySample(*self._final_target, self._final_yaw, 0.0, 0.0, 0.0, finished=True)
            self._leg_idx = 0
            self._phase = "transit"
            self._phase_start_t = t
            self.finished = False

        leg = self.legs[self._leg_idx]
        t_local = t - self._phase_start_t

        if self._phase == "transit":
            clamped = min(t_local, leg.transit.duration)
            sample = sample_segment(leg.transit, clamped)

            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(real_pos, leg.target)))
            arrived = dist <= self.tolerance_m
            timed_out = t_local >= leg.transit.duration * self.timeout_factor

            if arrived or timed_out:
                idx = leg.waypoint_number - 1
                self.arrived_by_tolerance[idx] = arrived
                self.arrived_at_t[idx] = t
                if arrived:
                    logger.info(
                        "[closed-loop] ✓ Waypoint %d/%d REACHED -- distance=%.3fm "
                        "(tolerance=%.2fm) t=%.2fs, moving to the next leg.",
                        leg.waypoint_number, len(self.legs), dist, self.tolerance_m, t,
                    )
                else:
                    logger.warning(
                        "[closed-loop] Waypoint %d/%d NEVER confirmed within %.2fm "
                        "(final distance=%.3fm) -- %.1fs timeout expired (%.1fx the "
                        "nominal duration of %.1fs), advancing anyway. t=%.2fs",
                        leg.waypoint_number, len(self.legs), self.tolerance_m, dist,
                        leg.transit.duration * self.timeout_factor, self.timeout_factor,
                        leg.transit.duration, t,
                    )
                self._advance(t)
            return sample

        else:  # "hold"
            clamped = min(t_local, leg.hold.duration)
            sample = sample_segment(leg.hold, clamped)
            if t_local >= leg.hold.duration:
                self._advance_leg(t)
            return sample

    def _advance(self, t: float) -> None:
        leg = self.legs[self._leg_idx]
        if leg.hold is not None:
            self._phase = "hold"
            self._phase_start_t = t
        else:
            self._advance_leg(t)

    def _advance_leg(self, t: float) -> None:
        if self._leg_idx + 1 >= len(self.legs):
            self.finished = True
        else:
            self._leg_idx += 1
            self._phase = "transit"
            self._phase_start_t = t

    def log_summary(self) -> None:
        total = len(self.legs)
        reached = sum(self.arrived_by_tolerance)
        if reached == total:
            logger.info("[closed-loop] Waypoints reached (by actual tolerance): %d/%d -- mission complete.",
                         reached, total)
        else:
            missing = [i + 1 for i, ok in enumerate(self.arrived_by_tolerance) if not ok]
            shown = missing[:20]
            suffix = f" ... (+{len(missing) - 20} more)" if len(missing) > 20 else ""
            logger.warning(
                "[closed-loop] Waypoints reached (by actual tolerance): %d/%d -- advanced by "
                "timeout (never confirmed within %.2fm) at: %s%s",
                reached, total, self.tolerance_m, shown, suffix,
            )
