"""Parametrizable trajectory generator (line, square, circle, figure eight,
waypoints).

Pure Python module (no ROS dependencies), so it can be tested in isolation.
All shape-builder positions are in a LOCAL frame with origin at (0,0); the
caller translates them to the vehicle's real origin (first ground truth
pose) via TrajectoryPlan.
"""

import csv as csv_module
import math
from dataclasses import dataclass
from enum import Enum
from typing import List


class SegmentKind(Enum):
    DIVE = 'dive'       # depth transition + yaw alignment, fixed x,y
    TURN = 'turn'       # in-place rotation (square corners / waypoint hold)
    LINE = 'line'       # straight segment at constant body speed
    ARC = 'arc'         # constant radius/speed arc
    TRANSIT = 'transit'  # 3D straight line + yaw slew between two arbitrary poses (waypoints)


@dataclass
class Segment:
    kind: SegmentKind
    duration: float
    params: dict


@dataclass
class TrajectorySample:
    x: float
    y: float
    z: float
    yaw_deg: float
    vx_body: float
    vy_body: float
    wz: float
    finished: bool = False


def _wrap_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


# --------------------------------------------------------------------------
# Segment constructors
# --------------------------------------------------------------------------

def make_line_segment(x0: float, y0: float, z: float, heading_deg: float,
                       length: float, speed: float) -> Segment:
    if speed <= 0.0:
        raise ValueError(
            "speed must be > 0 (to stay still use line_length=0.0 "
            "with any speed > 0, not speed=0.0)")
    duration = length / speed
    return Segment(SegmentKind.LINE, duration,
                    {'x0': x0, 'y0': y0, 'z': z, 'heading_deg': heading_deg, 'speed': speed})


def make_arc_segment(cx: float, cy: float, z: float, radius: float,
                      start_angle_rad: float, sweep_rad: float, speed: float) -> Segment:
    duration = abs(sweep_rad) * radius / speed
    angular_speed = sweep_rad / duration if duration > 0.0 else 0.0
    return Segment(SegmentKind.ARC, duration,
                    {'cx': cx, 'cy': cy, 'z': z, 'radius': radius,
                     'start_angle': start_angle_rad, 'angular_speed': angular_speed, 'speed': speed})


def make_turn_segment(x: float, y: float, z: float, yaw_start_deg: float,
                       yaw_target_deg: float, duration: float) -> Segment:
    delta_deg = _wrap_deg(yaw_target_deg - yaw_start_deg)
    return Segment(SegmentKind.TURN, duration,
                    {'x': x, 'y': y, 'z': z, 'yaw_start_deg': yaw_start_deg, 'delta_deg': delta_deg})


def make_dive_segment(x: float, y: float, z_start: float, z_target: float,
                       yaw_start_deg: float, yaw_target_deg: float, duration: float) -> Segment:
    delta_deg = _wrap_deg(yaw_target_deg - yaw_start_deg)
    return Segment(SegmentKind.DIVE, duration,
                    {'x': x, 'y': y, 'z_start': z_start, 'z_target': z_target,
                     'yaw_start_deg': yaw_start_deg, 'delta_deg': delta_deg})


def make_transit_segment(x0: float, y0: float, z0: float, yaw0_deg: float,
                          x1: float, y1: float, z1: float, yaw1_deg: float,
                          speed: float) -> Segment:
    """3D straight line (x,y,z linearly interpolated) + yaw slew
    (interpolated separately, independent of the direction of travel --
    valid because 'cmd_pos_yaw' controls position and yaw decoupled)
    between two arbitrary poses. 'duration' is derived from the actual 3D
    distance / speed, so a pure depth change (x0,y0 == x1,y1) is also
    interpolated smoothly, not only horizontal displacement."""
    if speed <= 0.0:
        raise ValueError(
            "speed must be > 0 in a transit between waypoints (for a "
            "waypoint with no displacement use hold_time_s, not speed=0.0)")
    dist = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
    duration = dist / speed
    delta_yaw_deg = _wrap_deg(yaw1_deg - yaw0_deg)
    return Segment(SegmentKind.TRANSIT, duration,
                    {'x0': x0, 'y0': y0, 'z0': z0, 'yaw0_deg': yaw0_deg,
                     'x1': x1, 'y1': y1, 'z1': z1, 'delta_yaw_deg': delta_yaw_deg})


# --------------------------------------------------------------------------
# Sampling a segment in local time [0, segment.duration]
# --------------------------------------------------------------------------

def sample_segment(segment: Segment, t_local: float) -> TrajectorySample:
    p = segment.params

    if segment.kind is SegmentKind.LINE:
        heading_rad = math.radians(p['heading_deg'])
        dist = p['speed'] * t_local
        x = p['x0'] + dist * math.cos(heading_rad)
        y = p['y0'] + dist * math.sin(heading_rad)
        return TrajectorySample(x, y, p['z'], p['heading_deg'], p['speed'], 0.0, 0.0)

    if segment.kind is SegmentKind.ARC:
        angular_speed = p['angular_speed']
        angle = p['start_angle'] + angular_speed * t_local
        x = p['cx'] + p['radius'] * math.cos(angle)
        y = p['cy'] + p['radius'] * math.sin(angle)
        turn_sign = math.copysign(1.0, angular_speed) if angular_speed != 0.0 else 1.0
        heading_rad = angle + turn_sign * (math.pi / 2.0)
        return TrajectorySample(x, y, p['z'], math.degrees(heading_rad), p['speed'], 0.0, angular_speed)

    if segment.kind is SegmentKind.TURN:
        frac = (t_local / segment.duration) if segment.duration > 0.0 else 1.0
        yaw_deg = p['yaw_start_deg'] + p['delta_deg'] * frac
        wz = math.radians(p['delta_deg']) / segment.duration if segment.duration > 0.0 else 0.0
        return TrajectorySample(p['x'], p['y'], p['z'], yaw_deg, 0.0, 0.0, wz)

    if segment.kind is SegmentKind.DIVE:
        frac = (t_local / segment.duration) if segment.duration > 0.0 else 1.0
        z = p['z_start'] + (p['z_target'] - p['z_start']) * frac
        yaw_deg = p['yaw_start_deg'] + p['delta_deg'] * frac
        wz = math.radians(p['delta_deg']) / segment.duration if segment.duration > 0.0 else 0.0
        return TrajectorySample(p['x'], p['y'], z, yaw_deg, 0.0, 0.0, wz)

    if segment.kind is SegmentKind.TRANSIT:
        frac = (t_local / segment.duration) if segment.duration > 0.0 else 1.0
        x = p['x0'] + (p['x1'] - p['x0']) * frac
        y = p['y0'] + (p['y1'] - p['y0']) * frac
        z = p['z0'] + (p['z1'] - p['z0']) * frac
        yaw_deg = p['yaw0_deg'] + p['delta_yaw_deg'] * frac

        # vx_body/vy_body: the WORLD velocity (straight line towards the
        # next waypoint) is rotated into the body frame using the COMMANDED
        # yaw at this instant (the generator does not know the real
        # orientation -- same limitation already present in the original
        # DIVE/_build_plan, which assumes a hardcoded yaw_start_deg=0.0).
        # Formula checked against LINE: if yaw_deg matches the direction of
        # travel (as LINE does), it gives vx_body=speed, vy_body=0.
        if segment.duration > 0.0:
            vx_world = (p['x1'] - p['x0']) / segment.duration
            vy_world = (p['y1'] - p['y0']) / segment.duration
            wz = math.radians(p['delta_yaw_deg']) / segment.duration
        else:
            vx_world = vy_world = wz = 0.0
        yaw_rad = math.radians(yaw_deg)
        vx_body = vx_world * math.cos(yaw_rad) + vy_world * math.sin(yaw_rad)
        vy_body = -vx_world * math.sin(yaw_rad) + vy_world * math.cos(yaw_rad)
        return TrajectorySample(x, y, z, yaw_deg, vx_body, vy_body, wz)

    raise ValueError(f"Unknown segment type: {segment.kind}")


# --------------------------------------------------------------------------
# Shape builders (local frame, origin (0,0), DIVE segment not included)
# --------------------------------------------------------------------------

def build_line_shape(length: float, speed: float, z: float) -> List[Segment]:
    return [make_line_segment(0.0, 0.0, z, 0.0, length, speed)]


def build_square_shape(side: float, speed: float, z: float, corner_wait_time: float) -> List[Segment]:
    headings = [0.0, 90.0, 180.0, 270.0]
    segments: List[Segment] = []
    x, y = 0.0, 0.0
    for i, heading in enumerate(headings):
        segments.append(make_line_segment(x, y, z, heading, side, speed))
        heading_rad = math.radians(heading)
        x += side * math.cos(heading_rad)
        y += side * math.sin(heading_rad)
        next_heading = headings[(i + 1) % len(headings)]
        segments.append(make_turn_segment(x, y, z, heading, next_heading, corner_wait_time))
    return segments


def build_hold_shape(x: float, y: float, z: float, yaw_deg: float, duration: float) -> List[Segment]:
    """Single segment: stay still at (x,y,z,yaw_deg) for 'duration' seconds
    (TURN with delta=0 = no rotation nor displacement). Meant to be used with
    loop_trajectory=true (see planner.py::build_plan, trajectory_type='hold')
    -- it then repeats forever and what really decides how long the
    collection lasts is --max-duration/--max-ticks (or Ctrl-C), not
    'duration' itself."""
    return [make_turn_segment(x, y, z, yaw_deg, yaw_deg, duration)]


def build_circle_shape(radius: float, speed: float, z: float, num_loops: int) -> List[Segment]:
    # Center at (0, radius): the vehicle starts at (0,0) facing +x (0 degrees),
    # going around the circle counter-clockwise (ccw).
    cx, cy = 0.0, radius
    start_angle = -math.pi / 2.0
    sweep = 2.0 * math.pi * num_loops
    return [make_arc_segment(cx, cy, z, radius, start_angle, sweep, speed)]


def build_figure_eight_shape(radius: float, speed: float, z: float, num_loops: int,
                              transition_time: float = 2.0) -> List[Segment]:
    # Two lobes tangent at the origin (0,0), with heading 0 degrees at each
    # pass through the crossing: upper lobe counter-clockwise (ccw), lower
    # lobe clockwise (cw). Constant curvature and speed in each lobe ->
    # stable, repeatable "8"/infinity-shaped trajectory.
    #
    # A short pause is inserted between lobes (0-degree TURN = zero
    # commanded velocity, unchanged position/heading) to give the real
    # controller time to settle before the curvature flips -- without it the
    # angular velocity jumps from +speed/radius to -speed/radius from one
    # tick to the next, and the controller (P, cmd_pos_yaw) cannot follow,
    # producing overshoot/drift right at the crossing (same effect seen at
    # the square's corners).
    segments: List[Segment] = []
    for loop_idx in range(num_loops):
        segments.append(make_arc_segment(0.0, radius, z, radius, -math.pi / 2.0, 2.0 * math.pi, speed))
        if transition_time > 0.0:
            segments.append(make_turn_segment(0.0, 0.0, z, 0.0, 0.0, transition_time))
        segments.append(make_arc_segment(0.0, -radius, z, radius, math.pi / 2.0, -2.0 * math.pi, speed))
        if transition_time > 0.0 and loop_idx < num_loops - 1:
            segments.append(make_turn_segment(0.0, 0.0, z, 0.0, 0.0, transition_time))
    return segments


# --------------------------------------------------------------------------
# Waypoints: loading from CSV + segment construction.
# --------------------------------------------------------------------------

WAYPOINTS_CSV_REQUIRED_COLUMNS = ('x', 'y', 'z', 'yaw_deg', 'speed_mps', 'hold_time_s')


@dataclass
class Waypoint:
    x: float           # absolute world position (m), same frame as ground truth
    y: float
    z: float            # absolute depth/altitude (m), same sign as 'location:'/target_depth
    yaw_deg: float       # absolute orientation (degrees), wrapped to [-180, 180]
    speed_mps: float     # cruise speed of the leg that ENDS at this waypoint
    hold_time_s: float   # pause when reaching this waypoint, before continuing


def load_waypoints_csv(path: str) -> List[Waypoint]:
    """Loads a list of Waypoint from a CSV with columns
    x,y,z,yaw_deg,speed_mps,hold_time_s (the 'waypoint_id' column, if
    present, is informative only -- the actual visiting order is the order
    of the rows in the file, not that value)."""
    with open(path, newline='') as f:
        reader = csv_module.DictReader(f)
        fieldnames = set(reader.fieldnames or [])
        missing = [c for c in WAYPOINTS_CSV_REQUIRED_COLUMNS if c not in fieldnames]
        if missing:
            raise ValueError(
                f"Waypoints CSV '{path}' is missing required columns: {missing}")

        waypoints: List[Waypoint] = []
        for row_num, row in enumerate(reader, start=2):
            try:
                waypoints.append(Waypoint(
                    x=float(row['x']), y=float(row['y']), z=float(row['z']),
                    yaw_deg=float(row['yaw_deg']),
                    speed_mps=float(row['speed_mps']),
                    hold_time_s=float(row['hold_time_s'])))
            except (TypeError, ValueError) as e:
                raise ValueError(
                    f"Waypoints CSV '{path}', row {row_num}: invalid value ({e})") from e

    if not waypoints:
        raise ValueError(f"Waypoints CSV '{path}' has no data rows")
    for i, wp in enumerate(waypoints):
        if wp.speed_mps <= 0.0:
            raise ValueError(
                f"Waypoints CSV '{path}', waypoint #{i + 1}: speed_mps must be > 0 "
                f"(value: {wp.speed_mps})")
        if wp.hold_time_s < 0.0:
            raise ValueError(
                f"Waypoints CSV '{path}', waypoint #{i + 1}: hold_time_s cannot be "
                f"negative (value: {wp.hold_time_s})")
    return waypoints


def build_waypoints_shape(waypoints: List[Waypoint]) -> List[Segment]:
    """Builds TRANSIT segments (towards each waypoint) interleaved with TURN
    (in-place pause if hold_time_s > 0) for each waypoint, in the given
    order. Does NOT include the approach leg from the vehicle's real spawn
    position to the first waypoint -- planner.py::build_plan adds that, the
    same way build_line_shape/build_square_shape/etc do not include the
    initial DIVE."""
    if not waypoints:
        raise ValueError("At least one waypoint is needed")

    segments: List[Segment] = []
    first = waypoints[0]
    if first.hold_time_s > 0.0:
        segments.append(make_turn_segment(
            first.x, first.y, first.z, first.yaw_deg, first.yaw_deg, first.hold_time_s))

    for prev, nxt in zip(waypoints, waypoints[1:]):
        segments.append(make_transit_segment(
            prev.x, prev.y, prev.z, prev.yaw_deg,
            nxt.x, nxt.y, nxt.z, nxt.yaw_deg, nxt.speed_mps))
        if nxt.hold_time_s > 0.0:
            segments.append(make_turn_segment(
                nxt.x, nxt.y, nxt.z, nxt.yaw_deg, nxt.yaw_deg, nxt.hold_time_s))

    return segments


class TrajectoryPlan:
    def __init__(self, segments: List[Segment], origin_x: float, origin_y: float):
        self.segments = segments
        self.origin_x = origin_x
        self.origin_y = origin_y

    def total_duration(self) -> float:
        return sum(segment.duration for segment in self.segments)

    def sample(self, t: float, loop: bool = False) -> TrajectorySample:
        total = self.total_duration()
        finished = False

        if total <= 0.0:
            t_query = 0.0
        elif loop:
            t_query = t % total
        elif t >= total:
            t_query = total
            finished = True
        else:
            t_query = t

        elapsed = 0.0
        chosen_segment = self.segments[-1]
        t_local = chosen_segment.duration
        for segment in self.segments:
            if t_query <= elapsed + segment.duration or segment is self.segments[-1]:
                chosen_segment = segment
                t_local = min(max(0.0, t_query - elapsed), segment.duration)
                break
            elapsed += segment.duration

        sample = sample_segment(chosen_segment, t_local)
        sample.x += self.origin_x
        sample.y += self.origin_y
        sample.finished = finished
        return sample
