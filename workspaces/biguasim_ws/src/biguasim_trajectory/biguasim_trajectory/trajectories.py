"""Generador de trayectorias parametrizable (cuadrado, circulo).

Modulo puro en Python (sin dependencias de ROS), portado y recortado de
biguasim_trajectory en el workspace 'anterior' -- aqui solo se mantienen los
dos shape builders que pide el foco actual (cuadrado, circulo); 'line' y
'figure_eight' se dejaron fuera a proposito. Todas las posiciones de los
shape builders estan en un marco LOCAL con origen en (0,0); el nodo que los
usa las traslada al origen real del vehiculo (primera pose de ground truth)
via TrajectoryPlan.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List


class SegmentKind(Enum):
    DIVE = 'dive'   # transicion de profundidad + alineacion de yaw, x,y fijos
    TURN = 'turn'   # rotacion en el sitio (esquinas del cuadrado)
    LINE = 'line'   # segmento recto a velocidad de cuerpo constante
    ARC = 'arc'     # arco de radio/velocidad constante (circulo)


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
# Constructores de segmento
# --------------------------------------------------------------------------

def make_line_segment(x0: float, y0: float, z: float, heading_deg: float,
                       length: float, speed: float) -> Segment:
    if speed <= 0.0:
        raise ValueError("speed debe ser > 0")
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


# --------------------------------------------------------------------------
# Muestreo de un segmento en tiempo local [0, segment.duration]
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

    raise ValueError(f"Tipo de segmento desconocido: {segment.kind}")


# --------------------------------------------------------------------------
# Shape builders (marco local, origen (0,0), no incluyen el segmento DIVE)
# --------------------------------------------------------------------------

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


def build_circle_shape(radius: float, speed: float, z: float, num_loops: int) -> List[Segment]:
    # Centro en (0, radius): el vehiculo arranca en (0,0) mirando hacia +x (0 grados),
    # recorriendo el circulo en sentido antihorario (ccw).
    cx, cy = 0.0, radius
    start_angle = -math.pi / 2.0
    sweep = 2.0 * math.pi * num_loops
    return [make_arc_segment(cx, cy, z, radius, start_angle, sweep, speed)]


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
