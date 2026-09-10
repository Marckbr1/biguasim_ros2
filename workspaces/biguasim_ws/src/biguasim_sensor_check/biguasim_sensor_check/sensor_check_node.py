#!/usr/bin/env python3
"""Verify BiguaSim's sensor topics are flowing over ROS 2.

Subscribes to a configurable list of expected sensor topics, measures the
arrival rate of each, and prints a status table every `report_period`
seconds:

    topic                                            type                 msgs   rate    last    status
    /biguasim/rov0_id0/DynamicsSensor/Odom           nav_msgs/Odometry    1240   30.1Hz  0.03s   OK
    /biguasim/rov0_id0/ImagingSonar                  .../ImagingSonar      207   5.0Hz   0.19s   OK
    /biguasim/rov0_id0/IMUSensor                     sensor_msgs/Imu         0   --      --      SILENT
    ...
    -> 6/7 sensors OK

Use it to confirm the BiguaSim <-> ROS 2 connection: run BiguaSim, then run
this node (or `ros2 launch biguasim_sensor_check sensor_check.launch.py`).

Parameters
----------
sensors : string[]
    One entry per topic, "topic|type|expected_hz", e.g.
    "/biguasim/rov0_id0/IMUSensor|sensor_msgs/msg/Imu|30". `type` accepts
    "pkg/msg/Type" or "pkg/Type". `expected_hz` may be 0 (rate not checked).
report_period : float   seconds between status tables (default 2.0)
window : float          sliding window for the rate estimate, seconds (default 5.0)
stale_after : float     no message for this long -> STALE (default 2.0)
exit_after : float      if >0, print a final table after N s and shut down
                        (handy for scripts / CI). 0 = run forever.
"""

from __future__ import annotations

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message


DEFAULT_SENSORS = [
    "/biguasim/rov0_id0/DynamicsSensor/Odom|nav_msgs/msg/Odometry|30",
    "/biguasim/rov0_id0/ImagingSonar|biguasim_interfaces/msg/ImagingSonar|5",
    "/biguasim/rov0_id0/IMUSensor|sensor_msgs/msg/Imu|30",
    "/biguasim/rov0_id0/DVLSensor/Velocity|geometry_msgs/msg/TwistWithCovarianceStamped|5",
    "/biguasim/rov0_id0/DVLSensor/Range|biguasim_interfaces/msg/DVLSensorRange|5",
    "/biguasim/rov0_id0/DepthSensor|geometry_msgs/msg/PoseWithCovarianceStamped|30",
    "/biguasim/rov0_id0/MagnetometerSensor|sensor_msgs/msg/MagneticField|30",
]


class _Track:
    __slots__ = ("topic", "type_str", "expected_hz", "count", "last_wall", "arrivals", "error")

    def __init__(self, topic: str, type_str: str, expected_hz: float) -> None:
        self.topic = topic
        self.type_str = type_str
        self.expected_hz = expected_hz
        self.count = 0
        self.last_wall: float | None = None
        self.arrivals: deque[float] = deque()
        self.error: str | None = None


class SensorCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("biguasim_sensor_check")

        self.declare_parameter("sensors", DEFAULT_SENSORS)
        self.declare_parameter("report_period", 2.0)
        self.declare_parameter("window", 5.0)
        self.declare_parameter("stale_after", 2.0)
        self.declare_parameter("exit_after", 0.0)

        self.window = float(self.get_parameter("window").value)
        self.stale_after = float(self.get_parameter("stale_after").value)
        self.exit_after = float(self.get_parameter("exit_after").value)

        # Best-effort + volatile is the safest default for sensor streams; a
        # reliable/transient publisher is still matched by a best-effort sub.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tracks: list[_Track] = []
        for entry in self.get_parameter("sensors").value:
            parts = [p.strip() for p in str(entry).split("|")]
            topic = parts[0]
            type_str = parts[1] if len(parts) > 1 and parts[1] else "std_msgs/msg/Empty"
            expected_hz = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
            tr = _Track(topic, type_str, expected_hz)
            # accept "pkg/Type" and "pkg/msg/Type"
            norm = type_str if "/msg/" in type_str else type_str.replace("/", "/msg/", 1)
            tr.type_str = norm
            try:
                msg_cls = get_message(norm)
            except Exception as exc:  # noqa: BLE001
                tr.error = f"type not found ({exc.__class__.__name__})"
                self.tracks.append(tr)
                continue
            self.create_subscription(
                msg_cls, topic, lambda _m, t=tr: self._on_msg(t), qos)
            self.tracks.append(tr)

        self._t0 = time.monotonic()
        self.create_timer(float(self.get_parameter("report_period").value), self._report)
        self.get_logger().info(
            f"watching {len(self.tracks)} sensor topic(s); "
            f"report every {self.get_parameter('report_period').value}s")

    def _on_msg(self, tr: _Track) -> None:
        now = time.monotonic()
        tr.count += 1
        tr.last_wall = now
        tr.arrivals.append(now)
        cutoff = now - self.window
        while tr.arrivals and tr.arrivals[0] < cutoff:
            tr.arrivals.popleft()

    def _rate(self, tr: _Track) -> float:
        if len(tr.arrivals) < 2:
            return 0.0
        span = tr.arrivals[-1] - tr.arrivals[0]
        return (len(tr.arrivals) - 1) / span if span > 0 else 0.0

    def _status(self, tr: _Track, now: float) -> str:
        if tr.error:
            return "ERR"
        if tr.count == 0:
            return "SILENT"
        if tr.last_wall is None or now - tr.last_wall > self.stale_after:
            return "STALE"
        if tr.expected_hz > 0 and self._rate(tr) < 0.5 * tr.expected_hz:
            return "LOW"
        return "OK"

    def _report(self) -> None:
        now = time.monotonic()
        lines = [
            "",
            f"{'topic':<48} {'type':<42} {'msgs':>6} {'rate':>8} {'last':>7} status",
            "-" * 120,
        ]
        n_ok = 0
        for tr in self.tracks:
            st = self._status(tr, now)
            n_ok += st == "OK"
            last = "--" if tr.last_wall is None else f"{now - tr.last_wall:5.2f}s"
            rate = "--" if tr.count == 0 else f"{self._rate(tr):5.1f}Hz"
            note = f"  ({tr.error})" if tr.error else ""
            lines.append(
                f"{tr.topic:<48} {tr.type_str:<42} {tr.count:>6} {rate:>8} {last:>7} {st}{note}")
        lines.append("-" * 120)
        lines.append(f"-> {n_ok}/{len(self.tracks)} sensors OK")
        self.get_logger().info("\n".join(lines))

        if self.exit_after > 0 and now - self._t0 >= self.exit_after:
            self.get_logger().info("exit_after reached -- shutting down")
            raise SystemExit(0 if n_ok == len(self.tracks) else 1)


def main() -> None:
    rclpy.init()
    node = SensorCheckNode()
    code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        code = int(getattr(exc, "code", 0) or 0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
