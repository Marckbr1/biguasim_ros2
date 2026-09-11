#!/usr/bin/env python3
"""Waypoint follower for a BiguaSim agent.

Ported from holoocean_examples/waypoint_follower.py. The waypoint-seeking
geometry (distance/bearing to the next XY waypoint, advance/loop logic,
color-coded RViz markers) is a direct port -- none of that depends on
BiguaSim's command semantics.

What is NOT a direct port: HoloOcean publishes a typed AgentCommand whose
meaning WaypointFollower doesn't need to know (HoloOcean's own bridge/Fossen
layer interprets it). BiguaSim instead expects a flat Float64MultiArray
whose per-index meaning depends on the agent's control_abstraction, which
is not documented anywhere available in this workspace (see
biguasim_examples/README.md and biguasim_main/README.md "Known gaps").

So the waypoint-derived control quantities this node computes -- `surge`
(desired forward speed), `yaw_rate` (desired turn rate, positive = turn
toward the waypoint), `depth` (from `target_depth`, a fixed parameter) and
`heading_deg` (absolute bearing to the waypoint, degrees) -- are placed
into the output vector according to a `command_slots` parameter: a list of
tokens, one per output slot, each "surge", "yaw_rate", "depth",
"heading_deg", "zero", or a literal number. The default is all "zero"
(safe no-op).

CONFIRMED exception: for `control_abstraction: 'cmd_depth_heading_rpm_surge'`,
the repo root BIGUASIM.md documents the real order directly (worked example,
`env.step([5, 45, 1000, 1])  # depth, heading, rpm, surge`) --
`command_slots: ['depth', 'heading_deg', '<rpm>', 'surge']` (pick a literal
`<rpm>`, e.g. a cruise RPM constant, since this node has no independent RPM
target). For every other control_abstraction, the index order is still
unverified -- fill in `command_slots` only after confirming it empirically
(see README.md).
"""
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def _slot_value(token: str, surge: float, yaw_rate: float, depth: float, heading_deg: float) -> float:
    named = {
        'surge': surge, '-surge': -surge,
        'yaw_rate': yaw_rate, '-yaw_rate': -yaw_rate,
        'depth': depth, '-depth': -depth,
        'heading_deg': heading_deg, '-heading_deg': -heading_deg,
        'zero': 0.0,
    }
    if token in named:
        return named[token]
    try:
        return float(token)
    except ValueError:
        return 0.0


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('waypoint_threshold', 3.0)
        self.declare_parameter('waypoint_file', 'config/sv_waypoints.yaml')
        self.declare_parameter('relative_path', True)
        self.declare_parameter('agent_name', 'auv0')
        self.declare_parameter('odom_topic', '')  # '' -> '<agent_name>/DynamicsSensor/Odom'
        self.declare_parameter('loop_waypoints', True)
        self.declare_parameter('marker_scale', 0.2)
        self.declare_parameter('marker_frame', 'map')

        self.declare_parameter('max_surge', 1.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('yaw_gain', 1.0)
        # Used only by the 'depth' command_slots token (see module docstring).
        self.declare_parameter('target_depth', 0.0)
        # One token per output vector slot: "surge" | "yaw_rate" | "zero" | a
        # literal number. Default = all-zero (safe no-op) -- see module
        # docstring. Example for a 4-slot control_abstraction where index 3
        # is forward speed and index 1 is yaw:
        #   command_slots: ["zero", "yaw_rate", "zero", "surge"]
        self.declare_parameter('command_slots', ['zero'])

        self.marker_frame = self.get_parameter('marker_frame').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.agent_name = self.get_parameter('agent_name').value
        self.loop_waypoints = self.get_parameter('loop_waypoints').value
        self.marker_scale = self.get_parameter('marker_scale').value
        self.max_surge = float(self.get_parameter('max_surge').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.target_depth = float(self.get_parameter('target_depth').value)
        self.command_slots = list(self.get_parameter('command_slots').value)

        if all(tok == 'zero' for tok in self.command_slots):
            self.get_logger().warn(
                "command_slots is all 'zero' -- this node will publish "
                "no-op commands. Set command_slots to place 'surge'/'yaw_rate' "
                "in the right index for your scenario's control_abstraction "
                "once you've confirmed it (see README.md)."
            )

        self.waypoint_locations = self._load_waypoints(
            self.get_parameter('waypoint_file').value,
            self.get_parameter('relative_path').value)
        if len(self.waypoint_locations) == 0:
            raise RuntimeError('Waypoint file contains no waypoints')
        self.waypoint_idx = 0

        odom_topic = self.get_parameter('odom_topic').value
        if not odom_topic:
            odom_topic = f'{self.agent_name}/DynamicsSensor/Odom'

        self.cmd_pub = self.create_publisher(Float64MultiArray, f'{self.agent_name}/command_control', 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.marker_pub = self.create_publisher(Marker, 'waypoints/markers', 10)

        self.current_position = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self._have_odom = False

        self.sim_clock = self.get_clock()
        self.last_publish_time = self.sim_clock.now()
        self.publish_interval = Duration(seconds=0.5)

        self.timer = self.create_timer(0.1, self._timer_cb)

        self.get_logger().info(
            f'Waypoint follower ready for agent [{self.agent_name}], '
            f'{len(self.waypoint_locations)} waypoints, odom <- {odom_topic}')

    def _load_waypoints(self, waypoint_file, relative_path):
        if relative_path:
            share_dir = get_package_share_directory('biguasim_examples')
            waypoint_file = os.path.join(share_dir, waypoint_file)
        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)
        waypoints = np.array(data['waypoints'])
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {waypoint_file}')
        return waypoints

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_position = np.array([p.x, p.y])
        # yaw from quaternion (Z-up convention), avoids adding a scipy dep here
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        self._have_odom = True

    def _reset_waypoints(self):
        self.waypoint_idx = 0
        self.get_logger().info('Waypoints reset, starting from the first waypoint.')

    def _timer_cb(self):
        if not self._have_odom:
            return

        target = self.waypoint_locations[self.waypoint_idx][:2]
        distance = float(np.linalg.norm(self.current_position - target))

        if distance < self.waypoint_threshold:
            self.get_logger().info(f'Reached waypoint {self.waypoint_idx}')
            if self.waypoint_idx < len(self.waypoint_locations) - 1:
                self.waypoint_idx += 1
                self.get_logger().info(f'Moving to waypoint {self.waypoint_idx}')
            elif self.loop_waypoints:
                self._reset_waypoints()
            else:
                self.get_logger().info('Final waypoint reached, holding position.')
                self.timer.cancel()

        now = self.sim_clock.now()
        if now - self.last_publish_time >= self.publish_interval:
            self._publish_command()
            self._publish_markers()
            self.last_publish_time = now

    def _publish_command(self):
        target = self.waypoint_locations[self.waypoint_idx][:2]
        delta = target - self.current_position
        distance = float(np.linalg.norm(delta))
        bearing = float(np.arctan2(delta[1], delta[0]))
        yaw_error = float(np.arctan2(np.sin(bearing - self.current_yaw), np.cos(bearing - self.current_yaw)))

        surge = float(np.clip(distance / max(self.waypoint_threshold, 1e-6), 0.0, 1.0) * self.max_surge)
        yaw_rate = float(np.clip(self.yaw_gain * yaw_error, -self.max_yaw_rate, self.max_yaw_rate))
        heading_deg = float(np.degrees(bearing))  # absolute bearing to the waypoint, NWU-style degrees
        depth = self.target_depth

        cmd = Float64MultiArray()
        cmd.data = [_slot_value(tok, surge, yaw_rate, depth, heading_deg) for tok in self.command_slots]
        self.cmd_pub.publish(cmd)

    def _publish_markers(self):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Waypoints'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 600_000_000

        marker.points = []
        marker.colors = []
        for i, loc in enumerate(self.waypoint_locations):
            p = Point()
            p.x = float(loc[0])
            p.y = float(loc[1])
            p.z = 0.5
            marker.points.append(p)

            if i < self.waypoint_idx:
                c = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif i == self.waypoint_idx:
                c = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                c = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker.colors.append(c)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
