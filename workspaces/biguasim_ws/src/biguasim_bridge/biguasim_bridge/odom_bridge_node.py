#!/usr/bin/env python3
"""BiguaSim DynamicsSensor/Odom -> PoseStamped + TwistStamped.

Splits BiguaSim's nav_msgs/Odometry (true pose in the map frame + true
twist) into two plain, RViz-friendly topics:

  * pose_topic     (geometry_msgs/PoseStamped)  -- default /pose_gt
  * cmd_vel_topic  (geometry_msgs/TwistStamped) -- default /cmd_vel

Notes:
  * BiguaSim's Odom.twist.twist.linear is expressed in the WORLD (map)
    frame, not the body frame. The full vector is copied through unchanged.
  * The quaternion is passed through untouched (standard ROS ENU/FLU).
  * Outputs are throttled to max_rate_hz (default 20) so downstream Python
    consumers are not overloaded by BiguaSim's 30 Hz stream x playback rate.

All topic names and frame ids are parameters.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class OdomBridgeNode(Node):

    def __init__(self):
        super().__init__("odom_bridge_node")

        self.declare_parameter(
            "input_topic", "/biguasim/rov0_id0/DynamicsSensor/Odom")
        self.declare_parameter("pose_topic", "/pose_gt")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_pose", True)
        self.declare_parameter("publish_cmd_vel", True)
        # frame_id stamped on the outputs.
        self.declare_parameter("pose_frame_id", "odom")
        self.declare_parameter("cmd_vel_frame_id", "base_link")
        # Throttle the outputs to this rate (Hz). BiguaSim's Odom is 30 Hz and
        # gets multiplied by the bag playback rate; 180 msg/s is pointless load
        # on the Python consumers and causes queue-overflow drops (which, for
        # dead-reckoning, silently lose displacement). 0 = no throttle.
        self.declare_parameter("max_rate_hz", 20.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.publish_pose = self.get_parameter("publish_pose").value
        self.publish_cmd_vel = self.get_parameter("publish_cmd_vel").value
        self.pose_frame_id = self.get_parameter("pose_frame_id").value
        self.cmd_vel_frame_id = self.get_parameter("cmd_vel_frame_id").value
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self._min_dt = 1.0 / self.max_rate_hz if self.max_rate_hz > 0.0 else 0.0
        self._last_pub_stamp = None

        self.pub_pose = None
        self.pub_cmd_vel = None
        if self.publish_pose:
            self.pub_pose = self.create_publisher(
                PoseStamped, self.get_parameter("pose_topic").value, 10)
        if self.publish_cmd_vel:
            self.pub_cmd_vel = self.create_publisher(
                TwistStamped, self.get_parameter("cmd_vel_topic").value, 10)

        self.sub = self.create_subscription(
            Odometry, self.input_topic, self._cb, 50)

        self._n = 0
        self.get_logger().info(
            "odom_bridge: %s -> %s%s%s"
            % (self.input_topic,
               self.get_parameter("pose_topic").value if self.publish_pose else "",
               " + " if self.publish_pose and self.publish_cmd_vel else "",
               self.get_parameter("cmd_vel_topic").value if self.publish_cmd_vel else ""))

    def _cb(self, msg: Odometry):
        if self._min_dt > 0.0:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._last_pub_stamp is not None and \
                    0.0 <= stamp - self._last_pub_stamp < self._min_dt:
                return
            self._last_pub_stamp = stamp

        if self.pub_pose is not None:
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = self.pose_frame_id
            ps.pose = msg.pose.pose
            self.pub_pose.publish(ps)

        if self.pub_cmd_vel is not None:
            tw = TwistStamped()
            tw.header.stamp = msg.header.stamp
            tw.header.frame_id = self.cmd_vel_frame_id
            tw.twist = msg.twist.twist
            self.pub_cmd_vel.publish(tw)

        self._n += 1
        if self._n == 1:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.get_logger().info(
                "first Odom: pos=(%.2f, %.2f, %.2f) quat=(%.3f, %.3f, %.3f, %.3f)"
                % (p.x, p.y, p.z, q.x, q.y, q.z, q.w))


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
