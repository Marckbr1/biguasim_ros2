#!/usr/bin/env python3
"""BiguaSim ImagingSonar -> sensor_msgs/Image (mono8) Cartesian fan.

BiguaSim's ImagingSonar carries a *polar* float image (raw_image: 32FC1,
shape (bins_range, bins_azimuth)) plus its geometry (range_min/max,
azimuth_aperture_deg). This node remaps it to a *Cartesian fan* grayscale
image -- apex at bottom-centre, forward pointing up, rows = range -- and
quantises the float intensities to mono8, so it displays directly in RViz2
(Image / Camera display) and is consumable by any LaserScan converter.

Geometry conventions (all parameters, because the polar axis directions are
not documented and should be confirmed visually once the sim runs):

  range_axis_near_first = True   polar row 0 is the nearest range (range_min)
  azimuth_flip          = False  polar col 0 -> right side of the fan

Output: raw sensor_msgs/Image (mono8) on output_topic (default /son).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from biguasim_interfaces.msg import ImagingSonar

_ENCODING_DTYPE = {
    "32FC1": "<f4",
    "64FC1": "<f8",
    "mono8": "u1",
    "8UC1": "u1",
    "mono16": "<u2",
    "16UC1": "<u2",
}


class SonarBridgeNode(Node):

    def __init__(self):
        super().__init__("sonar_bridge_node")

        self.declare_parameter("input_topic", "/biguasim/rov0_id0/ImagingSonar")
        self.declare_parameter("output_topic", "/son")
        self.declare_parameter("output_frame_id", "son")
        # which sub-image of ImagingSonar to render: "raw_image" or "intensity"
        self.declare_parameter("source_image", "raw_image")
        # linear gain applied to the float intensities before clipping to 0-255
        self.declare_parameter("intensity_scale", 600.0)
        # optional gamma (<1 brightens weak returns); applied on the 0-1 range
        self.declare_parameter("intensity_gamma", 1.0)
        # Cartesian output image height in pixels (range resolution). 0 -> use
        # bins_range from the message.
        self.declare_parameter("output_height", 512)
        # output width in pixels. 0 -> auto: 2 * H * sin(aperture/2) + 1
        self.declare_parameter("output_width", 0)
        # polar axis orientation (see module docstring)
        self.declare_parameter("range_axis_near_first", True)
        # azimuth_flip=False: polar column 0 -> starboard (image right).
        # Flip if the fan comes out mirrored versus the sim view.
        self.declare_parameter("azimuth_flip", False)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.source_image = self.get_parameter("source_image").value
        self.intensity_scale = float(self.get_parameter("intensity_scale").value)
        self.intensity_gamma = float(self.get_parameter("intensity_gamma").value)
        self.output_height = int(self.get_parameter("output_height").value)
        self.output_width = int(self.get_parameter("output_width").value)
        self.range_axis_near_first = bool(
            self.get_parameter("range_axis_near_first").value)
        self.azimuth_flip = bool(self.get_parameter("azimuth_flip").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value, qos)
        self.sub = self.create_subscription(
            ImagingSonar, self.input_topic, self._cb, qos)

        self._map_key = None
        self._flat_idx = None   # (H*W,) int32, -1 where outside the fan
        self._n = 0
        self.get_logger().info(
            "sonar_bridge: %s (%s) -> %s (mono8 Cartesian fan)"
            % (self.input_topic, self.source_image,
               self.get_parameter("output_topic").value))

    # -- polar -> Cartesian pixel index map (cached; rebuilt only if geometry
    #    changes) -------------------------------------------------------------
    def _build_map(self, h_pol, w_pol, aperture_deg, range_min, range_max):
        H = self.output_height if self.output_height > 0 else h_pol
        ap = math.radians(aperture_deg)
        if self.output_width > 0:
            W = self.output_width
        else:
            W = int(2.0 * H * math.sin(ap / 2.0)) + 1
        apex_x = W / 2.0

        ys, xs = np.mgrid[0:H, 0:W].astype(np.float64)
        dx = xs - apex_x
        dy = ys - H                      # <= 0, distance above the apex row
        r_px = np.hypot(dx, dy)
        ang = np.arctan2(-dx, -dy)       # straight up = 0, image-right = negative

        range_m = r_px / float(H) * range_max
        valid = (np.abs(ang) <= ap / 2.0) & \
                (range_m >= range_min) & (range_m <= range_max)

        rb = (range_m - range_min) / (range_max - range_min) * (h_pol - 1)
        if not self.range_axis_near_first:
            rb = (h_pol - 1) - rb
        ab = (ang + ap / 2.0) / ap * (w_pol - 1)
        if self.azimuth_flip:
            ab = (w_pol - 1) - ab

        rb = np.clip(np.round(rb), 0, h_pol - 1).astype(np.int64)
        ab = np.clip(np.round(ab), 0, w_pol - 1).astype(np.int64)
        flat = rb * w_pol + ab
        flat[~valid] = -1

        self._out_shape = (H, W)
        self._flat_idx = flat.reshape(-1)
        self._valid = valid.reshape(-1)
        self.get_logger().info(
            "remap built: polar (%d x %d) -> fan (%d x %d), aperture %.1f deg, "
            "range %.2f-%.2f m, near_first=%s azimuth_flip=%s"
            % (h_pol, w_pol, H, W, aperture_deg, range_min, range_max,
               self.range_axis_near_first, self.azimuth_flip))

    def _cb(self, msg: ImagingSonar):
        img = getattr(msg, self.source_image)
        dtype = _ENCODING_DTYPE.get(img.encoding)
        if dtype is None:
            self.get_logger().error(
                "unsupported sub-image encoding '%s'" % img.encoding)
            return
        if img.height == 0 or img.width == 0:
            return

        polar = np.frombuffer(bytes(img.data), dtype=dtype)
        polar = polar.reshape(img.height, img.width).astype(np.float32)

        key = (img.height, img.width,
               round(msg.azimuth_aperture_deg, 4),
               round(msg.range_min, 4), round(msg.range_max, 4))
        if key != self._map_key:
            self._build_map(img.height, img.width,
                            msg.azimuth_aperture_deg,
                            msg.range_min, msg.range_max)
            self._map_key = key

        flat_polar = polar.reshape(-1)
        idx = self._flat_idx
        sampled = flat_polar[np.where(idx >= 0, idx, 0)]
        sampled[~self._valid] = 0.0

        vals = sampled * self.intensity_scale
        if self.intensity_gamma != 1.0:
            vals = np.clip(vals / 255.0, 0.0, 1.0) ** self.intensity_gamma * 255.0
        mono8 = np.clip(vals, 0.0, 255.0).astype(np.uint8)

        H, W = self._out_shape
        out = Image()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame_id
        out.height = H
        out.width = W
        out.encoding = "mono8"
        out.is_bigendian = 0
        out.step = W
        out.data = mono8.tobytes()
        self.pub.publish(out)

        self._n += 1
        if self._n == 1 or self._n % 200 == 0:
            self.get_logger().info(
                "frame %d: polar max=%.4f -> fan nonzero=%.1f%% max=%d"
                % (self._n, float(polar.max()),
                   100.0 * float((mono8 > 0).mean()), int(mono8.max())))


def main(args=None):
    rclpy.init(args=args)
    node = SonarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
