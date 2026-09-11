"""Nodo que genera trayectorias controladas (cuadrado, circulo) comandando el
vehiculo via la abstraccion 'cmd_pos_yaw' de BiguaSim ([x, y, z, yaw_deg]),
y publica ademas la velocidad de cuerpo comandada (analitica, derivada de la
propia trayectoria) en <agent_ros_id>/cmd_vel_body.

Portado y recortado de biguasim_trajectory en el workspace 'anterior':
trajectory_type solo acepta 'square' | 'circle' (el foco actual), 'line' y
'figure_eight' quedaron fuera.

Requiere que el agente indicado en agent_ros_id tenga
control_abstraction: 'cmd_pos_yaw' en el escenario de biguasim_main (ver
config/trajectory_scenario.yaml en este mismo paquete) -- con
'cmd_motor_speeds' (el default de biguasim_main/config/config.yaml) este
nodo no mueve el vehiculo.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from biguasim_trajectory.trajectories import (
    TrajectoryPlan,
    make_dive_segment,
    sample_segment,
    build_square_shape,
    build_circle_shape,
)

VALID_TRAJECTORY_TYPES = ('square', 'circle')


class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        self.declare_parameter('agent_ros_id', 'auv0_id0')
        self.declare_parameter('trajectory_type', 'square')
        self.declare_parameter('square_side', 10.0)
        self.declare_parameter('circle_radius', 5.0)
        self.declare_parameter('num_loops', 1)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('dive_speed', 0.3)
        self.declare_parameter('target_depth', -3.0)
        self.declare_parameter('corner_wait_time', 3.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('loop_trajectory', False)
        self.declare_parameter('startup_timeout_sec', 10.0)

        self._agent_ros_id = self.get_parameter('agent_ros_id').value
        self._trajectory_type = self.get_parameter('trajectory_type').value
        self._loop = self.get_parameter('loop_trajectory').value
        self._startup_timeout = self.get_parameter('startup_timeout_sec').value

        if self._trajectory_type not in VALID_TRAJECTORY_TYPES:
            raise ValueError(
                f"trajectory_type '{self._trajectory_type}' desconocido, "
                f"debe ser uno de {VALID_TRAJECTORY_TYPES}")

        self._plan = None
        self._t_start = None
        self._origin_captured = False
        self._finished_logged = False
        self._node_start_time = self.get_clock().now()

        self._gt_sub = self.create_subscription(
            Odometry, f'{self._agent_ros_id}/DynamicsSensor/Odom', self._ground_truth_callback, 10)
        self._control_pub = self.create_publisher(
            Float64MultiArray, f'{self._agent_ros_id}/command_control', 10)
        self._cmd_vel_pub = self.create_publisher(
            TwistStamped, f'{self._agent_ros_id}/cmd_vel_body', 10)

        control_rate_hz = self.get_parameter('control_rate_hz').value
        self._timer = self.create_timer(1.0 / control_rate_hz, self._tick)

        self.get_logger().info(
            f"Esperando primer mensaje de ground truth en "
            f"{self._agent_ros_id}/DynamicsSensor/Odom...")

    def _ground_truth_callback(self, msg):
        if self._origin_captured:
            return
        self._origin_x = msg.pose.pose.position.x
        self._origin_y = msg.pose.pose.position.y
        self._origin_z = msg.pose.pose.position.z
        self._origin_captured = True
        self.get_logger().info(
            f"Origen capturado: ({self._origin_x:.2f}, {self._origin_y:.2f}, {self._origin_z:.2f})")

    def _build_shape_segments(self):
        speed = self.get_parameter('speed').value
        depth = self.get_parameter('target_depth').value

        if self._trajectory_type == 'square':
            return build_square_shape(
                self.get_parameter('square_side').value, speed, depth,
                self.get_parameter('corner_wait_time').value)
        return build_circle_shape(
            self.get_parameter('circle_radius').value, speed, depth,
            self.get_parameter('num_loops').value)

    def _build_plan(self):
        shape_segments = self._build_shape_segments()
        first_sample = sample_segment(shape_segments[0], 0.0)

        target_depth = self.get_parameter('target_depth').value
        dive_speed = self.get_parameter('dive_speed').value
        dive_duration = max(1.0, abs(self._origin_z - target_depth) / dive_speed)

        dive = make_dive_segment(
            x=0.0, y=0.0,
            z_start=self._origin_z, z_target=target_depth,
            yaw_start_deg=0.0, yaw_target_deg=first_sample.yaw_deg,
            duration=dive_duration)

        return TrajectoryPlan([dive] + shape_segments,
                               origin_x=self._origin_x, origin_y=self._origin_y)

    def _tick(self):
        if self._plan is None:
            if self._origin_captured:
                self._plan = self._build_plan()
                self._t_start = self.get_clock().now()
                self.get_logger().info(f"Trayectoria '{self._trajectory_type}' iniciada.")
            else:
                elapsed = (self.get_clock().now() - self._node_start_time).nanoseconds * 1e-9
                if elapsed >= self._startup_timeout:
                    self.get_logger().warn(
                        "No llego ground truth a tiempo; asumiendo origen (0, 0, target_depth).")
                    self._origin_x = 0.0
                    self._origin_y = 0.0
                    self._origin_z = self.get_parameter('target_depth').value
                    self._origin_captured = True
                return

        t = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9
        sample = self._plan.sample(t, loop=self._loop)

        control_msg = Float64MultiArray()
        control_msg.data = [sample.x, sample.y, sample.z, sample.yaw_deg]
        self._control_pub.publish(control_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self._agent_ros_id
        twist_msg.twist.linear.x = sample.vx_body
        twist_msg.twist.linear.y = sample.vy_body
        twist_msg.twist.angular.z = sample.wz
        self._cmd_vel_pub.publish(twist_msg)

        if sample.finished and not self._finished_logged:
            self.get_logger().info("Trayectoria completada; manteniendo posicion final.")
            self._finished_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
