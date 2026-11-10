#!/usr/bin/env python3
"""
Nó EKF 3D (6-DoF) autossuficiente para fusão de sensores do agente híbrido uav0.
Sem dependência externa de tf_transformations.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path


def quaternion_to_euler(x, y, z, w):
    """Converte quaternion para ângulos de Euler (roll, pitch, yaw) em radianos."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Converte ângulos de Euler (roll, pitch, yaw) para quaternion [x, y, z, w]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [x, y, z, w]


class EKF3DNode(Node):
    def __init__(self):
        super().__init__('ekf_3d_node')

        self.declare_parameter('agent_ros_id', 'uav0_id0')
        self.declare_parameter('use_dvl', True)
        self.declare_parameter('gravity', 9.80665)

        self.agent_id = self.get_parameter('agent_ros_id').value
        self.use_dvl = self.get_parameter('use_dvl').value
        self.g = np.array([0.0, 0.0, self.get_parameter('gravity').value])

        # Estado: [px, py, pz, vx, vy, vz, phi, theta, psi]
        self.x = np.zeros(9)
        self.P = np.eye(9) * 0.1

        # Ruídos de processo (Q) e medição (R)
        self.Q = np.diag([
            0.01, 0.01, 0.01,    # pos
            0.1,  0.1,  0.1,     # vel
            0.001, 0.001, 0.001  # roll, pitch, yaw
        ])
        
        self.R_dvl = np.diag([0.05, 0.05, 0.05])
        self.R_att = np.diag([0.005, 0.005, 0.005])

        self.last_imu_time = None
        self.origin_initialized = False

        prefix = f'/biguasim/{self.agent_id}'
        self.sub_imu = self.create_subscription(
            Imu, f'{prefix}/IMUSensor', self.imu_callback, 100)

        if self.use_dvl:
            self.sub_dvl = self.create_subscription(
                TwistWithCovarianceStamped, f'{prefix}/DVLSensor/Velocity', self.dvl_callback, 100)

        self.sub_gt = self.create_subscription(
            Odometry, f'{prefix}/DynamicsSensor/Odom', self.gt_callback, 100)

        self.pub_ekf_odom = self.create_publisher(Odometry, f'{prefix}/ekf/odom', 10)
        self.pub_ekf_path = self.create_publisher(Path, f'{prefix}/ekf/path', 10)
        self.pub_gt_path = self.create_publisher(Path, f'{prefix}/ground_truth/path', 10)

        self.ekf_path = Path()
        self.gt_path = Path()

        self.get_logger().info(f"Nó EKF 3D inicializado para agente: {self.agent_id}")

    def rotation_matrix(self, phi, theta, psi):
        c_p, s_p = math.cos(phi), math.sin(phi)
        c_t, s_t = math.cos(theta), math.sin(theta)
        c_y, s_y = math.cos(psi), math.sin(psi)

        return np.array([
            [c_y * c_t, c_y * s_t * s_p - s_y * c_p, c_y * s_t * c_p + s_y * s_p],
            [s_y * c_t, s_y * s_t * s_p + c_y * c_p, s_y * s_t * c_p - c_y * s_p],
            [-s_t,      c_t * s_p,                   c_t * c_p]
        ])

    def gt_callback(self, msg: Odometry):
        if not self.origin_initialized:
            self.x[0] = msg.pose.pose.position.x
            self.x[1] = msg.pose.pose.position.y
            self.x[2] = msg.pose.pose.position.z
            q = msg.pose.pose.orientation
            r, p, y = quaternion_to_euler(q.x, q.y, q.z, q.w)
            self.x[6:9] = [r, p, y]
            self.origin_initialized = True
            self.get_logger().info(
                f"Origem alinhada ao Ground Truth: P=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f})")

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.gt_path.header = msg.header
        self.gt_path.poses.append(pose)
        if len(self.gt_path.poses) > 2000:
            self.gt_path.poses.pop(0)
        self.pub_gt_path.publish(self.gt_path)

    def imu_callback(self, msg: Imu):
        if not self.origin_initialized:
            return

        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = t_now
            return

        dt = t_now - self.last_imu_time
        self.last_imu_time = t_now

        if dt <= 0.0 or dt > 0.5:
            return

        # 1. Predição
        R = self.rotation_matrix(self.x[6], self.x[7], self.x[8])
        a_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        a_global = R @ a_body - self.g

        self.x[0:3] += self.x[3:6] * dt + 0.5 * a_global * (dt ** 2)
        self.x[3:6] += a_global * dt

        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt
        self.P = F @ self.P @ F.T + self.Q * dt

        # 2. Correção de orientação
        q = msg.orientation
        r, p, y = quaternion_to_euler(q.x, q.y, q.z, q.w)
        z_att = np.array([r, p, y])

        H_att = np.zeros((3, 9))
        H_att[0:3, 6:9] = np.eye(3)

        y_att = z_att - self.x[6:9]
        for i in range(3):
            y_att[i] = math.atan2(math.sin(y_att[i]), math.cos(y_att[i]))

        S_att = H_att @ self.P @ H_att.T + self.R_att
        K_att = self.P @ H_att.T @ np.linalg.inv(S_att)
        self.x += K_att @ y_att
        self.P = (np.eye(9) - K_att @ H_att) @ self.P

        self.publish_ekf_odom(msg.header)

    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        if not self.origin_initialized:
            return

        R = self.rotation_matrix(self.x[6], self.x[7], self.x[8])
        v_body = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        v_global_meas = R @ v_body

        H_v = np.zeros((3, 9))
        H_v[0:3, 3:6] = np.eye(3)

        y_v = v_global_meas - self.x[3:6]
        S_v = H_v @ self.P @ H_v.T + self.R_dvl
        K_v = self.P @ H_v.T @ np.linalg.inv(S_v)

        self.x += K_v @ y_v
        self.P = (np.eye(9) - K_v @ H_v) @ self.P

    def publish_ekf_odom(self, header):
        odom = Odometry()
        odom.header = header
        odom.header.frame_id = 'world'
        odom.child_frame_id = f'{self.agent_id}/base_link'

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.position.z = float(self.x[2])

        q = euler_to_quaternion(self.x[6], self.x[7], self.x[8])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = float(self.x[3])
        odom.twist.twist.linear.y = float(self.x[4])
        odom.twist.twist.linear.z = float(self.x[5])

        self.pub_ekf_odom.publish(odom)

        pose = PoseStamped()
        pose.header = header
        pose.pose = odom.pose.pose
        self.ekf_path.header = header
        self.ekf_path.poses.append(pose)
        if len(self.ekf_path.poses) > 2000:
            self.ekf_path.poses.pop(0)
        self.pub_ekf_path.publish(self.ekf_path)


def main(args=None):
    rclpy.init(args=args)
    node = EKF3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()