import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

class UnderwaterEKFNode(Node):
    def __init__(self):
        super().__init__('underwater_ekf_node')

        # Estado inicial [x, y, yaw] correspondente à posição inicial do BlueROV2 no SkyDive
        # Location no YAML: [10.0, -301.0, -1.0], Rotation: [0, 0, 0]
        self.x = np.array([10.0, -301.0, 0.0])
        self.P = np.eye(3) * 0.1

        # Matrizes de Ruído de Processo (Q) e Medição (R)
        self.Q = np.diag([0.02, 0.02, 0.01])
        self.R_dvl = np.diag([0.05, 0.05])
        
        self.last_time = self.get_clock().now()

        # Inscrições nos tópicos publicados pelo BiguaSim ou pela Bag
        self.sub_dvl = self.create_subscription(
            TwistWithCovarianceStamped, 
            '/biguasim/auv0_id0/DVLSensor/Velocity', 
            self.dvl_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, 
            '/biguasim/auv0_id0/IMUSensor', 
            self.imu_callback, 10)

        # Publicador da odometria estimada
        self.odom_pub = self.create_publisher(Odometry, '/biguasim/auv0_id0/ekf_odom', 10)
        self.yaw_rate = 0.0
        self.get_logger().info('EKF Node iniciado! Publicando em /biguasim/auv0_id0/ekf_odom')

    def imu_callback(self, msg: Imu):
        # Medição de velocidade angular no eixo Z (yaw rate)
        self.yaw_rate = msg.angular_velocity.z

    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Evita deltas inválidos no primeiro frame ou pausas
        if dt <= 0.0 or dt > 0.5:
            return

        vx_body = msg.twist.twist.linear.x
        vy_body = msg.twist.twist.linear.y
        psi = self.x[2]

        # 1. PREDICÃO: Integração cinemática (Referencial Body -> World)
        cos_p = np.cos(psi)
        sin_p = np.sin(psi)

        self.x[0] += (vx_body * cos_p - vy_body * sin_p) * dt
        self.x[1] += (vx_body * sin_p + vy_body * cos_p) * dt
        self.x[2] += self.yaw_rate * dt

        # Jacobiana do modelo em relação ao estado (F = df / dx)
        F = np.eye(3)
        F[0, 2] = (-vx_body * sin_p - vy_body * cos_p) * dt
        F[1, 2] = ( vx_body * cos_p - vy_body * sin_p) * dt

        # Propagação da covariância P
        self.P = F @ self.P @ F.T + self.Q * dt

        # Normalizar yaw entre -pi e pi
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

        # Publicar estimativa de odometria calculada
        self.publish_odom(now)

    def publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = -1.0  # Profundidade fixa da missão
        
        # Orientação planar em quaternion (Z e W)
        msg.pose.pose.orientation.z = np.sin(self.x[2] / 2.0)
        msg.pose.pose.orientation.w = np.cos(self.x[2] / 2.0)
        self.odom_pub.publish(msg)

def main():
    rclpy.init()
    node = UnderwaterEKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando EKF Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()