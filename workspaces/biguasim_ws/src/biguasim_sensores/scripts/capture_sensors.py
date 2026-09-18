import rclpy
from rclpy.node import Node
import csv
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

class SensorRecorder(Node):
    def __init__(self):
        super().__init__('sensor_recorder')
        self.csv_file = open('sensor_log.csv', mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'sec', 'nanosec',
            'pos_x', 'pos_y', 'pos_z',
            'quat_x', 'quat_y', 'quat_z', 'quat_w',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'dvl_vx', 'dvl_vy', 'dvl_vz'
        ])

        self.last_imu = [0.0]*6
        self.last_dvl = [0.0]*3

        self.create_subscription(Imu, '/biguasim/auv0_id0/IMUSensor', self.imu_cb, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/biguasim/auv0_id0/DVLSensor/Velocity', self.dvl_cb, 10)
        self.create_subscription(Odometry, '/biguasim/auv0_id0/DynamicsSensor/Odom', self.odom_cb, 10)

        self.get_logger().info('Gravador iniciado. Salvando em sensor_log.csv...')

    def imu_cb(self, msg):
        self.last_imu = [
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]

    def dvl_cb(self, msg):
        self.last_dvl = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.writer.writerow([
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w,
            *self.last_imu,
            *self.last_dvl
        ])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parando gravação e salvando arquivo.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
