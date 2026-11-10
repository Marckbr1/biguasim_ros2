import rclpy
from rclpy.node import Node
import numpy as np
import csv

# Caso o BiguaSim use sensor_msgs/LaserScan ou pacote próprio como ping360_sonar_msgs
from sensor_msgs.msg import LaserScan

class SonarDistanceExtractor(Node):
    def __init__(self):
        super().__init__('sonar_distance_extractor')
        
        self.csv_file = open('sonar_distance_log.csv', mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['sec', 'nanosec', 'angle_rad', 'detected_distance_m', 'max_intensity'])

        # Se o tópico for do tipo LaserScan:
        self.sub = self.create_subscription(
            LaserScan,
            '/biguasim/auv0_id0/ImagingSonar/ScanLine',
            self.sonar_callback,
            10
        )
        self.get_logger().info('Extrator de Sonar iniciado! Gravando em sonar_distance_log.csv...')

    def sonar_callback(self, msg: LaserScan):
        # Para cada varredura, localizamos a distância refletida pelo obstáculo
        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities) if len(msg.intensities) > 0 else None

        # Filtra retornos nulos ou ruídos próximos à carcaça do robô (< 0.5m)
        valid_indices = np.where((ranges >= 0.5) & (ranges <= msg.range_max))[0]

        if len(valid_indices) > 0:
            if intensities is not None and len(intensities) == len(ranges):
                peak_idx = valid_indices[np.argmax(intensities[valid_indices])]
                dist = float(ranges[peak_idx])
                max_int = float(intensities[peak_idx])
            else:
                dist = float(np.min(ranges[valid_indices]))
                max_int = 1.0

            angle = float(msg.angle_min)
            self.writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                round(angle, 4),
                round(dist, 3),
                round(max_int, 2)
            ])
            self.get_logger().info(f"Ângulo: {np.degrees(angle):.1f}° | Obstáculo a: {dist:.2f} m")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = SonarDistanceExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando extração do sonar.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()