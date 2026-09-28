import rclpy
from rclpy.node import Node
import csv
import os
import pandas as pd
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

class TrajectoryComparator(Node):
    def __init__(self):
        super().__init__('trajectory_comparator')
        self.csv_path = 'comparison_log.csv'
        self.file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['sec', 'nanosec', 'gt_x', 'gt_y', 'ekf_x', 'ekf_y'])

        self.last_gt = None
        self.last_ekf = None

        # Ground Truth
        self.create_subscription(
            Odometry,
            '/biguasim/auv0_id0/DynamicsSensor/Odom',
            self.gt_cb,
            10
        )

        # Estimativa EKF
        self.create_subscription(
            Odometry,
            '/biguasim/auv0_id0/ekf_odom',
            self.ekf_cb,
            10
        )

        # Timer para gravar os dados sincronizados a 10 Hz
        self.timer = self.create_timer(0.1, self.record_step)
        self.get_logger().info('Gravador de comparacao iniciado. Aguardando topicos...')

    def gt_cb(self, msg: Odometry):
        self.last_gt = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def ekf_cb(self, msg: Odometry):
        self.last_ekf = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def record_step(self):
        if self.last_gt is not None and self.last_ekf is not None:
            now = self.get_clock().now().to_msg()
            self.writer.writerow([
                now.sec, now.nanosec,
                self.last_gt[0], self.last_gt[1],
                self.last_ekf[0], self.last_ekf[1]
            ])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def plot_results():
    if not os.path.exists('comparison_log.csv'):
        print("Log de comparacao nao encontrado.")
        return

    df = pd.read_csv('comparison_log.csv')
    if len(df) < 5:
        print("Poucas amostras coletadas para comparacao.")
        return

    # Calculo do erro euclidiano ponto a ponto (L2 norm)
    error = ((df['gt_x'] - df['ekf_x'])**2 + (df['gt_y'] - df['ekf_y'])**2)**0.5

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Grafico 1: Trajetorias XY sobrepostas
    ax1.plot(df['gt_x'], df['gt_y'], label='Ground Truth (Real)', color='green', lw=2)
    ax1.plot(df['ekf_x'], df['ekf_y'], label='Estimativa EKF (DVL+IMU)', color='blue', linestyle='--', lw=2)
    ax1.scatter(df['gt_x'].iloc[0], df['gt_y'].iloc[0], color='black', s=80, marker='o', label='Inicio')
    ax1.set_title('Comparativo de Trajetoria: Ground Truth vs EKF')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.axis('equal')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend()

    # Grafico 2: Erro de posicao ao longo do tempo (drift acumulado)
    ax2.plot(error, color='crimson', lw=2)
    ax2.set_title(f'Erro de Posicao Euclidiano (Medio: {error.mean():.3f} m)')
    ax2.set_xlabel('Amostras (a 10 Hz)')
    ax2.set_ylabel('Erro [m]')
    ax2.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    plt.savefig('comparacao_ekf_vs_gt.png', dpi=300)
    print(f"Grafico salvo em 'comparacao_ekf_vs_gt.png'! Erro final: {error.iloc[-1]:.3f} m.")

def main():
    rclpy.init()
    node = TrajectoryComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Finalizando captura e gerando grafico...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plot_results()

if __name__ == '__main__':
    main()
