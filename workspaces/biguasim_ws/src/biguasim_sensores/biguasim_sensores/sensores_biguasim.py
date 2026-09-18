#!/usr/bin/env python3
"""
Leitura dos sensores do ROV no BiguaSim.

Contrato de tópicos e tipos conforme o biguasim_sensor_check:
  /biguasim/rov0_id0/DynamicsSensor/Odom   nav_msgs/Odometry
  /biguasim/rov0_id0/IMUSensor             sensor_msgs/Imu
  /biguasim/rov0_id0/ImagingSonar          biguasim_interfaces/ImagingSonar
  /biguasim/rov0_id0/DVLSensor/Velocity    geometry_msgs/TwistWithCovarianceStamped
  /biguasim/rov0_id0/DVLSensor/Range       biguasim_interfaces/DVLSensorRange
  /biguasim/rov0_id0/DepthSensor           geometry_msgs/PoseWithCovarianceStamped
  /biguasim/rov0_id0/MagnetometerSensor    sensor_msgs/MagneticField

Tópico vazio ("") desativa o sensor. O prefixo do veículo é parâmetro, então
dá para trocar rov0_id0 por auv0_id0 sem editar código.
Publica um resumo em /status_sensores.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, Image
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped

try:
    from biguasim_interfaces.msg import ImagingSonar, DVLSensorRange
    INTERFACES_OK = True
except ImportError:
    ImagingSonar = DVLSensorRange = None
    INTERFACES_OK = False

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_OK = True
except ImportError:
    CV_BRIDGE_OK = False


class SensorInfo:
    def __init__(self, nome):
        self.nome = nome
        self.ultima_msg = None
        self.ultimo_t = None
        self.contador = 0

    def atualiza(self, msg, agora):
        self.ultima_msg = msg
        self.ultimo_t = agora
        self.contador += 1

    def idade(self, agora):
        return None if self.ultimo_t is None else agora - self.ultimo_t


class SensoresBiguaSim(Node):
    def __init__(self):
        super().__init__('sensores_biguasim')

        self.declare_parameter('prefixo', '/biguasim/rov0_id0')
        self.declare_parameter('sub_odom', 'DynamicsSensor/Odom')
        self.declare_parameter('sub_imu', 'IMUSensor')
        self.declare_parameter('sub_sonar', 'ImagingSonar')
        self.declare_parameter('sub_dvl_vel', 'DVLSensor/Velocity')
        self.declare_parameter('sub_dvl_range', 'DVLSensor/Range')
        self.declare_parameter('sub_depth', 'DepthSensor')
        self.declare_parameter('sub_mag', 'MagnetometerSensor')
        self.declare_parameter('topico_camera', '')   # não existe no ROV atual
        self.declare_parameter('qos_best_effort', True)
        self.declare_parameter('periodo_status', 2.0)

        p = self.get_parameter
        prefixo = p('prefixo').value.rstrip('/')

        def full(sub):
            return f'{prefixo}/{sub}' if sub else ''

        if p('qos_best_effort').value:
            qos = qos_profile_sensor_data
        else:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sensores = []
        self.altitude_dvl = None      # menor feixe válido do DVL (m)
        self.alcance_sonar = None     # eco mais próximo do sonar (m)
        self.bridge = CvBridge() if CV_BRIDGE_OK else None

        self.odom = self._assina(Odometry, full(p('sub_odom').value), 'Odom', self.cb_odom, qos)
        self.imu = self._assina(Imu, full(p('sub_imu').value), 'IMU', self.cb_imu, qos)
        self.dvl_vel = self._assina(TwistWithCovarianceStamped, full(p('sub_dvl_vel').value),
                                    'DVL_vel', self.cb_dvl_vel, qos)
        self.depth = self._assina(PoseWithCovarianceStamped, full(p('sub_depth').value),
                                  'Depth', self.cb_depth, qos)
        self.mag = self._assina(MagneticField, full(p('sub_mag').value), 'Mag', self.cb_mag, qos)
        self.camera = self._assina(Image, p('topico_camera').value, 'Camera', self.cb_camera, qos)

        # Tipos customizados: só assina se o pacote de interfaces estiver no workspace.
        self.sonar = self.dvl_range = None
        if INTERFACES_OK:
            self.sonar = self._assina(ImagingSonar, full(p('sub_sonar').value),
                                      'Sonar', self.cb_sonar, qos)
            self.dvl_range = self._assina(DVLSensorRange, full(p('sub_dvl_range').value),
                                          'DVL_range', self.cb_dvl_range, qos)
        else:
            self.get_logger().error(
                'biguasim_interfaces não encontrado: sonar e DVL/Range desativados. '
                'Verifique se o pacote foi compilado e se o install/setup.bash foi carregado.')

        if not CV_BRIDGE_OK:
            self.get_logger().warn('cv_bridge ausente: imagens não serão convertidas.')

        self.pub_status = self.create_publisher(String, '/status_sensores', 10)
        self.create_timer(p('periodo_status').value, self.publica_status)

    def _assina(self, tipo_msg, topico, nome, callback, qos):
        if not topico:
            self.get_logger().info(f'{nome}: desativado')
            return None
        info = SensorInfo(nome)
        self.create_subscription(tipo_msg, topico, callback, qos)
        self.sensores.append(info)
        self.get_logger().info(f'{nome} -> {topico}')
        return info

    # ---------------- Callbacks ----------------
    def _agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_odom(self, msg: Odometry):
        self.odom.atualiza(msg, self._agora())
        pos = msg.pose.pose.position
        self.get_logger().debug(f'Odom x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}')

    def cb_imu(self, msg: Imu):
        self.imu.atualiza(msg, self._agora())
        q = msg.orientation
        self.get_logger().debug(
            f'IMU quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}) '
            f'az={msg.linear_acceleration.z:.3f}')

    def cb_dvl_vel(self, msg: TwistWithCovarianceStamped):
        self.dvl_vel.atualiza(msg, self._agora())
        v = msg.twist.twist.linear
        self.get_logger().debug(f'DVL vel=({v.x:.3f}, {v.y:.3f}, {v.z:.3f}) m/s')

    def cb_dvl_range(self, msg):
        """float32[4] range: os quatro feixes. O menor válido ~ altitude."""
        self.dvl_range.atualiza(msg, self._agora())
        feixes = np.asarray(msg.range, dtype=float)
        validos = feixes[np.isfinite(feixes) & (feixes > 0.0)]
        if validos.size == 0:
            # Sem retorno nos 4 feixes: fora de alcance do fundo (ou na superfície).
            self.altitude_dvl = None
            return
        self.altitude_dvl = float(validos.min())
        self.get_logger().info(
            f'DVL: {validos.size}/4 feixes, altitude~{self.altitude_dvl:.2f} m '
            f'(feixes: {np.array2string(feixes, precision=2)})',
            throttle_duration_sec=2.0)

    def cb_depth(self, msg: PoseWithCovarianceStamped):
        self.depth.atualiza(msg, self._agora())
        z = msg.pose.pose.position.z
        # Profundidade é o critério mais direto para saber se está submerso.
        self.get_logger().debug(f'Depth z={z:.3f} m')

    def cb_mag(self, msg: MagneticField):
        self.mag.atualiza(msg, self._agora())
        m = msg.magnetic_field
        self.get_logger().debug(f'Mag=({m.x:.3e}, {m.y:.3e}, {m.z:.3e}) T')

    def cb_sonar(self, msg):
        """Imagem polar 32FC1 + geometria embutida na própria mensagem."""
        self.sonar.atualiza(msg, self._agora())
        if self.bridge is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg.raw_image, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Falha ao converter raw_image: {e}')
            return

        # O eixo de range pode vir nas linhas ou nas colunas; decide pelos bins.
        if img.shape == (msg.bins_range, msg.bins_azimuth):
            eixo_range = 0
        elif img.shape == (msg.bins_azimuth, msg.bins_range):
            eixo_range = 1
        else:
            self.get_logger().warn(
                f'Shape {img.shape} não bate com bins '
                f'(range={msg.bins_range}, azimuth={msg.bins_azimuth})',
                throttle_duration_sec=5.0)
            return

        # Retorno mais forte de cada feixe -> índice do bin -> metros.
        idx = np.argmax(img, axis=eixo_range)
        intens = np.max(img, axis=eixo_range)
        passo = (msg.range_max - msg.range_min) / max(msg.bins_range - 1, 1)
        alcances = msg.range_min + idx * passo

        # Descarta feixes sem eco (limiar simples sobre a intensidade).
        limiar = float(np.mean(img) + 2.0 * np.std(img))
        com_eco = intens > limiar
        self.alcance_sonar = float(alcances[com_eco].min()) if com_eco.any() else None

        if self.alcance_sonar is not None:
            self.get_logger().info(
                f'Sonar: {int(com_eco.sum())}/{com_eco.size} feixes com eco, '
                f'mais próximo {self.alcance_sonar:.2f} m '
                f'(abertura {msg.azimuth_aperture_deg:.1f} graus, '
                f'{msg.range_min:.1f}-{msg.range_max:.1f} m)',
                throttle_duration_sec=2.0)

    def cb_camera(self, msg: Image):
        self.camera.atualiza(msg, self._agora())
        if self.bridge is not None:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.get_logger().debug(f'Frame {frame.shape}')
            except Exception as e:
                self.get_logger().error(f'Falha ao converter imagem: {e}')

    # ---------------- Status ----------------
    def publica_status(self):
        agora = self._agora()
        partes = []
        for s in self.sensores:
            idade = s.idade(agora)
            partes.append(f'{s.nome}=SILENT' if idade is None
                          else f'{s.nome}=ok({s.contador}, {idade:.2f}s)')

        msg = String()
        msg.data = ' | '.join(partes) if partes else 'nenhum sensor configurado'
        self.pub_status.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SensoresBiguaSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()