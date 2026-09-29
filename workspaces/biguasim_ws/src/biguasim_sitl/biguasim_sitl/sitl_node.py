#!/usr/bin/env python3
"""
Nó ROS 2 que roda o laço ArduPilot JSON SITL do BiguaSim.

Divisão de responsabilidades:
  MAVROS    -> navegação, missão/waypoints, estado do veículo e GPS (via ArduPilot)
  este nó   -> env.step(), ponte UDP com o ArduPilot, sensores de percepção

Só um processo pode ser dono do ambiente BiguaSim: este nó SUBSTITUI o
biguasim_node/biguasim_main quando se roda em modo SITL.

Cadeia completa:
  QGroundControl <--MAVLink 14550--> ArduPilot SITL <--UDP 9002--> este nó --> BiguaSim
  MAVROS         <--MAVLink 14551-->
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

import biguasim
from biguasim.ardubridge.bridge import ArduPilotBridge
from biguasim.ardubridge.runner import ArduBiguaSimRunner
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_OK = True
except ImportError:
    CV_BRIDGE_OK = False


class BiguaSimSITLNode(Node):
    def __init__(self):
        super().__init__('biguasim_sitl')

        # ---------------- Parâmetros ----------------
        self.declare_parameter('vehicle', 'BlueROV2')
        self.declare_parameter('package_name', 'SkyDive')
        self.declare_parameter('world', 'Pier-Harbor')
        self.declare_parameter('ticks_per_sec', 200)
        self.declare_parameter('show_viewport', True)
        self.declare_parameter('location', [0.0, 0.0, -1.0])

        # Origem GPS sintética. NÃO há GPS no simulador: a lat/lon enviada ao
        # ArduPilot é calculada a partir da posição local com esta origem.
        # Precisa bater com o -L do sim_vehicle.py, senão o EKF diverge.
        self.declare_parameter('gps_lat', 33.810313)    # RAT Beach (default do repo)
        self.declare_parameter('gps_lon', -118.393867)

        self.declare_parameter('bridge_address', '127.0.0.1')
        self.declare_parameter('bridge_port', 9002)

        # Percepção: Hz próprio, senão rodariam a ticks_per_sec (200 Hz).
        self.declare_parameter('enable_sonar', False)
        self.declare_parameter('sonar_hz', 10)
        self.declare_parameter('enable_camera', False)
        self.declare_parameter('camera_hz', 20)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)

        self.declare_parameter('periodo_status', 2.0)

        p = self.get_parameter
        nome = p('vehicle').value
        if nome not in VEHICLE_REGISTRY:
            raise ValueError(f"veículo '{nome}' desconhecido. "
                             f"Disponíveis: {list(VEHICLE_REGISTRY)}")
        self.profile = VEHICLE_REGISTRY[nome]
        self.ticks = p('ticks_per_sec').value
        self.dt = 1.0 / self.ticks
        self.periodo_status = p('periodo_status').value

        # ---------------- Cenário ----------------
        scenario = ArduBiguaSimRunner.build_scenario(
            self.profile,
            package_name=p('package_name').value,
            world=p('world').value,
            location=list(p('location').value),
            ticks_per_sec=self.ticks,
        )
        self._acrescenta_percepcao(scenario['agents'][0]['sensors'])
        self.agent = scenario['main_agent']

        # ---------------- Ambiente + ponte ----------------
        self.env = biguasim.make(
            scenario_cfg=scenario,
            show_viewport=p('show_viewport').value,
            verbose=False)
        self.ponte = ArduPilotBridge(
            self.profile,
            address=p('bridge_address').value,
            port=p('bridge_port').value,
            gps_origin=(p('gps_lat').value, p('gps_lon').value))

        self.cv = CvBridge() if CV_BRIDGE_OK else None
        if not CV_BRIDGE_OK:
            self.get_logger().warn('cv_bridge ausente: imagens não serão publicadas.')

        # ---------------- Publishers ----------------
        self.pub_odom = self.create_publisher(Odometry, '~/odom_gt', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '~/velocity_gt', 10)
        self.pub_sonar = self.create_publisher(Image, '~/sonar/image', 10)
        self.pub_cam = self.create_publisher(Image, '~/camera/image_raw', 10)
        self.pub_status = self.create_publisher(String, '~/status', 10)

        self.sim_time = 0.0
        self.frames = 0
        self._ultimo_status = 0.0

        self.get_logger().info(
            f'{nome} / {self.profile.ardupilot_vehicle} @ {self.ticks} Hz | '
            f'origem GPS ({p("gps_lat").value:.6f}, {p("gps_lon").value:.6f}) | '
            f'UDP {p("bridge_address").value}:{p("bridge_port").value}')
        self.get_logger().info(f'SITL sugerido: {self.profile.sitl_args}')

    # ------------------------------------------------------------------
    def _acrescenta_percepcao(self, sensores: list):
        """build_scenario só monta os sensores exigidos pelo SITL."""
        p = self.get_parameter
        if p('enable_sonar').value:
            sensores.append({
                'sensor_type': 'ImagingSonar',
                'Hz': p('sonar_hz').value,
                'configuration': {
                    'RangeBins': 64, 'AzimuthBins': 64,
                    'RangeMin': 0.5, 'RangeMax': 40,
                    'InitOctreeRange': 20,
                    'Elevation': 28, 'Azimuth': 28.8,
                    'ScaleNoise': True, 'AddSigma': 0.15,
                    'MultSigma': 0.2, 'MultiPath': True,
                },
            })
        if p('enable_camera').value:
            sensores.append({
                'sensor_type': 'RGBCamera',
                'Hz': p('camera_hz').value,
                'configuration': {
                    'CaptureWidth': p('camera_width').value,
                    'CaptureHeight': p('camera_height').value,
                },
            })

    # ------------------------------------------------------------------
    def inicia(self):
        self.ponte.bind()
        self.env.__enter__()
        self.env.step([0.0] * self.profile.num_motors)
        self.get_logger().info('Aguardando o ArduPilot na porta UDP...')

    def passo(self) -> bool:
        """Um ciclo do SITL. False = nenhum pacote recebido neste ciclo."""
        frame, pwm = self.ponte.receive_pwm()
        if frame is None:
            return False

        cmds = self.ponte.pwm_to_motor_cmds(pwm, frame)
        estado = self.env.step(cmds)[self.agent][0]
        self.sim_time += self.dt
        self.frames += 1

        json_state = self.ponte.build_json_state(estado, self.sim_time)
        if json_state is None:
            # build_json_state devolve None quando algum valor não é finito:
            # a dinâmica divergiu. Nada é enviado ao ArduPilot neste ciclo.
            self.get_logger().error('estado não-finito: dinâmica divergiu',
                                    throttle_duration_sec=1.0)
        else:
            self.ponte.send_state(json_state)

        self.publica(estado)
        return True

    # ------------------------------------------------------------------
    def publica(self, estado: dict):
        agora = self.get_clock().now().to_msg()

        # Ground truth. Referencial NWU do simulador (o que vai ao ArduPilot
        # é convertido para NED dentro do bridge).
        if 'LocationSensor' in estado:
            odom = Odometry()
            odom.header.stamp = agora
            odom.header.frame_id = 'world_nwu'
            odom.child_frame_id = 'base_link'
            pos = np.asarray(estado['LocationSensor'], dtype=float)
            odom.pose.pose.position.x = float(pos[0])
            odom.pose.pose.position.y = float(pos[1])
            odom.pose.pose.position.z = float(pos[2])
            if 'DynamicsSensor' in estado:
                q = np.asarray(estado['DynamicsSensor'], dtype=float)[-4:]
                odom.pose.pose.orientation.x = float(q[0])
                odom.pose.pose.orientation.y = float(q[1])
                odom.pose.pose.orientation.z = float(q[2])
                odom.pose.pose.orientation.w = float(q[3])
            if 'VelocitySensor' in estado:
                v = np.asarray(estado['VelocitySensor'], dtype=float)
                odom.twist.twist.linear.x = float(v[0])
                odom.twist.twist.linear.y = float(v[1])
                odom.twist.twist.linear.z = float(v[2])
            self.pub_odom.publish(odom)

        if 'VelocitySensor' in estado:
            tw = TwistStamped()
            tw.header.stamp = agora
            tw.header.frame_id = 'world_nwu'
            v = np.asarray(estado['VelocitySensor'], dtype=float)
            tw.twist.linear.x = float(v[0])
            tw.twist.linear.y = float(v[1])
            tw.twist.linear.z = float(v[2])
            self.pub_vel.publish(tw)

        if self.cv is None:
            return

        if 'ImagingSonar' in estado:
            sonar = np.ascontiguousarray(
                np.asarray(estado['ImagingSonar'], dtype=np.float32))
            msg = self.cv.cv2_to_imgmsg(sonar, encoding='32FC1')
            msg.header.stamp = agora
            msg.header.frame_id = 'sonar_link'
            self.pub_sonar.publish(msg)

        if 'RGBCamera' in estado:
            rgba = np.asarray(estado['RGBCamera'], dtype=np.uint8)
            bgr = np.ascontiguousarray(rgba[:, :, [2, 1, 0]])   # RGBA -> BGR
            msg = self.cv.cv2_to_imgmsg(bgr, encoding='bgr8')
            msg.header.stamp = agora
            msg.header.frame_id = 'camera_link'
            self.pub_cam.publish(msg)

    def publica_status(self):
        msg = String()
        estado_ap = 'online' if self.ponte.is_online else 'aguardando ArduPilot'
        msg.data = (f'{estado_ap} | frames={self.frames} '
                    f'| sim_t={self.sim_time:.1f}s')
        self.pub_status.publish(msg)
        self.get_logger().info(msg.data)

    def encerra(self):
        self.ponte.close()
        try:
            self.env.__exit__(None, None, None)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BiguaSimSITLNode()
    node.inicia()

    # O laço é ditado pelo UDP do ArduPilot, não por um timer ROS.
    # spin_once com timeout 0 processa os callbacks sem bloquear: travar aqui
    # faz o ArduPilot acusar perda de frames.
    try:
        while rclpy.ok():
            node.passo()
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.sim_time - node._ultimo_status >= node.periodo_status:
                node.publica_status()
                node._ultimo_status = node.sim_time
    except KeyboardInterrupt:
        pass
    finally:
        node.encerra()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
