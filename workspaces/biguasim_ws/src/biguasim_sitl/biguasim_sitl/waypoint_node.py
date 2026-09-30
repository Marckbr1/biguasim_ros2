#!/usr/bin/env python3
"""
Controle por waypoints do BiguaSim SITL via MAVROS.

Os waypoints são declarados em COORDENADAS LOCAIS (metros, referencial NWU do
simulador) e convertidos para lat/lon com a MESMA origem GPS usada pela ponte
(ver frame.pos_nwu_to_ap). Se as origens divergirem, o veículo navega para o
lugar errado.

Dois modos de operação:

  guided  -> envia um alvo por vez em /mavros/setpoint_position/global e avança
             quando o veículo entra na tolerância. Mais confiável no ArduSub.
  mission -> empurra a missão inteira via /mavros/mission/push e muda para AUTO.
             É o que o QGroundControl faz. Missões não são oficialmente
             suportadas no ArduSub: pode não funcionar.

Pré-requisitos: sim_vehicle.py, o nó biguasim_sitl e o MAVROS no ar.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from mavros_msgs.msg import State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear

_EARTH_RADIUS_M = 6_371_000.0

# Constantes MAVLink
MAV_FRAME_GLOBAL_REL_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16


def local_nwu_para_global(x, y, z, lat0, lon0):
    """[x=Norte, y=Oeste, z=Cima] em metros -> (lat, lon, alt).

    Réplica de biguasim.ardubridge.frame.pos_nwu_to_ap: manter as duas
    consistentes, senão o alvo enviado não corresponde à posição reportada.
    """
    east = -y                       # NWU y=Oeste -> NED y=Leste
    lat = lat0 + math.degrees(x / _EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (_EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon, z


def distancia_global(lat1, lon1, lat2, lon2, lat0):
    """Distância horizontal aproximada em metros (plano tangente)."""
    dn = math.radians(lat2 - lat1) * _EARTH_RADIUS_M
    de = math.radians(lon2 - lon1) * _EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return math.hypot(dn, de)


class WaypointNode(Node):
    def __init__(self):
        super().__init__('biguasim_waypoints')

        # ---------------- Parâmetros ----------------
        self.declare_parameter('modo', 'guided')          # guided | mission
        self.declare_parameter('gps_lat', 33.810313)      # precisa bater com o
        self.declare_parameter('gps_lon', -118.393867)    # nó biguasim_sitl
        # Waypoints locais achatados: [x1,y1,z1, x2,y2,z2, ...] em metros NWU.
        self.declare_parameter('waypoints', [
            10.0, 0.0, -3.0,
            10.0, 10.0, -3.0,
            0.0, 10.0, -3.0,
            0.0, 0.0, -3.0,
        ])
        self.declare_parameter('tolerancia', 2.0)         # metros
        self.declare_parameter('repetir', False)
        self.declare_parameter('armar', True)
        self.declare_parameter('taxa_envio', 5.0)         # Hz dos setpoints
        self.declare_parameter('espera_mavros', 30.0)     # s

        p = self.get_parameter
        self.modo = p('modo').value
        self.lat0 = p('gps_lat').value
        self.lon0 = p('gps_lon').value
        self.tol = p('tolerancia').value
        self.repetir = p('repetir').value

        plano = list(p('waypoints').value)
        if len(plano) % 3 != 0 or not plano:
            raise ValueError('waypoints deve ter múltiplos de 3 valores: x,y,z por ponto')
        self.wps_locais = [tuple(plano[i:i + 3]) for i in range(0, len(plano), 3)]
        self.wps_globais = [
            local_nwu_para_global(x, y, z, self.lat0, self.lon0)
            for (x, y, z) in self.wps_locais
        ]

        # ---------------- Estado ----------------
        self.estado = State()
        self.fix = None
        self.idx = 0
        self.missao_iniciada = False

        # MAVROS publica state com QoS best-effort + transient local.
        qos_estado = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(State, '/mavros/state', self.cb_state, qos_estado)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.cb_fix, qos_estado)
        self.create_subscription(
            WaypointReached, '/mavros/mission/reached', self.cb_reached, 10)

        self.pub_alvo = self.create_publisher(
            GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        self.pub_status = self.create_publisher(String, '~/status', 10)

        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_modo = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_push = self.create_client(WaypointPush, '/mavros/mission/push')
        self.cli_clear = self.create_client(WaypointClear, '/mavros/mission/clear')

        self.get_logger().info(
            f'modo={self.modo} | {len(self.wps_locais)} waypoints | '
            f'tolerância={self.tol} m | origem ({self.lat0:.6f}, {self.lon0:.6f})')
        for i, ((x, y, z), (la, lo, al)) in enumerate(
                zip(self.wps_locais, self.wps_globais)):
            self.get_logger().info(
                f'  WP{i}: local({x:.1f}, {y:.1f}, {z:.1f}) -> '
                f'({la:.6f}, {lo:.6f}, {al:.1f})')

        self.create_timer(1.0 / p('taxa_envio').value, self.ciclo)
        self.create_timer(2.0, self.publica_status)
        self._t0 = self.get_clock().now()
        self._espera = p('espera_mavros').value
        self._avisou_sem_conexao = False

    # ---------------- Callbacks ----------------
    def cb_state(self, msg: State):
        self.estado = msg

    def cb_fix(self, msg: NavSatFix):
        self.fix = msg

    def cb_reached(self, msg: WaypointReached):
        self.get_logger().info(f'missão: waypoint {msg.wp_seq} alcançado')

    # ---------------- Utilidades ----------------
    def _chama(self, cliente, req, descricao):
        """Chamada de serviço não bloqueante: dispara e loga o resultado."""
        if not cliente.service_is_ready():
            self.get_logger().warn(f'{descricao}: serviço indisponível')
            return None
        fut = cliente.call_async(req)
        fut.add_done_callback(
            lambda f: self.get_logger().info(f'{descricao}: {f.result()}'))
        return fut

    def _garante_pronto(self) -> bool:
        """Conexão com a FCU, modo correto e veículo armado."""
        if not self.estado.connected:
            decorrido = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            if decorrido > self._espera and not self._avisou_sem_conexao:
                self.get_logger().error(
                    'MAVROS sem conexão com a FCU. Verifique se o sim_vehicle.py '
                    'está publicando na porta configurada em fcu_url.')
                self._avisou_sem_conexao = True
            return False

        alvo = 'GUIDED' if self.modo == 'guided' else 'AUTO'
        if self.estado.mode != alvo:
            req = SetMode.Request()
            req.custom_mode = alvo
            self._chama(self.cli_modo, req, f'set_mode {alvo}')
            return False

        if self.get_parameter('armar').value and not self.estado.armed:
            req = CommandBool.Request()
            req.value = True
            self._chama(self.cli_arm, req, 'arming')
            return False

        return True

    # ---------------- Modo guided ----------------
    def _passo_guided(self):
        if self.idx >= len(self.wps_globais):
            if self.repetir:
                self.idx = 0
            else:
                return

        lat, lon, alt = self.wps_globais[self.idx]

        alvo = GeoPoseStamped()
        alvo.header.stamp = self.get_clock().now().to_msg()
        alvo.header.frame_id = 'map'
        alvo.pose.position.latitude = lat
        alvo.pose.position.longitude = lon
        alvo.pose.position.altitude = alt
        alvo.pose.orientation.w = 1.0
        self.pub_alvo.publish(alvo)

        if self.fix is None:
            return
        d = distancia_global(self.fix.latitude, self.fix.longitude, lat, lon, self.lat0)
        if d < self.tol:
            self.get_logger().info(f'WP{self.idx} alcançado ({d:.2f} m)')
            self.idx += 1

    # ---------------- Modo mission ----------------
    def _passo_mission(self):
        if self.missao_iniciada:
            return

        wps = []
        for i, (lat, lon, alt) in enumerate(self.wps_globais):
            wp = Waypoint()
            wp.frame = MAV_FRAME_GLOBAL_REL_ALT
            wp.command = MAV_CMD_NAV_WAYPOINT
            wp.is_current = (i == 0)
            wp.autocontinue = True
            wp.param1 = 0.0          # tempo de espera no ponto
            wp.param2 = self.tol     # raio de aceitação
            wp.param3 = 0.0
            wp.param4 = float('nan')  # yaw livre
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            wps.append(wp)

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = wps
        self._chama(self.cli_push, req, f'push de {len(wps)} waypoints')
        self.missao_iniciada = True
        self.get_logger().info(
            'Missão enviada. O ArduSub não suporta missões oficialmente: '
            'se não houver movimento, use modo:=guided.')

    # ---------------- Laço ----------------
    def ciclo(self):
        if not self._garante_pronto():
            return
        if self.modo == 'guided':
            self._passo_guided()
        else:
            self._passo_mission()

    def publica_status(self):
        if self.modo == 'guided':
            alvo = (f'WP{self.idx}/{len(self.wps_globais)}'
                    if self.idx < len(self.wps_globais) else 'concluído')
        else:
            alvo = 'missão enviada' if self.missao_iniciada else 'preparando'
        msg = String()
        msg.data = (f'conectado={self.estado.connected} armado={self.estado.armed} '
                    f'modo={self.estado.mode} alvo={alvo}')
        self.pub_status.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
