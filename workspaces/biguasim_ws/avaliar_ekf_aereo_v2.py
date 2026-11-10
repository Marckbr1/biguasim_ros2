#!/usr/bin/env python3
"""
ESKF Aéreo (IMU + VIO) com avaliação realista de deriva.

Mudanças em relação à versão anterior (EKF 9 estados):
  1. Error-State EKF de 15 estados: [p, v, q, b_a, b_g]. A atitude é propagada
     pelo GIROSCÓPIO (antes o giroscópio não era usado e a atitude vinha do campo
     `orientation` do IMU do simulador, que é praticamente o Ground Truth).
  2. Jacobiana F completa (acoplamento velocidade-atitude e bias), forma de Joseph.
  3. IMU corrompido com bias + passeio aleatório + ruído branco (perfil MEMS).
  4. VIO sintético REALISTA: 20 Hz, latência, deriva em passeio aleatório na
     posição e no yaw, erro de escala, quedas e outliers.
  5. Compensação de atraso: buffer de estados + repropagação.
  6. Gating por NIS (teste chi²) para rejeitar outliers.
  7. Métricas: ATE, RPE (deriva por distância), NEES, NIS; exportação TUM p/ `evo`.
  8. Diagnóstico de eixos do IMU contra o GT (o mesmo tipo de bug do DVL).

Uso:
  python3 eskf_aereo_realista.py --bag bag_uav_aereo_completo --diagnostico
  python3 eskf_aereo_realista.py --bag bag_uav_aereo_completo
  python3 eskf_aereo_realista.py --bag bag_uav_aereo_completo --vio-ideal   # armadilha antiga
  python3 eskf_aereo_realista.py --sintetico                                 # sem bag, autoteste
"""

import copy
import math
import argparse
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

G = 9.80665
G_VEC = np.array([0.0, 0.0, -G])     # gravidade no mundo (ENU, z para cima)
I3 = np.eye(3)
CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345, 6: 16.812}

# Mapeamento de eixos do IMU (corpo do simulador -> corpo FLU do filtro).
# Ajuste com base na saída de --diagnostico, como foi feito no DVL.
# ATENÇÃO: aceleração é vetor; velocidade angular é PSEUDOVETOR. Se M_ACC for uma
# reflexão (det = -1, típico da conversão Unreal left-handed -> ROS right-handed),
# o giro precisa de um sinal extra: M_GYR = det(M_ACC) * M_ACC.
# Ex.: Unreal (X frente, Y direita, Z cima) -> FLU: M_ACC = diag(1,-1,1)
#      => M_GYR = diag(-1, 1, -1)  (roll rate e yaw rate invertem!)
M_ACC = np.eye(3)
M_GYR = np.linalg.det(M_ACC) * M_ACC
# Em que referencial o simulador publica a aceleração do IMU?
#   'corpo'   -> correto (IMU real sempre mede no corpo)
#   'mundo'   -> simulador publica no mundo; convertemos com a atitude do GT
#   'heading' -> alinhado à gravidade, girando só com o yaw
# Escolha a hipótese de menor resíduo no --diagnostico.
ACC_REFERENCIAL = 'corpo'
# Fator de correção da base de tempo: t' = t0 + ESCALA_TEMPO·(t - t0), aplicado a IMU e GT.
# Se o simulador roda k× mais rápido que o tempo real e carimba com o relógio de parede,
# o --diagnostico mostra ganho do giro ≈ 1/k em TODOS os eixos. Use ESCALA_TEMPO = k.
ESCALA_TEMPO = 1.0
# Rejeições consecutivas do VIO antes de resetar posição/velocidade (anti-travamento)
MAX_REJEICOES_SEGUIDAS = 10


# =============================================================================
# Quaternions [w, x, y, z], rotação corpo -> mundo
# =============================================================================
def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def q_mult(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def q_exp(rv):
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        q = np.array([1.0, 0.5*rv[0], 0.5*rv[1], 0.5*rv[2]])
        return q / np.linalg.norm(q)
    ax = rv / ang
    return np.r_[math.cos(ang/2), math.sin(ang/2)*ax]


def q_log(q):
    q = q if q[0] >= 0 else -q
    nv = np.linalg.norm(q[1:])
    if nv < 1e-12:
        return 2.0 * q[1:]
    return 2.0 * math.atan2(nv, q[0]) * q[1:] / nv


def q_to_R(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def R_to_q(R):
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = [0.25*s, (R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1]-R[1, 2])/s, 0.25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2]-R[2, 0])/s, (R[0, 1]+R[1, 0])/s, 0.25*s, (R[1, 2]+R[2, 1])/s]
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0]-R[0, 1])/s, (R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, 0.25*s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def yaw_de_q(q):
    w, x, y, z = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def Rz(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# =============================================================================
# Configurações
# =============================================================================
@dataclass
class CfgIMUReal:
    """Erros injetados no IMU do simulador (perfil MEMS de drone, ~EuRoC x2)."""
    n_a: float = 4e-3      # densidade ruído acel.   [m/s²/√Hz]
    n_g: float = 3e-4      # densidade ruído giro    [rad/s/√Hz]
    n_ba: float = 6e-4     # passeio aleat. bias acel [m/s³/√Hz]
    n_bg: float = 4e-5     # passeio aleat. bias giro [rad/s²/√Hz]
    ba0: float = 0.08      # bias inicial acel (1σ)  [m/s²]
    bg0: float = 0.004     # bias inicial giro (1σ)  [rad/s]


@dataclass
class CfgFiltro:
    """O que o filtro ASSUME (inflado em relação ao real, prática padrão)."""
    n_a: float = 8e-3
    n_g: float = 6e-4
    n_ba: float = 1.2e-3
    n_bg: float = 8e-5
    p0: float = 0.05
    v0: float = 0.05
    att0: float = math.radians(2)
    ba0: float = 0.15
    bg0: float = 0.008
    # Ruído assumido do VIO. Desvio padrão (σ), NÃO variância.
    # Inflado de propósito: a deriva do VIO é correlacionada no tempo.
    sigma_vio_pos: float = 0.15
    sigma_vio_vel: float = 0.08
    sigma_baro: float = 0.25
    sigma_mag: float = math.radians(4)
    gating: bool = True


@dataclass
class CfgVIO:
    taxa_hz: float = 20.0
    latencia_s: float = 0.08
    jitter_s: float = 0.01
    sigma_pos: float = 0.03          # ruído branco de posição [m]
    sigma_vel: float = 0.03          # ruído branco de velocidade [m/s]
    rw_pos: float = 0.02             # deriva de posição [m/√s]
    rw_yaw: float = math.radians(0.15)   # deriva de yaw [rad/√s]
    erro_escala: float = 0.01        # 1 % de erro de escala
    prob_queda: float = 0.02
    janelas_perda: list = field(default_factory=lambda: [(55.0, 58.0)])
    prob_outlier: float = 0.004
    tam_outlier: float = 1.5         # [m]


def cfg_vio_ideal():
    """Reproduz a 'armadilha' anterior: ruído branco ancorado no GT, sem atraso."""
    return CfgVIO(latencia_s=0.0, jitter_s=0.0, sigma_pos=0.10, sigma_vel=0.02,
                  rw_pos=0.0, rw_yaw=0.0, erro_escala=0.0, prob_queda=0.0,
                  janelas_perda=[], prob_outlier=0.0)


# =============================================================================
# ESKF 15 estados: δx = [δp, δv, δθ, δb_a, δb_g]
# =============================================================================
class ESKF:
    def __init__(self, p0, v0, q0, cfg: CfgFiltro):
        self.cfg = cfg
        self.p, self.v = p0.astype(float).copy(), v0.astype(float).copy()
        self.q = q0 / np.linalg.norm(q0)
        self.ba, self.bg = np.zeros(3), np.zeros(3)
        self.P = np.diag(np.r_[[cfg.p0**2]*3, [cfg.v0**2]*3, [cfg.att0**2]*3,
                               [cfg.ba0**2]*3, [cfg.bg0**2]*3])

    def snapshot(self):
        return (self.p.copy(), self.v.copy(), self.q.copy(),
                self.ba.copy(), self.bg.copy(), self.P.copy())

    def restaurar(self, s):
        self.p, self.v, self.q, self.ba, self.bg, self.P = [x.copy() for x in s]

    def propagar(self, am, wm, dt):
        c = self.cfg
        R = q_to_R(self.q)
        a_b = am - self.ba
        w_b = wm - self.bg
        a_w = R @ a_b + G_VEC

        self.p = self.p + self.v*dt + 0.5*a_w*dt*dt
        self.v = self.v + a_w*dt
        dq = q_exp(w_b*dt)
        self.q = q_mult(self.q, dq)
        self.q /= np.linalg.norm(self.q)

        F = np.eye(15)
        F[0:3, 3:6] = I3*dt
        F[3:6, 6:9] = -R @ skew(a_b) * dt
        F[3:6, 9:12] = -R*dt
        F[6:9, 6:9] = q_to_R(dq).T
        F[6:9, 12:15] = -I3*dt

        Qd = np.zeros((15, 15))
        Qd[3:6, 3:6] = I3 * c.n_a**2 * dt
        Qd[6:9, 6:9] = I3 * c.n_g**2 * dt
        Qd[9:12, 9:12] = I3 * c.n_ba**2 * dt
        Qd[12:15, 12:15] = I3 * c.n_bg**2 * dt

        self.P = F @ self.P @ F.T + Qd
        self.P = 0.5*(self.P + self.P.T)

    def atualizar(self, r, H, Rm):
        S = H @ self.P @ H.T + Rm
        S_inv = np.linalg.inv(S)
        nis = float(r @ S_inv @ r)
        if self.cfg.gating and nis > CHI2_99[len(r)]:
            return False, nis

        K = self.P @ H.T @ S_inv
        dx = K @ r
        IKH = np.eye(15) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T     # forma de Joseph

        self.p += dx[0:3]
        self.v += dx[3:6]
        self.q = q_mult(self.q, q_exp(dx[6:9]))
        self.q /= np.linalg.norm(self.q)
        self.ba += dx[9:12]
        self.bg += dx[12:15]

        Gr = np.eye(15)                                   # reset do erro
        Gr[6:9, 6:9] = I3 - skew(0.5*dx[6:9])
        self.P = Gr @ self.P @ Gr.T
        self.P = 0.5*(self.P + self.P.T)
        return True, nis

    def reset_pos_vel(self, zp, zv):
        """Reinicializa p e v a partir do VIO e infla a incerteza (anti-travamento)."""
        c = self.cfg
        self.p, self.v = zp.copy(), zv.copy()
        self.P[0:6, :] = 0.0
        self.P[:, 0:6] = 0.0
        self.P[0:3, 0:3] = I3 * c.sigma_vio_pos**2
        self.P[3:6, 3:6] = I3 * c.sigma_vio_vel**2
        self.P[6:9, 6:9] += I3 * math.radians(10)**2   # atitude provavelmente ruim também
        self.P[9:12, 9:12] += I3 * c.ba0**2
        return True, 0.0

    def H_yaw(self):
        H = np.zeros((1, 15))
        y0 = yaw_de_q(self.q)
        for i in range(3):
            d = np.zeros(3)
            d[i] = 1e-6
            H[0, 6+i] = wrap(yaw_de_q(q_mult(self.q, q_exp(d))) - y0) / 1e-6
        return H


# ---- Modelos de medição -----------------------------------------------------
def upd_vio(f: ESKF, zp, zv, modo):
    c = f.cfg
    if modo == 'pose_vel':
        H = np.zeros((6, 15))
        H[0:3, 0:3] = I3
        H[3:6, 3:6] = I3
        r = np.r_[zp - f.p, zv - f.v]
        Rm = np.diag([c.sigma_vio_pos**2]*3 + [c.sigma_vio_vel**2]*3)
    else:  # 'vel'
        H = np.zeros((3, 15))
        H[:, 3:6] = I3
        r = zv - f.v
        Rm = I3 * c.sigma_vio_vel**2
    return f.atualizar(r, H, Rm)


def upd_baro(f: ESKF, z):
    H = np.zeros((1, 15))
    H[0, 2] = 1.0
    return f.atualizar(np.array([z - f.p[2]]), H, np.array([[f.cfg.sigma_baro**2]]))


def upd_mag(f: ESKF, yaw_med):
    r = np.array([wrap(yaw_med - yaw_de_q(f.q))])
    return f.atualizar(r, f.H_yaw(), np.array([[f.cfg.sigma_mag**2]]))


# =============================================================================
# Compensação de atraso: buffer de estados + repropagação
# =============================================================================
class FiltroComAtraso:
    def __init__(self, f: ESKF, janela_s=1.0, compensar=True):
        self.f = f
        self.janela = janela_s
        self.compensar = compensar
        self.hist = deque()          # (t, am, wm, snapshot após propagar)
        self.t_ult = None

    def imu(self, t, am, wm):
        if self.t_ult is not None:
            dt = t - self.t_ult
            if dt <= 0.0:
                return
            if dt > 0.5:             # buraco no log: não integra
                self.t_ult = t
                self.hist.clear()
                return
            self.f.propagar(am, wm, dt)
        self.t_ult = t
        self.hist.append((t, am, wm, self.f.snapshot()))
        while self.hist and t - self.hist[0][0] > self.janela:
            self.hist.popleft()

    def medida(self, t_med, aplicar):
        if not self.compensar:
            return aplicar(self.f)           # aplica dado velho como se fosse atual

        idx = None
        for i in range(len(self.hist) - 1, -1, -1):
            if self.hist[i][0] <= t_med:
                idx = i
                break
        if idx is None:
            return None                      # mais velha que o buffer: descarta

        self.f.restaurar(self.hist[idx][3])
        res = aplicar(self.f)
        t_i, am_i, wm_i, _ = self.hist[idx]
        self.hist[idx] = (t_i, am_i, wm_i, self.f.snapshot())
        for j in range(idx + 1, len(self.hist)):
            t_j, am_j, wm_j, _ = self.hist[j]
            self.f.propagar(am_j, wm_j, t_j - self.hist[j-1][0])
            self.hist[j] = (t_j, am_j, wm_j, self.f.snapshot())
        return res


# =============================================================================
# Sensores sintéticos
# =============================================================================
class CorruptorIMU:
    def __init__(self, cfg: CfgIMUReal, rng):
        self.c, self.rng = cfg, rng
        self.ba = rng.normal(0, cfg.ba0, 3)
        self.bg = rng.normal(0, cfg.bg0, 3)

    def aplicar(self, am, wm, dt):
        c, rng = self.c, self.rng
        self.ba = self.ba + rng.normal(0, c.n_ba*math.sqrt(dt), 3)
        self.bg = self.bg + rng.normal(0, c.n_bg*math.sqrt(dt), 3)
        am_c = am + self.ba + rng.normal(0, c.n_a/math.sqrt(dt), 3)
        wm_c = wm + self.bg + rng.normal(0, c.n_g/math.sqrt(dt), 3)
        return am_c, wm_c


def gerar_vio(cfg: CfgVIO, gt_t, gt_p, gt_v, rng):
    """Retorna lista de (t_carimbo, t_chegada, zp, zv) + deriva verdadeira."""
    ts = np.arange(gt_t[0] + 1.0/cfg.taxa_hz, gt_t[-1], 1.0/cfg.taxa_hz)
    dt = 1.0 / cfg.taxa_hz
    p0 = gt_p[0]
    d_pos, d_yaw = np.zeros(3), 0.0
    s = 1.0 + cfg.erro_escala
    medidas = []
    for t in ts:
        d_pos = d_pos + rng.normal(0, cfg.rw_pos*math.sqrt(dt), 3)
        d_yaw = d_yaw + rng.normal(0, cfg.rw_yaw*math.sqrt(dt))
        p = np.array([np.interp(t, gt_t, gt_p[:, k]) for k in range(3)])
        v = np.array([np.interp(t, gt_t, gt_v[:, k]) for k in range(3)])
        if any(a <= t <= b for a, b in cfg.janelas_perda) or rng.random() < cfg.prob_queda:
            continue
        Rd = Rz(d_yaw)
        zp = p0 + s*(Rd @ (p - p0)) + d_pos + rng.normal(0, cfg.sigma_pos, 3)
        zv = s*(Rd @ v) + rng.normal(0, cfg.sigma_vel, 3)
        if rng.random() < cfg.prob_outlier:
            zp = zp + rng.normal(0, cfg.tam_outlier, 3)
        t_cheg = t + cfg.latencia_s + abs(rng.normal(0, cfg.jitter_s))
        medidas.append((t, t_cheg, zp, zv))
    return medidas


def gerar_baro(gt_t, gt_p, rng, taxa=25.0, sigma=0.2, rw_bias=0.01):
    ts = np.arange(gt_t[0], gt_t[-1], 1.0/taxa)
    b = 0.0
    out = []
    for t in ts:
        b += rng.normal(0, rw_bias*math.sqrt(1.0/taxa))
        out.append((t, t, np.interp(t, gt_t, gt_p[:, 2]) + b + rng.normal(0, sigma)))
    return out


def gerar_mag(gt_t, gt_yaw_unw, rng, taxa=10.0, sigma=math.radians(3)):
    ts = np.arange(gt_t[0], gt_t[-1], 1.0/taxa)
    return [(t, t, wrap(np.interp(t, gt_t, gt_yaw_unw) + rng.normal(0, sigma))) for t in ts]


# =============================================================================
# Fontes de dados
# =============================================================================
def carregar_bag(bag_path, agent_id='uav0_id0', duracao=100.0):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    odom_topic = f'/biguasim/{agent_id}/DynamicsSensor/Odom'
    imu_topic = f'/biguasim/{agent_id}/IMUSensor'

    imu_t, acc, gyr, gt_t, gt_p, gt_q = [], [], [], [], [], []
    t0 = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in (odom_topic, imu_topic):
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        t0 = t if t0 is None else t0
        if t - t0 > duracao:
            break
        if topic == imu_topic:
            a, w = msg.linear_acceleration, msg.angular_velocity
            imu_t.append(t)
            acc.append(M_ACC @ [a.x, a.y, a.z])
            gyr.append(M_GYR @ [w.x, w.y, w.z])
        else:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            gt_t.append(t)
            gt_p.append([p.x, p.y, p.z])
            gt_q.append([q.w, q.x, q.y, q.z])

    d = dict(imu_t=np.array(imu_t), imu_acc=np.array(acc), imu_gyr=np.array(gyr),
             gt_t=np.array(gt_t), gt_p=np.array(gt_p), gt_q=np.array(gt_q))
    for pre in ('imu', 'gt'):                      # garante ordem temporal
        o = np.argsort(d[f'{pre}_t'])
        for k in list(d):
            if k.startswith(pre):
                d[k] = d[k][o]
    return d


def gerar_dados_sinteticos(duracao=100.0, f_imu=200.0, f_gt=50.0):
    """Trajetória em '8' com dinâmica de quadrirrotor (para autoteste sem bag)."""
    w = 2*np.pi/40.0

    def pos(t): return np.array([15*np.sin(w*t), 8*np.sin(2*w*t), 5 + 1.5*np.sin(0.5*w*t)])
    def vel(t): return np.array([15*w*np.cos(w*t), 16*w*np.cos(2*w*t), 0.75*w*np.cos(0.5*w*t)])
    def acc(t): return np.array([-15*w*w*np.sin(w*t), -32*w*w*np.sin(2*w*t), -0.375*w*w*np.sin(0.5*w*t)])

    def rot(t):
        zb = acc(t) - G_VEC
        zb /= np.linalg.norm(zb)
        v = vel(t)
        psi = math.atan2(v[1], v[0])
        xc = np.array([math.cos(psi), math.sin(psi), 0.0])
        yb = np.cross(zb, xc)
        yb /= np.linalg.norm(yb)
        return np.column_stack([np.cross(yb, zb), yb, zb])

    imu_t = np.arange(0, duracao, 1.0/f_imu)
    h = 1e-4
    acc_m, gyr_m = [], []
    for t in imu_t:
        R = rot(t)
        acc_m.append(R.T @ (acc(t) - G_VEC))
        gyr_m.append(q_log(R_to_q(rot(t-h).T @ rot(t+h))) / (2*h))
    gt_t = np.arange(0, duracao, 1.0/f_gt)
    return dict(imu_t=imu_t, imu_acc=np.array(acc_m), imu_gyr=np.array(gyr_m),
                gt_t=gt_t, gt_p=np.array([pos(t) for t in gt_t]),
                gt_q=np.array([R_to_q(rot(t)) for t in gt_t]))


def q_interp(tq, t, qs):
    """Interpolação (nlerp) de quaternions do GT nos instantes tq."""
    idx = np.clip(np.searchsorted(t, tq), 1, len(t) - 1)
    out = np.empty((len(tq), 4))
    for n, (ti, i) in enumerate(zip(tq, idx)):
        q0, q1 = qs[i-1], qs[i]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        u = np.clip((ti - t[i-1]) / max(t[i] - t[i-1], 1e-9), 0.0, 1.0)
        q = (1 - u)*q0 + u*q1
        out[n] = q / np.linalg.norm(q)
    return out


def corrigir_referencial_acc(d, modo):
    """Leva a aceleração publicada pelo simulador para o referencial do corpo.

    Usa a atitude do GT: isso é legítimo aqui porque estamos CORRIGINDO O SIMULADOR
    para se comportar como um IMU real (que mede no corpo). No hardware isso não existe.
    """
    if modo == 'corpo':
        return d
    qi = q_interp(d['imu_t'], d['gt_t'], d['gt_q'])
    acc = np.empty_like(d['imu_acc'])
    for n, (q, a) in enumerate(zip(qi, d['imu_acc'])):
        R = q_to_R(q)
        if modo == 'mundo':
            acc[n] = R.T @ a
        elif modo == 'heading':
            acc[n] = R.T @ (Rz(yaw_de_q(q)) @ a)
        else:
            raise ValueError(modo)
    d['imu_acc'] = acc
    print(f"[*] Aceleração convertida do referencial '{modo}' para o corpo (via atitude do GT).")
    return d


def corrigir_base_tempo(d, k):
    if abs(k - 1.0) < 1e-9:
        return d
    t0 = min(d['imu_t'][0], d['gt_t'][0])
    d['imu_t'] = t0 + k * (d['imu_t'] - t0)
    d['gt_t'] = t0 + k * (d['gt_t'] - t0)
    print(f"[*] Base de tempo corrigida: carimbos multiplicados por {k:.3f}.")
    return d


def velocidade_gt(gt_t, gt_p):
    return np.column_stack([np.gradient(gt_p[:, k], gt_t) for k in range(3)])


# =============================================================================
# Diagnóstico de eixos do IMU contra o Ground Truth
# =============================================================================
def diagnosticar_eixos(d, janela=5):
    t, p, qs = d['gt_t'], d['gt_p'], d['gt_q']
    ker = np.ones(janela)/janela
    ps = np.column_stack([np.convolve(p[:, k], ker, mode='same') for k in range(3)])
    v = velocidade_gt(t, ps)
    a = velocidade_gt(t, v)

    f_pred, w_pred = [], []
    for k in range(1, len(t)-1):
        R = q_to_R(qs[k])
        f_pred.append(R.T @ (a[k] - G_VEC))
        dq = q_mult(np.r_[qs[k-1][0], -qs[k-1][1:]], qs[k+1])
        w_pred.append(q_log(dq) / (t[k+1] - t[k-1]))
    f_pred, w_pred = np.array(f_pred), np.array(w_pred)
    tc = t[1:-1]
    sl = slice(janela, len(tc)-janela)

    am = np.column_stack([np.interp(tc, d['imu_t'], d['imu_acc'][:, k]) for k in range(3)])
    wm = np.column_stack([np.interp(tc, d['imu_t'], d['imu_gyr'][:, k]) for k in range(3)])

    def corr(A, B):
        C = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                a_, b_ = A[sl, i], B[sl, j]
                if a_.std() < 1e-9 or b_.std() < 1e-9:
                    C[i, j] = np.nan
                else:
                    C[i, j] = np.corrcoef(a_, b_)[0, 1]
        return C

    np.set_printoptions(precision=2, suppress=True)
    print("\n=== DIAGNÓSTICO DE EIXOS DO IMU ===")
    print(f"Média acel. medida (corpo): {am[sl].mean(0)}  | prevista pelo GT: {f_pred[sl].mean(0)}")
    print("  -> Em voo nivelado, o eixo z deve ficar perto de +9.81 nos dois.")
    print("Correlação acel. medida (linhas) x prevista pelo GT (colunas):")
    print(corr(am, f_pred))
    print(f"RMS do resíduo de acel.: {np.sqrt(np.mean((am[sl]-f_pred[sl])**2, axis=0))} m/s²")
    print("Correlação giro medido (linhas) x ω_corpo do GT (colunas):")
    print(corr(wm, w_pred))
    print(f"RMS do resíduo do giro: {np.sqrt(np.mean((wm[sl]-w_pred[sl])**2, axis=0))} rad/s")
    print("Leitura: diagonal ≈ +1 => eixos corretos. Valor ≈ -1 ou fora da diagonal =>")
    print("permutação/inversão (como no DVL).")

    # ---- Ganho do giro por eixo (erro de escala, unidade ou BASE DE TEMPO) ----
    ganho = np.array([float(np.dot(wm[sl, j], w_pred[sl, j]) /
                            max(np.dot(w_pred[sl, j], w_pred[sl, j]), 1e-12)) for j in range(3)])
    print(f"Ganho estimado do giro por eixo (ideal 1.0): {np.round(ganho, 3)}")
    k_tempo = 1.0
    if abs(ganho.mean() - 1) > 0.05 and ganho.std() < 0.05 * abs(ganho.mean()):
        k_tempo = 1.0 / ganho.mean()
        print(f">>> Ganho UNIFORME nos 3 eixos: assinatura de base de tempo errada.")
        print(f"    Carimbos andam {k_tempo:.3f}× mais devagar que o tempo físico.")
        print(f"    Sugestão: ESCALA_TEMPO = {ESCALA_TEMPO * k_tempo:.3f}  (valor absoluto, já inclui o atual)")
    print(f"Período do IMU: {np.median(np.diff(d['imu_t'])) * 1e3:.1f} ms "
          f"({1/np.median(np.diff(d['imu_t'])):.0f} Hz)")

    def testar_hipoteses(esc_a, titulo):
        hip = {}
        for k in range(len(tc)):
            kk = k + 1
            R = q_to_R(qs[kk])
            a_k = a[kk] * esc_a
            fw = a_k - G_VEC
            cand = {
                "corpo, com g      R^T(a-g)  [IMU real]": R.T @ fw,
                "mundo, com g      (a-g)": fw,
                "heading, com g    Rz^T(a-g)": Rz(yaw_de_q(qs[kk])).T @ fw,
                "quat. conjugado   R(a-g)": R @ fw,
                "corpo, sem g      R^T a": R.T @ a_k,
                "mundo, sem g      a": a_k,
            }
            for nome, val in cand.items():
                hip.setdefault(nome, []).append(val)
        print(titulo)
        res_h = []
        for nome, val in hip.items():
            val = np.array(val)
            r = np.sqrt(np.mean((am[sl] - val[sl])**2, axis=0))
            res_h.append((np.linalg.norm(r), nome, r))
        for tot, nome, r in sorted(res_h):
            print(f"  {nome:40s} resíduo {tot:6.2f} m/s²  por eixo {r}")
        return sorted(res_h)[0][1]

    melhor = testar_hipoteses(1.0, "Hipóteses para o referencial da aceleração (menor resíduo = mais provável):")
    if k_tempo != 1.0:
        melhor = testar_hipoteses(1.0 / k_tempo**2,
                                  f"Mesmas hipóteses APÓS corrigir a base de tempo (k = {k_tempo:.3f}):")
    if not melhor.startswith("corpo, com g"):
        print(f">>> O acelerômetro NÃO está no corpo. Melhor hipótese: '{melhor.split('  ')[0].strip()}'.")
        print("    Ajuste ACC_REFERENCIAL (e ESCALA_TEMPO, se sugerido) e rode o diagnóstico de novo.")
    print()

    def sugerir(C, nome):
        C = np.nan_to_num(C)
        M = np.zeros((3, 3))
        for j in range(3):               # para cada eixo do filtro, melhor eixo medido
            i = int(np.argmax(abs(C[:, j])))
            M[j, i] = np.sign(C[i, j]) if abs(C[i, j]) > 0.5 else 0.0
        ok = np.allclose(abs(M).sum(0), 1) and np.allclose(abs(M).sum(1), 1)
        print(f"Sugestão {nome} (correlação > 0.5):\n{M}" + ("" if ok else
              "\n  -> ambígua (eixo sem excitação ou correlação fraca); use o outro sensor + regra det."))
        return M, ok

    Ma, oka = sugerir(corr(am, f_pred), 'M_ACC')
    Mg, okg = sugerir(corr(wm, w_pred), 'M_GYR')
    if okg and not oka:
        # O giro sempre mapeia por matriz própria (det=+1): M_ACC = ±M_GYR.
        # Desempata pelo eixo do acelerômetro que tem excitação (normalmente z, gravidade).
        for cand in (Mg, -Mg):
            if all(np.allclose(cand[j], Ma[j]) for j in range(3) if abs(Ma[j]).sum() > 0):
                print(f"M_ACC inferida pela regra do pseudovetor:\n{cand}")
                Ma, oka = cand, True
                break
    if oka and okg:
        if np.allclose(Mg, np.linalg.det(Ma) * Ma):
            print("Consistente com a regra M_GYR = det(M_ACC)·M_ACC.")
        else:
            print("ALERTA: M_GYR != det(M_ACC)·M_ACC — confira a convenção do simulador.")
    if (oka and not np.allclose(Ma, I3)) or (okg and not np.allclose(Mg, I3)):
        print(">>> Mapeamento atual NÃO é o correto. Atualize M_ACC/M_GYR no topo do arquivo")
        print("    (as sugestões acima são RELATIVAS ao mapeamento já aplicado).")
    print()


# =============================================================================
# Execução
# =============================================================================
def executar(d, args, salvar=True):
    rng = np.random.default_rng(args.seed)
    gt_t, gt_p, gt_q = d['gt_t'], d['gt_p'], d['gt_q']
    gt_v = velocidade_gt(gt_t, gt_p)
    gt_yaw = np.unwrap([yaw_de_q(q) for q in gt_q])

    cfg_f = CfgFiltro(gating=not args.sem_gating)
    cfg_v = cfg_vio_ideal() if args.vio_ideal else CfgVIO()
    if args.vio_ideal:          # filtro "sabe" o ruído exato, como na versão antiga
        cfg_f.sigma_vio_pos, cfg_f.sigma_vio_vel = cfg_v.sigma_pos, cfg_v.sigma_vel

    corr_imu = None if args.sem_corromper_imu else CorruptorIMU(CfgIMUReal(), rng)

    # Inicialização a partir do 1º GT (no hardware: 1ª pose do VIO + alinhamento estático)
    f = ESKF(gt_p[0], gt_v[0], gt_q[0], cfg_f)
    fa = FiltroComAtraso(f, janela_s=1.0, compensar=not args.sem_compensacao)

    # Fila de eventos ordenada pelo instante de CHEGADA
    eventos = [(t, 0, 'imu', i) for i, t in enumerate(d['imu_t']) if t >= gt_t[0]]
    vio = gerar_vio(cfg_v, gt_t, gt_p, gt_v, rng)
    eventos += [(m[1], 1, 'vio', m) for m in vio]
    if args.baro:
        eventos += [(m[1], 1, 'baro', m) for m in gerar_baro(gt_t, gt_p, rng)]
    if args.mag:
        eventos += [(m[1], 1, 'mag', m) for m in gerar_mag(gt_t, gt_yaw, rng)]
    eventos.sort(key=lambda e: (e[0], e[1]))

    log = {k: [] for k in ('t', 'p', 'v', 'q', 'yaw', 'Ppp', 'sy', 'ba', 'bg', 'ba_v', 'bg_v')}
    nis_log = []
    resets = []
    rej_seguidas = 0
    t_ult_imu = None
    n_imu = 0

    print(f"[*] VIO: {'IDEAL (armadilha)' if args.vio_ideal else 'REALISTA'} | modo={args.modo_vio} | "
          f"latência comp.={'sim' if fa.compensar else 'não'} | baro={args.baro} | mag={args.mag}")

    for t_ev, _, tipo, dado in eventos:
        if tipo == 'imu':
            am, wm = d['imu_acc'][dado].copy(), d['imu_gyr'][dado].copy()
            if corr_imu is not None:
                dt = (t_ev - t_ult_imu) if t_ult_imu is not None else 0.005
                am, wm = corr_imu.aplicar(am, wm, max(dt, 1e-4))
            t_ult_imu = t_ev
            fa.imu(t_ev, am, wm)
            n_imu += 1
            if n_imu % 4 == 0:      # log do estado em TEMPO REAL (o que o controle veria)
                log['t'].append(t_ev); log['p'].append(f.p.copy()); log['v'].append(f.v.copy())
                log['q'].append(f.q.copy()); log['yaw'].append(yaw_de_q(f.q))
                log['Ppp'].append(f.P[0:3, 0:3].copy())
                Hy = f.H_yaw()
                log['sy'].append(math.sqrt((Hy @ f.P @ Hy.T)[0, 0]))
                log['ba'].append(f.ba.copy()); log['bg'].append(f.bg.copy())
                if corr_imu is not None:
                    log['ba_v'].append(corr_imu.ba.copy()); log['bg_v'].append(corr_imu.bg.copy())
        elif tipo == 'vio':
            t_med, _, zp, zv = dado
            res = fa.medida(t_med, lambda ff: upd_vio(ff, zp, zv, args.modo_vio))
            if res is not None:
                nis_log.append((t_med, res[1], 6 if args.modo_vio == 'pose_vel' else 3, res[0]))
                rej_seguidas = 0 if res[0] else rej_seguidas + 1
                if rej_seguidas >= MAX_REJEICOES_SEGUIDAS and args.modo_vio == 'pose_vel':
                    fa.medida(t_med, lambda ff: ff.reset_pos_vel(zp, zv))
                    resets.append(t_med)
                    rej_seguidas = 0
        elif tipo == 'baro':
            fa.medida(dado[0], lambda ff: upd_baro(ff, dado[2]))
        elif tipo == 'mag':
            fa.medida(dado[0], lambda ff: upd_mag(ff, dado[2]))

    return avaliar(d, log, nis_log, vio, gt_v, gt_yaw, args, resets, salvar)


def avaliar(d, log, nis_log, vio, gt_v, gt_yaw, args, resets=(), salvar=True):
    gt_t, gt_p = d['gt_t'], d['gt_p']
    t = np.array(log['t'])
    sel = (t >= gt_t[0]) & (t <= gt_t[-1])
    t = t[sel]
    est = np.array(log['p'])[sel]
    Ppp = np.array(log['Ppp'])[sel]
    yaw_est = np.unwrap(np.array(log['yaw'])[sel])
    sy = np.array(log['sy'])[sel]
    gt = np.column_stack([np.interp(t, gt_t, gt_p[:, k]) for k in range(3)])
    gy = np.interp(t, gt_t, gt_yaw)

    err = est - gt
    en = np.linalg.norm(err, axis=1)
    ate = math.sqrt(np.mean(en**2))
    nees = np.array([e @ np.linalg.solve(P, e) for e, P in zip(err, Ppp)])
    err_yaw = np.array([wrap(a - b) for a, b in zip(yaw_est, gy)])

    # VIO bruto (para comparar)
    tv = np.array([m[0] for m in vio])
    pv = np.array([m[2] for m in vio])
    gv = np.column_stack([np.interp(tv, gt_t, gt_p[:, k]) for k in range(3)])
    ate_vio = math.sqrt(np.mean(np.sum((pv - gv)**2, axis=1)))

    # RPE translacional em janelas de Δ segundos
    delta = 10.0
    passo = max(1, int(round(delta / np.median(np.diff(t)))))
    rpe, dist = [], []
    for i in range(0, len(t) - passo, max(1, passo // 5)):
        j = i + passo
        rpe.append(np.linalg.norm((est[j]-est[i]) - (gt[j]-gt[i])))
        dist.append(np.sum(np.linalg.norm(np.diff(gt[i:j+1], axis=0), axis=1)))
    rpe, dist = np.array(rpe), np.array(dist)

    nis = np.array([n[1] for n in nis_log])
    dof = nis_log[0][2] if nis_log else 6
    aceitas = np.array([n[3] for n in nis_log])
    dist_total = np.sum(np.linalg.norm(np.diff(gt, axis=0), axis=1))

    print("\n=== RESULTADOS ===")
    print(f"ATE RMSE (ESKF)          : {ate:.3f} m   | máx {en.max():.3f} m | final {en[-1]:.3f} m")
    print(f"ATE RMSE (VIO bruto)     : {ate_vio:.3f} m")
    print(f"Deriva final / distância : {100*en[-1]/dist_total:.2f} % de {dist_total:.0f} m")
    print(f"RPE ({delta:.0f} s)             : média {rpe.mean():.3f} m "
          f"({100*np.mean(rpe/np.maximum(dist, 1e-6)):.2f} % da distância)")
    print(f"Erro de yaw              : RMS {math.degrees(math.sqrt(np.mean(err_yaw**2))):.2f}° | "
          f"final {math.degrees(err_yaw[-1]):.2f}°")
    print(f"NEES posição (médio)     : {nees.mean():.2f}   (consistente ≈ 3)")
    if len(nis):
        print(f"NIS VIO (médio)          : {nis[aceitas].mean():.2f}   (consistente ≈ {dof})")
        print(f"Medidas VIO rejeitadas   : {100*(1-aceitas.mean()):.1f} %")
    print(f"Resets anti-travamento   : {len(resets)}" +
          ("   <-- filtro divergiu; corrija a causa (eixos/ruídos), não confie no reset"
           if resets else ""))
    if log['ba_v']:
        ba_e = np.array(log['ba'])[sel][-1]
        ba_v = np.array(log['ba_v'])[sel][-1]
        bg_e = np.array(log['bg'])[sel][-1]
        bg_v = np.array(log['bg_v'])[sel][-1]
        print(f"Bias acel. final  est/real: {np.round(ba_e, 3)} / {np.round(ba_v, 3)} m/s²")
        print(f"Bias giro  final  est/real: {np.round(np.degrees(bg_e), 3)} / "
              f"{np.round(np.degrees(bg_v), 3)} °/s")

    if salvar:
        # ---- Exporta para evo (formato TUM) ----
        qs = np.array(log['q'])[sel]
        gq = [d['gt_q'][np.argmin(abs(d['gt_t'] - ti))] for ti in t]
        sufixo = '_ideal' if args.vio_ideal else '_realista'
        with open(f'est{sufixo}.tum', 'w') as fe, open(f'gt{sufixo}.tum', 'w') as fg:
            for ti, pe, qe, pg, qg in zip(t, est, qs, gt, gq):
                fe.write(f"{ti:.6f} {pe[0]:.5f} {pe[1]:.5f} {pe[2]:.5f} {qe[1]:.6f} {qe[2]:.6f} {qe[3]:.6f} {qe[0]:.6f}\n")
                fg.write(f"{ti:.6f} {pg[0]:.5f} {pg[1]:.5f} {pg[2]:.5f} {qg[1]:.6f} {qg[2]:.6f} {qg[3]:.6f} {qg[0]:.6f}\n")

        # ---- Gráficos ----
        sig_n = np.sqrt(np.array([np.trace(P) for P in Ppp]))
        tr = t - t[0]
        fig, ax = plt.subplots(2, 3, figsize=(20, 11))
        a = ax[0, 0]
        a.plot(gt[:, 0], gt[:, 1], 'g-', lw=3, label='Ground Truth')
        a.plot(pv[:, 0], pv[:, 1], '.', color='orange', ms=2, alpha=0.5, label='VIO (medida)')
        a.plot(est[:, 0], est[:, 1], 'b--', lw=1.6, label='ESKF')
        a.plot(gt[0, 0], gt[0, 1], 'ko', ms=9, label='Início')
        a.set_title('Trajetória XY', fontweight='bold'); a.axis('equal'); a.grid(ls=':'); a.legend()

        a = ax[0, 1]
        a.plot(tr, en, color='#c8102e', lw=1.5, label='|erro| vs GT')
        a.plot(tr, 3*sig_n, 'k--', lw=1, label='3σ do filtro')
        a.axhline(ate, color='navy', ls=':', label=f'ATE = {ate:.2f} m')
        for k, tr_ in enumerate(resets):
            a.axvline(tr_ - t[0], color='orange', lw=1, alpha=0.7, label='reset' if k == 0 else None)
        a.set_title('Erro de posição vs incerteza declarada', fontweight='bold')
        a.set_xlabel('t [s]'); a.set_ylabel('[m]'); a.grid(ls=':'); a.legend()

        a = ax[0, 2]
        a.plot(tr, gt[:, 2], 'g-', lw=2.5, label='z GT')
        a.plot(tr, est[:, 2], 'b--', lw=1.5, label='z ESKF')
        a.set_title('Altitude', fontweight='bold'); a.set_xlabel('t [s]'); a.grid(ls=':'); a.legend()

        a = ax[1, 0]
        a.plot(tr, np.degrees(err_yaw), color='purple', lw=1.5, label='erro de yaw')
        a.plot(tr, 3*np.degrees(sy), 'k--', lw=1, label='±3σ')
        a.plot(tr, -3*np.degrees(sy), 'k--', lw=1)
        a.set_title('Yaw (fracamente observável)', fontweight='bold')
        a.set_xlabel('t [s]'); a.set_ylabel('[°]'); a.grid(ls=':'); a.legend()

        a = ax[1, 1]
        if len(nis):
            tn = np.array([n[0] for n in nis_log]) - t[0]
            a.semilogy(tn[aceitas], nis[aceitas], '.', ms=3, label='NIS aceita')
            a.semilogy(tn[~aceitas], nis[~aceitas], 'rx', ms=8, mew=2, label='rejeitada (gate)')
            a.axhline(dof, color='k', ls='-', lw=1, label=f'esperado = {dof}')
            a.axhline(CHI2_99[dof], color='r', ls='--', lw=1, label='limiar χ² 99 %')
        a.set_title('NIS do VIO', fontweight='bold'); a.set_xlabel('t [s]'); a.grid(ls=':'); a.legend()

        a = ax[1, 2]
        a.semilogy(tr, nees, color='teal', lw=1, label='NEES posição')
        a.axhline(3, color='k', lw=1, label='esperado = 3')
        a.axhline(11.345, color='r', ls='--', lw=1, label='χ² 99 %')
        a.set_title('NEES (consistência vs GT)', fontweight='bold'); a.set_xlabel('t [s]')
        a.grid(ls=':'); a.legend()

        plt.suptitle(f"ESKF 15 estados | VIO {'ideal' if args.vio_ideal else 'realista'} | "
                     f"modo {args.modo_vio}", fontsize=14, fontweight='bold')
        plt.tight_layout()
        nome = f'eskf_vs_gt{sufixo}.png'
        plt.savefig(nome, dpi=200)
        print(f"\n[+] Gráfico: {nome} | trajetórias TUM: est{sufixo}.tum, gt{sufixo}.tum")
        print(f"    evo: evo_rpe tum gt{sufixo}.tum est{sufixo}.tum --delta 10 --delta_unit m -p")
    return dict(ate=ate, ate_vio=ate_vio, nees=nees.mean(),
                nis=nis[aceitas].mean() if len(nis) else np.nan,
                t=t - t[0], est=est, gt=gt, pv=pv, en=en, err_yaw=err_yaw,
                tv=tv - t[0], ev=np.linalg.norm(pv - gv, axis=1),
                sig3=3*np.sqrt(np.array([np.trace(P) for P in Ppp])),
                resets=[r - t[0] for r in resets], final=en[-1],
                rpe_pct=100*np.mean(rpe/np.maximum(dist, 1e-6)),
                yaw_rms=math.degrees(math.sqrt(np.mean(err_yaw**2))))


def estimar_escala_tempo(d):
    """k = 1/ganho médio do giro contra a velocidade angular derivada do GT."""
    t, qs = d['gt_t'], d['gt_q']
    w_pred = np.array([q_log(q_mult(np.r_[qs[k-1][0], -qs[k-1][1:]], qs[k+1])) / (t[k+1] - t[k-1])
                       for k in range(1, len(t)-1)])
    tc = t[1:-1]
    wm = np.column_stack([np.interp(tc, d['imu_t'], d['imu_gyr'][:, j]) for j in range(3)])
    g = [np.dot(wm[:, j], w_pred[:, j]) / max(np.dot(w_pred[:, j], w_pred[:, j]), 1e-12) for j in range(3)]
    return 1.0 / float(np.mean(g))


def evolucao(d_bruto, args):
    """Roda as etapas do processo de depuração na MESMA bag e compara numa figura."""
    k = ESCALA_TEMPO if abs(ESCALA_TEMPO - 1.0) > 1e-9 else estimar_escala_tempo(d_bruto)
    etapas = [
        ("1. Armadilha original\n(VIO ancorado no GT, tempo errado)", True, 1.0),
        ("2. VIO realista\n(tempo errado)", False, 1.0),
        ("3. VIO ideal\n(tempo corrigido)", True, k),
        ("4. VIO realista\n(tempo corrigido)", False, k),
    ]
    res = []
    for nome, ideal, kk in etapas:
        print("\n" + "=" * 70 + f"\n{nome.replace(chr(10), ' ')}  (k = {kk:.3f})\n" + "=" * 70)
        dd = copy.deepcopy(d_bruto)
        dd = corrigir_base_tempo(dd, kk)
        dd = corrigir_referencial_acc(dd, ACC_REFERENCIAL)
        a = argparse.Namespace(**vars(args))
        a.vio_ideal = ideal
        res.append((nome, executar(dd, a, salvar=False)))

    cores = ['#7f7f7f', '#c8102e', '#ff7f0e', '#1f77b4']
    fig = plt.figure(figsize=(22, 12))
    gs = fig.add_gridspec(2, 3)
    pos = [(0, 0), (0, 1), (1, 0), (1, 1)]
    gt = res[-1][1]['gt']
    cx = 0.5 * (gt[:, 0].min() + gt[:, 0].max())     # centro da caixa do GT (não a média)
    cy = 0.5 * (gt[:, 1].min() + gt[:, 1].max())
    meia = 0.6 * max(np.ptp(gt[:, 0]), np.ptp(gt[:, 1]), 10.0)
    for (nome, r), (i, j), cor in zip(res, pos, cores):
        a = fig.add_subplot(gs[i, j])
        a.plot(r['gt'][:, 0], r['gt'][:, 1], 'g-', lw=3, label='Ground Truth')
        a.plot(r['pv'][:, 0], r['pv'][:, 1], '.', color='orange', ms=1.5, alpha=0.4, label='VIO (medida)')
        a.plot(r['est'][:, 0], r['est'][:, 1], '--', color=cor, lw=1.8, label='ESKF')
        a.plot(r['gt'][0, 0], r['gt'][0, 1], 'ko', ms=8)
        saiu = np.any(np.abs(r['est'][:, :2] - [cx, cy]) > meia)
        a.set_xlim(cx - meia, cx + meia); a.set_ylim(cy - meia, cy + meia)
        a.set_aspect('equal')
        txt = f"ATE {r['ate']:.2f} m | final {r['final']:.2f} m\nyaw RMS {r['yaw_rms']:.1f}° | resets {len(r['resets'])}"
        if saiu:
            txt += "\n(ESKF sai do quadro: divergiu)"
        a.text(0.02, 0.02, txt, transform=a.transAxes, fontsize=10, va='bottom',
               bbox=dict(boxstyle='round', fc='white', alpha=0.85))
        a.set_title(nome, fontweight='bold'); a.grid(ls=':'); a.legend(loc='upper right', fontsize=9)

    a = fig.add_subplot(gs[0, 2])
    for (nome, r), cor in zip(res, cores):
        a.semilogy(r['t'], np.maximum(r['en'], 1e-3), color=cor, lw=1.6, label=nome.split('\n')[0])
    a.set_title('Erro de posição vs GT (escala log)', fontweight='bold')
    a.set_xlabel('t [s]'); a.set_ylabel('[m]'); a.grid(ls=':', which='both'); a.legend(fontsize=9)

    a = fig.add_subplot(gs[1, 2])
    nomes = [n.split('\n')[0] for n, _ in res]
    ates = [r['ate'] for _, r in res]
    b = a.barh(nomes[::-1], ates[::-1], color=cores[::-1])
    a.set_xscale('log'); a.set_xlabel('ATE RMSE [m] (log)')
    for rect, v in zip(b, ates[::-1]):
        a.text(rect.get_width(), rect.get_y() + rect.get_height()/2, f' {v:.2f} m', va='center')
    a.axvline(res[-1][1]['ate_vio'], color='orange', ls='--', label='ATE do VIO bruto (realista)')
    a.set_title('Resumo', fontweight='bold'); a.grid(ls=':', axis='x'); a.legend(fontsize=9)

    plt.suptitle(f'Evolução da depuração do ESKF aéreo (k de tempo = {k:.3f})', fontsize=15, fontweight='bold')
    plt.tight_layout()
    plt.savefig('evolucao_eskf.png', dpi=180)
    print("\n" + "=" * 70 + "\nRESUMO DA EVOLUÇÃO")
    for nome, r in res:
        print(f"  {nome.replace(chr(10), ' '):60s} ATE {r['ate']:9.2f} m | yaw RMS {r['yaw_rms']:6.1f}° | "
              f"resets {len(r['resets'])}")
    print("[+] Figura: evolucao_eskf.png")


def figura_etapa4(d_bruto, args):
    """Figura limpa da etapa 4: VIO realista com base de tempo corrigida."""
    k = ESCALA_TEMPO if abs(ESCALA_TEMPO - 1.0) > 1e-9 else estimar_escala_tempo(d_bruto)
    d = corrigir_base_tempo(copy.deepcopy(d_bruto), k)
    d = corrigir_referencial_acc(d, ACC_REFERENCIAL)
    a = argparse.Namespace(**vars(args))
    a.vio_ideal = False
    r = executar(d, a, salvar=False)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 7.5), gridspec_kw={'width_ratios': [1, 1.3]})
    ax1.plot(r['gt'][:, 0], r['gt'][:, 1], 'g-', lw=3, label='Ground Truth')
    ax1.plot(r['pv'][:, 0], r['pv'][:, 1], '.', color='orange', ms=2, alpha=0.45, label='VIO sintético (medida)')
    ax1.plot(r['est'][:, 0], r['est'][:, 1], '--', color='#1f77b4', lw=2, label='ESKF')
    ax1.plot(r['gt'][0, 0], r['gt'][0, 1], 'ko', ms=9, label='Início')
    ax1.plot(r['gt'][-1, 0], r['gt'][-1, 1], 's', color='#c8102e', ms=8, label='Fim (GT)')
    ax1.set_aspect('equal', adjustable='datalim')
    ax1.set_title('Trajetória XY', fontweight='bold')
    ax1.set_xlabel('X [m]'); ax1.set_ylabel('Y [m]'); ax1.grid(ls=':'); ax1.legend(loc='best')

    ax2.plot(r['tv'], r['ev'], '.', color='orange', ms=2, alpha=0.5, label=f"Erro do VIO bruto (ATE {r['ate_vio']:.2f} m)")
    ax2.plot(r['t'], r['en'], color='#1f77b4', lw=2, label=f"Erro do ESKF (ATE {r['ate']:.2f} m)")
    ax2.plot(r['t'], r['sig3'], 'k--', lw=1, label='3σ declarado pelo filtro')
    ax2.axhline(r['ate'], color='navy', ls=':', lw=1.2)
    for n, tr_ in enumerate(r['resets']):
        ax2.axvline(tr_, color='red', lw=1, alpha=0.6, label='reset' if n == 0 else None)
    ax2.set_title('Erro de posição vs Ground Truth', fontweight='bold')
    ax2.set_xlabel('t [s] (tempo físico corrigido)'); ax2.set_ylabel('Erro [m]')
    ax2.grid(ls=':'); ax2.legend(loc='upper left')
    txt = (f"ATE ESKF: {r['ate']:.2f} m   |   erro final: {r['final']:.2f} m\n"
           f"RPE (10 s): {r['rpe_pct']:.2f} % da distância\n"
           f"Yaw RMS: {r['yaw_rms']:.1f}°   |   resets: {len(r['resets'])}")
    ax2.text(0.98, 0.97, txt, transform=ax2.transAxes, ha='right', va='top', fontsize=11,
             bbox=dict(boxstyle='round', fc='white', alpha=0.9))
    plt.suptitle(f'ESKF 15 estados — IMU + VIO realista (base de tempo corrigida, k = {k:.3f})',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('etapa4_eskf_vio_realista.png', dpi=200)
    print("[+] Figura: etapa4_eskf_vio_realista.png")


def main():
    ap = argparse.ArgumentParser(description='ESKF aéreo com VIO realista')
    ap.add_argument('--bag', default='bag_uav_aereo_completo')
    ap.add_argument('--agente', default='uav0_id0')
    ap.add_argument('--duracao', type=float, default=100.0)
    ap.add_argument('--sintetico', action='store_true', help='usa trajetória sintética (sem bag)')
    ap.add_argument('--sintetico-lh', action='store_true',
                    help='sintético com IMU publicado em left-handed (reproduz o bug)')
    ap.add_argument('--sintetico-acc-mundo', action='store_true',
                    help='sintético com aceleração publicada no mundo (reproduz o bug)')
    ap.add_argument('--sintetico-tempo', type=float, default=0.0,
                    help='sintético com sim K× mais rápido que o real e carimbo de parede')
    ap.add_argument('--etapa4', action='store_true',
                    help='figura só da etapa 4 (VIO realista, tempo corrigido)')
    ap.add_argument('--evolucao', action='store_true',
                    help='roda as 4 etapas da depuração e gera evolucao_eskf.png')
    ap.add_argument('--diagnostico', action='store_true', help='só verifica eixos do IMU')
    ap.add_argument('--vio-ideal', action='store_true', help='reproduz a armadilha antiga')
    ap.add_argument('--modo-vio', choices=['pose_vel', 'vel'], default='pose_vel')
    ap.add_argument('--baro', action='store_true', help='funde barômetro (limita deriva em z)')
    ap.add_argument('--mag', action='store_true', help='funde magnetômetro (limita deriva de yaw)')
    ap.add_argument('--sem-compensacao', action='store_true', help='ignora a latência do VIO')
    ap.add_argument('--sem-gating', action='store_true')
    ap.add_argument('--sem-corromper-imu', action='store_true',
                    help='use se o IMU do BiguaSim já tiver ruído/bias configurados')
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    if args.sintetico_acc_mundo or args.sintetico_tempo:
        args.sintetico = True
    if args.sintetico or args.sintetico_lh:
        d = gerar_dados_sinteticos(args.duracao)
        if args.sintetico_lh:            # IMU "cru" em left-handed (Y invertido)
            M_lh = np.diag([1.0, -1.0, 1.0])
            d['imu_acc'] = d['imu_acc'] @ M_lh.T
            d['imu_gyr'] = d['imu_gyr'] @ (np.linalg.det(M_lh) * M_lh).T
        if args.sintetico_acc_mundo:     # simulador publicando acel. no mundo
            for n, q in enumerate(q_interp(d['imu_t'], d['gt_t'], d['gt_q'])):
                d['imu_acc'][n] = q_to_R(q) @ d['imu_acc'][n]
        if args.sintetico_tempo:         # carimbo de parede = tempo físico / K
            d['imu_t'] = d['imu_t'] / args.sintetico_tempo
            d['gt_t'] = d['gt_t'] / args.sintetico_tempo
        d['imu_acc'] = d['imu_acc'] @ M_ACC.T      # mesmo caminho da bag
        d['imu_gyr'] = d['imu_gyr'] @ M_GYR.T
    else:
        d = carregar_bag(args.bag, args.agente, args.duracao)
    if args.evolucao:
        evolucao(d, args)
        return
    if args.etapa4:
        figura_etapa4(d, args)
        return
    d = corrigir_base_tempo(d, ESCALA_TEMPO)
    d = corrigir_referencial_acc(d, ACC_REFERENCIAL)
    print(f"[*] IMU: {len(d['imu_t'])} amostras ({1/np.median(np.diff(d['imu_t'])):.0f} Hz) | "
          f"GT: {len(d['gt_t'])} amostras ({1/np.median(np.diff(d['gt_t'])):.0f} Hz)")

    diagnosticar_eixos(d)
    if args.diagnostico:
        return
    executar(d, args)


if __name__ == '__main__':
    main()