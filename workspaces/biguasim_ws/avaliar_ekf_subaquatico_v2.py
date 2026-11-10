#!/usr/bin/env python3
"""
Avaliação Final EKF vs Ground Truth (Padrão BlueROV2):
- Painel Esquerdo: Trajetória Horizontal (XY) - Ground Truth vs EKF (DVL + IMU)
- Painel Direito: Desvio Euclidiano ao Longo do Tempo (RMSE e Erro Máximo)
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rotation_matrix(phi, theta, psi):
    c_p, s_p = math.cos(phi), math.sin(phi)
    c_t, s_t = math.cos(theta), math.sin(theta)
    c_y, s_y = math.cos(psi), math.sin(psi)
    return np.array([
        [c_y * c_t, c_y * s_t * s_p - s_y * c_p, c_y * s_t * c_p + s_y * s_p],
        [s_y * c_t, s_y * s_t * s_p + c_y * c_p, s_y * s_t * c_p - c_y * s_p],
        [-s_t,      c_t * s_p,                   c_t * c_p]
    ])


def gerar_grafico_padrao(bag_path, agent_id='uav0_id0', duracao_seg=100.0):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    odom_topic = f'/biguasim/{agent_id}/DynamicsSensor/Odom'
    dvl_topic = f'/biguasim/{agent_id}/DVLSensor/Velocity'

    # Estado: [px, py, pz, vx, vy, vz]
    x_ekf = np.zeros(6)
    P = np.eye(6) * 0.01
    Q = np.diag([0.001, 0.001, 0.001, 0.005, 0.005, 0.005])
    R_dvl = np.diag([0.01, 0.01, 0.01])

    current_att = np.zeros(3)
    origem_definida = False
    t_zero = None
    ultimo_t_dvl = None

    t_amostras = []
    gt_xy = []
    ekf_xy = []

    print("[*] Lendo bag e calculando EKF corrigido...")
    while reader.has_next():
        (topic, data, _) = reader.read_next()
        msg_type_str = type_map.get(topic)
        if not msg_type_str:
            continue

        msg = deserialize_message(data, get_message(msg_type_str))
        t_atual = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if t_zero is None:
            t_zero = t_atual

        t_rel = t_atual - t_zero
        if t_rel > duracao_seg:
            break

        if topic == odom_topic:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            current_att = np.array(quaternion_to_euler(q.x, q.y, q.z, q.w))

            if not origem_definida:
                x_ekf[0:3] = [p.x, p.y, p.z]
                origem_definida = True

            t_amostras.append(t_rel)
            gt_xy.append([p.x, p.y, p.z])
            ekf_xy.append([x_ekf[0], x_ekf[1], x_ekf[2]])

        elif topic == dvl_topic and origem_definida:
            if ultimo_t_dvl is None:
                ultimo_t_dvl = t_atual
                continue
            dt = t_atual - ultimo_t_dvl
            ultimo_t_dvl = t_atual
            if dt <= 0.0 or dt > 0.5:
                continue

            # Correção de polaridade diagnosticada: v_corpo = -v_dvl
            v_corpo = np.array([
                  -msg.twist.twist.linear.x,  # Vy do DVL mapeia para o avanço longitudinal
                  -msg.twist.twist.linear.y,  # Vx do DVL mapeia para a velocidade lateral
                  -msg.twist.twist.linear.z,  # Vz vertical
            ])

            R = rotation_matrix(current_att[0], current_att[1], current_att[2])
            z_vel = R @ v_corpo

            # Predição
            x_ekf[0:3] += x_ekf[3:6] * dt
            F = np.eye(6)
            F[0:3, 3:6] = np.eye(3) * dt
            P = F @ P @ F.T + Q * dt

            # Atualização
            H = np.zeros((3, 6))
            H[0:3, 3:6] = np.eye(3)
            y = z_vel - x_ekf[3:6]
            S = H @ P @ H.T + R_dvl
            K = P @ H.T @ np.linalg.inv(S)

            x_ekf += K @ y
            P = (np.eye(6) - K @ H) @ P

    gt = np.array(gt_xy)
    ekf = np.array(ekf_xy)
    t = np.array(t_amostras)

    erro_inst = np.linalg.norm(gt - ekf, axis=1)
    rmse = np.sqrt(np.mean(erro_inst ** 2))
    max_err = np.max(erro_inst)

    print("\n" + "="*55)
    print(f"  RMSE 3D Final:  {rmse:.4f} metros")
    print(f"  Erro Máximo 3D: {max_err:.4f} metros")
    print("="*55 + "\n")

    # Plot idêntico ao modelo de apresentação do BlueROV2
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

    # 1. Trajetória Horizontal
    ax1.plot(gt[:, 0], gt[:, 1], 'g-', linewidth=2.5, label='Ground Truth (Real)')
    ax1.plot(ekf[:, 0], ekf[:, 1], 'b--', linewidth=2.0, label='Estimativa EKF (DVL + IMU)')
    ax1.plot(gt[0, 0], gt[0, 1], 'ko', markersize=10, label='Início')
    ax1.plot(gt[-1, 0], gt[-1, 1], 'ro', markersize=9, label='Fim GT')
    ax1.set_title('Trajetória Horizontal: Ground Truth vs EKF', fontsize=13, fontweight='bold')
    ax1.set_xlabel('X [m]', fontsize=11)
    ax1.set_ylabel('Y [m]', fontsize=11)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='best', fontsize=10)
    ax1.axis('equal')

    # 2. Desvio Euclidiano
    ax2.plot(t, erro_inst, color='#c8102e', linewidth=2.0, label='Erro Instantâneo')
    ax2.axhline(rmse, color='navy', linestyle='--', linewidth=1.5, label=f'RMSE = {rmse:.3f} m')
    ax2.set_title(f'Desvio Euclidiano (Médio: {rmse:.3f} m | Máx: {max_err:.3f} m)', fontsize=13, fontweight='bold')
    ax2.set_xlabel('Tempo [s]', fontsize=11)
    ax2.set_ylabel('Erro [m]', fontsize=11)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=10)

    plt.tight_layout()
    nome_saida = 'comparacao_ekf_vs_gt_uav.png'
    plt.savefig(nome_saida, dpi=300)
    print(f"[+] Gráfico comparativo gerado com sucesso: {nome_saida}")


if __name__ == '__main__':
    gerar_grafico_padrao('bag_uav_completa_sonar')