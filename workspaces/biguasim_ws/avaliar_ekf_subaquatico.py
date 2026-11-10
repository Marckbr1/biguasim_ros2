#!/usr/bin/env python3
"""
Script de avaliação autossuficiente para dados subaquáticos do BiguaSim.
Lê a bag diretamente via mcap/rosbag2_py ou simula o pipeline offline,
calcula as métricas de erro RMSE e plota o comparativo do EKF 3D vs Ground Truth.
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
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

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


def processar_bag(bag_path, agent_id='uav0_id0', duracao_max_sec=120.0):
    print(f"[*] Abrindo bag: {bag_path}...")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    # Identificar tipos dos tópicos
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    odom_topic = f'/biguasim/{agent_id}/DynamicsSensor/Odom'
    imu_topic = f'/biguasim/{agent_id}/IMUSensor'
    dvl_topic = f'/biguasim/{agent_id}/DVLSensor/Velocity'

    # Estado EKF: [px, py, pz, vx, vy, vz, phi, theta, psi]
    x_est = np.zeros(9)
    P = np.eye(9) * 0.1
    Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001])
    R_dvl = np.diag([0.05, 0.05, 0.05])
    R_att = np.diag([0.005, 0.005, 0.005])
    g = np.array([0.0, 0.0, 9.80665])

    origin_init = False
    t_start = None
    last_imu_time = None

    # Séries temporais para plotagem
    t_series = []
    gt_traj = []
    ekf_traj = []

    print("[*] Processando mensagens e executando EKF 3D...")
    while reader.has_next():
        (topic, data, t_nanosec) = reader.read_next()
        msg_type_str = type_map.get(topic)
        if not msg_type_str:
            continue

        msg_cls = get_message(msg_type_str)
        msg = deserialize_message(data, msg_cls)

        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t_start is None:
            t_start = t_sec

        t_rel = t_sec - t_start
        if t_rel > duracao_max_sec:
            break

        # Ground Truth
        if topic == odom_topic:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if not origin_init:
                x_est[0] = p.x
                x_est[1] = p.y
                x_est[2] = p.z
                r, pitch, y = quaternion_to_euler(q.x, q.y, q.z, q.w)
                x_est[6:9] = [r, pitch, y]
                origin_init = True
                print(f"[+] Origem alinhada: X={p.x:.2f}, Y={p.y:.2f}, Z={p.z:.2f}")

            # Registra ponto de amostragem síncrona
            t_series.append(t_rel)
            gt_traj.append([p.x, p.y, p.z])
            ekf_traj.append([x_est[0], x_est[1], x_est[2]])

        # IMU (Predição e Atitude)
        elif topic == imu_topic and origin_init:
            if last_imu_time is None:
                last_imu_time = t_sec
                continue
            dt = t_sec - last_imu_time
            last_imu_time = t_sec
            if dt <= 0.0 or dt > 0.5:
                continue

            R = rotation_matrix(x_est[6], x_est[7], x_est[8])
            a_body = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            a_global = R @ a_body - g

            # Predição de estados
            x_est[0:3] += x_est[3:6] * dt + 0.5 * a_global * (dt ** 2)
            x_est[3:6] += a_global * dt

            F = np.eye(9)
            F[0:3, 3:6] = np.eye(3) * dt
            P = F @ P @ F.T + Q * dt

            # Correção de atitude
            q = msg.orientation
            r, pitch, y = quaternion_to_euler(q.x, q.y, q.z, q.w)
            z_att = np.array([r, pitch, y])
            H_att = np.zeros((3, 9))
            H_att[0:3, 6:9] = np.eye(3)

            y_att = z_att - x_est[6:9]
            for i in range(3):
                y_att[i] = math.atan2(math.sin(y_att[i]), math.cos(y_att[i]))

            S_att = H_att @ P @ H_att.T + R_att
            K_att = P @ H_att.T @ np.linalg.inv(S_att)
            x_est += K_att @ y_att
            P = (np.eye(9) - K_att @ H_att) @ P

        # DVL (Correção de Velocidade Linear)
        elif topic == dvl_topic and origin_init:
            R = rotation_matrix(x_est[6], x_est[7], x_est[8])
            v_body = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
            v_global_meas = R @ v_body

            H_v = np.zeros((3, 9))
            H_v[0:3, 3:6] = np.eye(3)

            y_v = v_global_meas - x_est[3:6]
            S_v = H_v @ P @ H_v.T + R_dvl
            K_v = P @ H_v.T @ np.linalg.inv(S_v)

            x_est += K_v @ y_v
            P = (np.eye(9) - K_v @ H_v) @ P

    gt = np.array(gt_traj)
    ekf = np.array(ekf_traj)
    t = np.array(t_series)

    # Cálculo do Erro e RMSE
    err_3d = np.linalg.norm(gt - ekf, axis=1)
    rmse = np.sqrt(np.mean(err_3d ** 2))
    max_err = np.max(err_3d)

    print("\n" + "="*50)
    print("           RESULTADOS QUANTITATIVOS EKF 3D        ")
    print("="*50)
    print(f" Amostras avaliadas: {len(t)}")
    print(f" Duração analisada:  {t[-1]:.2f} segundos")
    print(f" RMSE 3D Total:      {rmse:.4f} metros")
    print(f" Erro Máximo 3D:     {max_err:.4f} metros")
    print("="*50 + "\n")

    # Plotagem dos Gráficos
    fig = plt.figure(figsize=(14, 9))

    # 1. Trajetória no plano XY
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(gt[:, 0], gt[:, 1], 'k--', linewidth=2, label='Ground Truth')
    ax1.plot(ekf[:, 0], ekf[:, 1], 'b-', linewidth=1.8, label='EKF 3D (IMU + DVL)')
    ax1.plot(gt[0, 0], gt[0, 1], 'go', markersize=8, label='Início')
    ax1.plot(gt[-1, 0], gt[-1, 1], 'rs', markersize=8, label='Fim')
    ax1.set_title('Trajetória Horizontal Subaquática (Plano XY)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Posição X (m)')
    ax1.set_ylabel('Posição Y (m)')
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.legend(loc='best')
    ax1.axis('equal')

    # 2. Perfil de Profundidade Z
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, gt[:, 2], 'k--', linewidth=2, label='Ground Truth Z')
    ax2.plot(t, ekf[:, 2], 'r-', linewidth=1.8, label='EKF 3D Z')
    ax2.set_title('Estabilidade de Profundidade (Cota Z)', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Tempo (s)')
    ax2.set_ylabel('Profundidade Z (m)')
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.legend(loc='best')

    # 3. Erro de Translação ao longo do tempo
    ax3 = fig.add_subplot(2, 1, 2)
    ax3.plot(t, err_3d, 'm-', linewidth=1.8, label=f'Erro Instantâneo (RMSE = {rmse:.3f} m)')
    ax3.axhline(rmse, color='r', linestyle='--', label=f'RMSE Médio = {rmse:.3f} m')
    ax3.set_title('Erro Euclidiano de Estimação 3D ao Longo da Missão', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Tempo (s)')
    ax3.set_ylabel('Erro (m)')
    ax3.grid(True, linestyle=':', alpha=0.7)
    ax3.legend(loc='best')

    plt.tight_layout()
    output_img = 'avaliacao_ekf_subaquatico.png'
    plt.savefig(output_img, dpi=300)
    print(f"[+] Gráfico comparativo salvo com sucesso em: {output_img}")


if __name__ == '__main__':
    bag_dir = 'bag_uav_completa_sonar'
    if not os.path.exists(bag_dir):
        print(f"Erro: diretório '{bag_dir}' não encontrado no caminho atual.")
        sys.exit(1)
    processar_bag(bag_dir)
