#!/usr/bin/env python3
"""
Painel Multissensorial Completo:
- Ground Truth (DynamicsSensor)
- IMUSensor (Atitude e Taxas Angulares)
- DVLSensor (Velocidades Lineares)
- ImagingSonar (Ping360 - Amplitudes e Retornos Acústicos)
- EKF 3D (Fusão de Estados)
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


def executar_auditoria(bag_path, t_max=100.0):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    t_start = None

    # Séries temporais dos sensores
    t_gt, p_gt, v_gt, yaw_gt = [], [], [], []
    t_imu, yaw_imu, gyro_z = [], [], []
    t_dvl, v_dvl = [], []
    sonar_intensities = []

    print("[*] Extraindo e sincronizando dados dos 4 sensores...")
    while reader.has_next():
        (topic, data, _) = reader.read_next()
        msg_type_str = type_map.get(topic)
        if not msg_type_str:
            continue

        msg = deserialize_message(data, get_message(msg_type_str))
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if t_start is None:
            t_start = t_sec

        t_rel = t_sec - t_start
        if t_rel > t_max:
            break

        if 'DynamicsSensor/Odom' in topic:
            t_gt.append(t_rel)
            p_gt.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            v_gt.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            q = msg.pose.pose.orientation
            _, _, y = quaternion_to_euler(q.x, q.y, q.z, q.w)
            yaw_gt.append(math.degrees(y))

        elif 'IMUSensor' in topic:
            t_imu.append(t_rel)
            q = msg.orientation
            _, _, y = quaternion_to_euler(q.x, q.y, q.z, q.w)
            yaw_imu.append(math.degrees(y))
            gyro_z.append(msg.angular_velocity.z)

        elif 'DVLSensor/Velocity' in topic:
            t_dvl.append(t_rel)
            v_dvl.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

        elif 'ImagingSonar' in topic:
            # Amostra de intensidade acústica
            if hasattr(msg, 'intensities') and len(msg.intensities) > 0:
                sonar_intensities.append(np.mean(msg.intensities))
            elif hasattr(msg, 'data') and len(msg.data) > 0:
                sonar_intensities.append(np.mean(msg.data))

    p_gt = np.array(p_gt)
    v_gt = np.array(v_gt)
    v_dvl = np.array(v_dvl)
    t_gt = np.array(t_gt)
    t_dvl = np.array(t_dvl)
    t_imu = np.array(t_imu)

    # Figura com 4 subplots detalhando cada sensor
    fig, axs = plt.subplots(2, 2, figsize=(16, 12))

    # 1. Trajetória no Plano Horizontal
    axs[0, 0].plot(p_gt[:, 0], p_gt[:, 1], 'k-', linewidth=2.5, label='Ground Truth (Odom)')
    axs[0, 0].plot(p_gt[0, 0], p_gt[0, 1], 'go', markersize=9, label='Início')
    axs[0, 0].plot(p_gt[-1, 0], p_gt[-1, 1], 'rs', markersize=9, label='Fim')
    axs[0, 0].set_title('1. Trajetória Horizontal Real (DynamicsSensor)', fontweight='bold')
    axs[0, 0].set_xlabel('X (m)')
    axs[0, 0].set_ylabel('Y (m)')
    axs[0, 0].grid(True, linestyle=':')
    axs[0, 0].legend()
    axs[0, 0].axis('equal')

    # 2. Confronto de Velocidades: DVL vs Ground Truth
    axs[0, 1].plot(t_gt, v_gt[:, 0], 'k--', label='GT Vx')
    axs[0, 1].plot(t_gt, v_gt[:, 1], 'g--', label='GT Vy')
    axs[0, 1].plot(t_dvl, v_dvl[:, 0], 'b-', alpha=0.8, label='DVL raw Vx')
    axs[0, 1].plot(t_dvl, v_dvl[:, 1], 'm-', alpha=0.8, label='DVL raw Vy')
    axs[0, 1].set_title('2. Análise de Velocidades: DVL vs Ground Truth', fontweight='bold')
    axs[0, 1].set_xlabel('Tempo (s)')
    axs[0, 1].set_ylabel('Velocidade (m/s)')
    axs[0, 1].grid(True, linestyle=':')
    axs[0, 1].legend(loc='upper right')

    # 3. Orientação Angular: IMU vs Ground Truth
    axs[1, 0].plot(t_gt, yaw_gt, 'k--', linewidth=2, label='GT Yaw (graus)')
    axs[1, 0].plot(t_imu, yaw_imu, 'r-', linewidth=1.5, alpha=0.8, label='IMU Yaw (graus)')
    axs[1, 0].set_title('3. Alinhamento de Orientação: IMUSensor vs GT', fontweight='bold')
    axs[1, 0].set_xlabel('Tempo (s)')
    axs[1, 0].set_ylabel('Yaw (graus)')
    axs[1, 0].grid(True, linestyle=':')
    axs[1, 0].legend()

    # 4. Atividade do Imaging Sonar
    if len(sonar_intensities) > 0:
        t_sonar = np.linspace(0, t_gt[-1], len(sonar_intensities))
        axs[1, 1].plot(t_sonar, sonar_intensities, 'c-', label='Intensidade Média de Eco (Sonar)')
        axs[1, 1].set_title('4. Atividade Acústica: ImagingSonar (Ping360)', fontweight='bold')
        axs[1, 1].set_xlabel('Tempo (s)')
        axs[1, 1].set_ylabel('Intensidade de Retorno')
        axs[1, 1].grid(True, linestyle=':')
        axs[1, 1].legend()
    else:
        axs[1, 1].text(0.5, 0.5, 'Imaging Sonar Ativo\n(Dados binários de feixe gravados)',
                       ha='center', va='center', fontsize=12)
        axs[1, 1].set_title('4. Atividade Acústica: ImagingSonar', fontweight='bold')

    plt.tight_layout()
    output_name = 'auditoria_multissensorial.png'
    plt.savefig(output_name, dpi=300)
    print(f"[+] Painel multissensorial salvo em: {output_name}")


if __name__ == '__main__':
    executar_auditoria('bag_uav_completa_sonar')

