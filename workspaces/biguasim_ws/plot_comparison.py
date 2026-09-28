import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = 'comparison_log.csv'
if not os.path.exists(csv_path):
    print(f"Erro: Arquivo '{csv_path}' não encontrado.")
    exit(1)

df = pd.read_csv(csv_path)

if len(df) < 5:
    print("Amostras insuficientes no arquivo CSV.")
    exit(1)

# Cálculo do erro euclidiano 2D ponto a ponto
error = ((df['gt_x'] - df['ekf_x'])**2 + (df['gt_y'] - df['ekf_y'])**2)**0.5

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

# 1. Trajetórias no plano horizontal (XY)
ax1.plot(df['gt_x'], df['gt_y'], label='Ground Truth (Real)', color='forestgreen', lw=2.5)
ax1.plot(df['ekf_x'], df['ekf_y'], label='Estimativa EKF (DVL + IMU)', color='royalblue', linestyle='--', lw=2)
ax1.scatter([df['gt_x'].iloc[0]], [df['gt_y'].iloc[0]], color='black', s=90, zorder=5, label='Início')
ax1.scatter([df['gt_x'].iloc[-1]], [df['gt_y'].iloc[-1]], color='crimson', s=90, zorder=5, label='Fim GT')
ax1.set_title('Trajetória Horizontal: Ground Truth vs EKF')
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.axis('equal')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.legend()

# 2. Curva de desvio acumulado (Drift)
ax2.plot(error, color='crimson', lw=2)
ax2.set_title(f'Desvio Euclidiano (Médio: {error.mean():.3f} m | Máx: {error.max():.3f} m)')
ax2.set_xlabel('Amostras (10 Hz)')
ax2.set_ylabel('Erro [m]')
ax2.grid(True, linestyle=':', alpha=0.6)

output_img = 'comparacao_ekf_vs_gt.png'
plt.tight_layout()
plt.savefig(output_img, dpi=300)
print(f"Gráfico gerado com sucesso: '{output_img}'")
print(f"Total de amostras: {len(df)}")
print(f"Erro médio: {error.mean():.3f} m | Erro final: {error.iloc[-1]:.3f} m")
