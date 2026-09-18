import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = 'sensor_log.csv'
if not os.path.exists(csv_path):
    print(f"Erro: Arquivo '{csv_path}' nao encontrado. Execute o capture_sensors.py primeiro!")
    exit(1)

df = pd.read_csv(csv_path)

if df.empty or len(df) < 2:
    print("Erro: sensor_log.csv nao contem amostras suficientes.")
    exit(1)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Grafico 1: Plano Horizontal (Trajetória XY)
ax1.plot(df['pos_x'], df['pos_y'], label='Trajetoria Real', color='royalblue', lw=2)
ax1.scatter([df['pos_x'].iloc[0]], [df['pos_y'].iloc[0]], color='forestgreen', s=100, label='Inicio', zorder=5)
ax1.scatter([df['pos_x'].iloc[-1]], [df['pos_y'].iloc[-1]], color='crimson', s=100, label='Fim', zorder=5)
ax1.set_title('Plano Horizontal (XY)')
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.axis('equal')
ax1.legend()

# Grafico 2: Perfil de Profundidade (Z)
ax2.plot(df.index, df['pos_z'], label='Profundidade (Z)', color='darkorange', lw=2)
ax2.set_title('Perfil Vertical (Profundidade ao longo do tempo)')
ax2.set_xlabel('Amostras')
ax2.set_ylabel('Profundidade Z [m]')
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.legend()

output_img = 'trajetoria_resultado.png'
plt.tight_layout()
plt.savefig(output_img, dpi=300)
print(f"Grafico salvo com sucesso: '{output_img}' ({len(df)} pontos salvos).")
