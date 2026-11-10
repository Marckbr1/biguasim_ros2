# BiguaSim SITL + QGroundControl + ROS 2

Cadeia completa: ArduSub (SITL, física do BiguaSim via JSON) → MAVProxy → MAVROS → nó de waypoints, com QGroundControl no mapa.

## Rodar

```bash
/mnt/HD2/biguasim_ros2/docker/enter_container.sh
cd ~/workspaces/biguasim_ws/src/<caminho>/biguasim_sitl
./scripts/run_sim.sh
```

Só isso. Na primeira vez após recriar o contêiner, o script instala as dependências, clona e compila o ArduPilot e compila o pacote (demora). Nas próximas, ele só confere e sobe direto.

O que abre:

- janela **ArduPilot SITL** (xterm) com o prompt do MAVProxy, mais console e mapa;
- o Unreal Engine (nó SITL);
- MAVROS e waypoints, com log no terminal onde você rodou o script.

O QGroundControl conecta sozinho na UDP 14550. **Ctrl+C** no terminal do script encerra tudo e limpa os processos do ArduPilot.

## Configurar

Tudo fica em **`config/sim.yaml`**: posição inicial, portas, caminho do ArduPilot, ligar/desligar MAVROS e waypoints, atrasos, argumentos extras. Não é preciso recompilar depois de editar.

## Verificação

```bash
ros2 topic echo /mavros/state --once              # connected: true
ros2 topic echo /biguasim_sitl/status             # online, frames contando
ros2 topic echo /biguasim_waypoints/status        # conectado, armado, modo, WP alvo
ros2 topic echo /biguasim_sitl/odom_gt --field pose.pose.position.z
```

No MAVProxy, o prompt muda de `MAV>` para `GUIDED>` quando o modo é aceito.

## Problemas comuns

| Sintoma | Causa / solução |
|---|---|
| MAVROS aborta ao iniciar | Falta o geoide: rode `./scripts/setup_container.sh` |
| MAVROS só sobe após ~90 s | O texto de `sitl.ready_text` não apareceu no log do nó; ajuste o texto ou reduza `sitl.ready_timeout` |
| Mudança no launch não aparece | `./scripts/run_sim.sh --rebuild` |
| `-L RATBeach` não encontrado | A localização sumiu do `locations.txt`; use `custom_location` no `sim.yaml` |
| Nada gráfico abre | `DISPLAY` vazio no contêiner |
