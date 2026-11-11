# BiguaSim SITL + QGroundControl + ROS 2

Cadeia completa: ArduSub (SITL, física do BiguaSim via JSON) → MAVProxy → MAVROS → nó de waypoints, com QGroundControl no mapa.

## Rodar

No host:

```bash
xhost +local:
/mnt/HD2/biguasim_ros2/docker/start_container.sh   # ou enter_container.sh, se já estiver ligado
```

Dentro do contêiner (prompt `biguauser@...`):

```bash
cd ~/workspaces/biguasim_ws/src/biguasim_sitl
./scripts/run_sim.sh
```

Só isso. Na primeira vez após recriar o contêiner, o script instala as dependências, clona e compila o ArduPilot e compila o pacote (demora). Nas próximas, ele só confere e sobe direto.

O que abre:

- janela **ArduPilot SITL** (xterm) com o prompt do MAVProxy;
- o Unreal Engine (nó SITL);
- MAVROS, assim que o nó SITL fica online;
- waypoints, só quando o EKF passa a usar GPS (`EKF3 IMU0 is using GPS`). Antes disso o ArduSub recusa GUIDED, então pode levar alguns minutos, porque a simulação roda mais devagar que o tempo real.

Os logs de tudo saem no terminal onde você rodou o script.

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
| Nada gráfico abre | Rode `xhost +local:` no host antes de ligar o contêiner |
| Waypoints sobem por timeout e o GUIDED é recusado ("Guided requires position") | O EKF ainda não tem posição; aumente `waypoints.wait_timeout` |
| Console/mapa do MAVProxy não abrem (`No module named 'wx'`) | Desligados por padrão; para usar, instale `python3-wxgtk4.0` e ponha `console`/`map` em `true` |
