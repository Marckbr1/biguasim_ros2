# biguasim_sitl

Expõe a ponte ArduPilot JSON SITL do BiguaSim como nó ROS 2, permitindo pilotar
o veículo simulado pelo QGroundControl (com mapa e missões) e, ao mesmo tempo,
consumir o estado e os sensores de percepção em ROS 2.

```
QGroundControl ──MAVLink 14550──┐
                                ├── ArduPilot SITL ──UDP 9002── biguasim_sitl ── BiguaSim (UE5)
MAVROS         ──MAVLink 14551──┘
```

Divisão de responsabilidades:

| Componente | Responsabilidade |
|---|---|
| MAVROS | navegação, missão/waypoints, modo de voo, arming, GPS e estado do veículo |
| `biguasim_sitl` | `env.step()`, ponte UDP com o ArduPilot, sonar, câmera e ground truth |

**Apenas um processo pode ser dono do ambiente BiguaSim.** Este nó substitui o
`biguasim_node`/`biguasim_main` quando se roda em modo SITL — não execute os dois.

## Pré-requisitos

BiguaSim instalado com o branch `sitl-test` (`hydrone-furg/biguasim`), que traz o
módulo `biguasim.ardubridge`. ArduPilot com `sim_vehicle.py` no PATH. MAVROS
(`ros-jazzy-mavros`) e QGroundControl.

## Uso

Quatro processos. O ArduPilot precisa subir antes do nó, ou o nó fica esperando
pacotes na porta UDP (o que é inofensivo — ele apenas avisa e aguarda).

**1. ArduPilot SITL** com dois endpoints MAVLink: um para o QGC, outro para o MAVROS.

```bash
sim_vehicle.py -v ArduSub -L RATBeach --console --map \
  -f JSON:127.0.0.1 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551
```

**2. Nó SITL + MAVROS**

```bash
ros2 launch biguasim_sitl sitl.launch.py
```

Com percepção ligada:

```bash
ros2 launch biguasim_sitl sitl.launch.py enable_sonar:=true enable_camera:=true
```

**3. QGroundControl** — conecta sozinho na 14550. O veículo aparece no mapa na
posição correspondente à origem GPS configurada.

## Tópicos publicados

| Tópico | Tipo | Descrição |
|---|---|---|
| `~/odom_gt` | `nav_msgs/Odometry` | Pose e velocidade ground truth, referencial NWU |
| `~/velocity_gt` | `geometry_msgs/TwistStamped` | Velocidade linear ground truth |
| `~/sonar/image` | `sensor_msgs/Image` (32FC1) | Imagem polar do ImagingSonar, se habilitado |
| `~/camera/image_raw` | `sensor_msgs/Image` (bgr8) | RGBCamera, se habilitada |
| `~/status` | `std_msgs/String` | Conexão com o ArduPilot, contagem de frames, tempo de simulação |

Navegação, GPS e missão vêm pelo MAVROS: `/mavros/state`,
`/mavros/global_position/global`, `/mavros/local_position/pose`,
`/mavros/mission/waypoints`.

## Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `vehicle` | `BlueROV2` | `BlueROV2`, `BlueROVHeavy`, `BlueBoat`, `DjiMatrice`, `TorpedoAUV` |
| `package_name` / `world` | `SkyDive` / `Pier-Harbor` | Pacote e mundo do simulador |
| `ticks_per_sec` | `200` | Taxa do SITL |
| `location` | `[0, 0, -1]` | Posição inicial do agente |
| `gps_lat` / `gps_lon` | RAT Beach | Origem GPS sintética |
| `bridge_address` / `bridge_port` | `127.0.0.1` / `9002` | Socket UDP do JSON FDM |
| `enable_sonar` / `sonar_hz` | `false` / `10` | ImagingSonar |
| `enable_camera` / `camera_hz` | `false` / `20` | RGBCamera |

## Sobre o GPS e o mapa

**Não existe sensor GPS nesta cadeia.** A latitude e longitude enviadas ao
ArduPilot são calculadas pelo `pos_nwu_to_ap()` a partir da posição local em
metros somada a uma origem fixa — por padrão RAT Beach, Califórnia
(33.810313, -118.393867), a mesma `-L RATBeach` do `sim_vehicle.py`.

Para mover o mundo para outra região, mude `gps_lat`/`gps_lon` **e** a flag `-L`
de forma coerente. Divergência entre as duas faz o EKF do ArduPilot rejeitar a
posição. Localizações personalizadas são definidas no `locations.txt` do
ArduPilot.

## Notas de operação

O `warmup_frames` do BlueROV2 é 1700: nos primeiros segundos os motores ficam
travados em zero por projeto, mesmo com o veículo armado. Não é falha.

O laço do nó é ditado pelo UDP do ArduPilot, não por um timer ROS. Por isso o
`main()` usa `spin_once(timeout_sec=0.0)` em vez de `spin()` — bloquear aqui faz
o ArduPilot acusar perda de frames.

Sonar e câmera têm `Hz` próprio no cenário. Sem isso rodariam a 200 Hz, o que
inviabiliza o desempenho.

## Limitações conhecidas

O `test.py` do repositório upstream importa `from ardupilot_biguasim import ...`,
mas o módulo se chama `biguasim.ardubridge` — o import quebra. Este pacote usa o
caminho correto.

Em `frame.py`, `pos_nwu_to_ap()` tem uma variável `z_down` não utilizada e um
comentário afirmando que o eixo z do sensor aponta para baixo. Pelo docstring
(NWU), o correto é não negar, que é o que o código faz; o comentário está
desatualizado.

Não há troca de dados entre este pacote e o `biguasim_main`/`biguasim_bridge`:
são integrações paralelas com o simulador. Aquela usa a API Python diretamente,
sem controlador de voo no circuito.
