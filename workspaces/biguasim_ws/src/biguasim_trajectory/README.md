# biguasim_trajectory

Mueve un agente BiguaSim en **cuadrado** o **circulo** (las dos unicas
trayectorias soportadas -- portado y recortado de `biguasim_trajectory` en
el workspace `anterior`, que ademas tenia `line` y `figure_eight`).

Usa su propio escenario de un solo agente (`config/trajectory_scenario.yaml`):
`auv0` con `control_abstraction: 'cmd_pos_yaw'` (a diferencia del
`config.yaml` default de `biguasim_main`, que usa `cmd_motor_speeds`) y
sensores `DynamicsSensor` + `ImagingSonar`.

## Uso

Solo trayectoria (sin sonar/RViz):

```bash
ros2 launch biguasim_trajectory trajectory_launch.py
ros2 launch biguasim_trajectory trajectory_launch.py trajectory_type:=circle circle_radius:=8.0
ros2 launch biguasim_trajectory trajectory_launch.py trajectory_type:=square square_side:=15.0
```

Trayectoria + sonar activado (bridge a `/son` + RViz2 con el display ya
armado):

```bash
ros2 launch biguasim_trajectory trajectory_sonar_launch.py
ros2 launch biguasim_trajectory trajectory_sonar_launch.py trajectory_type:=circle
ros2 launch biguasim_trajectory trajectory_sonar_launch.py rviz:=false   # solo el bridge, sin ventana
```

## Parametros principales (`trajectory_launch.py`, heredados por `trajectory_sonar_launch.py`)

| Parametro | Default | Nota |
|---|---|---|
| `trajectory_type` | `square` | `square` o `circle` |
| `square_side` | `10.0` | lado del cuadrado (m) |
| `circle_radius` | `5.0` | radio del circulo (m) |
| `speed` | `1.0` | velocidad de recorrido (m/s) |
| `target_depth` | `-3.0` | profundidad objetivo antes de arrancar la forma |
| `corner_wait_time` | `3.0` | pausa en cada esquina del cuadrado (s), da tiempo al controlador a asentarse |
| `loop_trajectory` | `true` | repite la forma indefinidamente en vez de mantenerse en la posicion final |
| `agent_ros_id` | `auv0_id0` | debe coincidir con el agente `cmd_pos_yaw` del escenario |

## Notas

- El nodo espera el primer `DynamicsSensor/Odom` para fijar el origen
  (posicion inicial real del agente) antes de arrancar; si no llega dentro
  de `startup_timeout_sec` asume origen `(0, 0, target_depth)`.
- La geometria pura (sin ROS) vive en `trajectories.py`, testeable de forma
  aislada.
