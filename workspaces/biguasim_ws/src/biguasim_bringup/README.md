# biguasim_bringup

Orchestration only (launch + config, no code). One command to start a
BiguaSim world and its ROS 2 sensor stack.

```bash
ros2 launch biguasim_bringup biguasim.launch.py
```

| Arg | Default | |
|---|---|---|
| `run_sim` | `true` | start BiguaSim as a subprocess |
| `biguasim_cmd` | `""` | **command that starts BiguaSim** — falls back to `$BIGUASIM_CMD`, then `DEFAULT_BIGUASIM_CMD` in the launch file |
| `world` | `""` | passed as `--world <world>` if set |
| `biguasim_extra_args` | `""` | appended to the sim command |
| `run_bridge` | `true` | also run `biguasim_bridge` (sonar/odom → standard types) |
| `run_check` | `true` | also run `biguasim_sensor_check` (rate/status table) |
| `rviz` | `false` | open RViz2 with `config/biguasim.rviz` (Image on `/son`, TF) |
| `use_sim_time` | `false` | set `true` when replaying a bag |

### You must tell it how BiguaSim starts

The launch runs `<biguasim_cmd> [--world <world>] [extra args]` as a plain
subprocess — it can't guess the command. Set it once:

```bash
# inside the container, e.g. append to ~/biguasim/setup.bash
export BIGUASIM_CMD="ros2 run <biguasim_pkg> <sim_executable>"
```

If BiguaSim runs on another machine (Windows/Unreal) and only publishes
over the network, use `run_sim:=false`:

```bash
ros2 launch biguasim_bringup biguasim.launch.py run_sim:=false
```

### Bag instead of a live sim

```bash
ros2 launch biguasim_bringup biguasim.launch.py run_sim:=false use_sim_time:=true &
ros2 bag play <bag> --clock
```
