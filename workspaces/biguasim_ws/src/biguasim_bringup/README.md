# biguasim_bringup

Orchestration only (launch + config, no code). One command to start a
BiguaSim world and its ROS 2 sensor stack.

```bash
ros2 launch biguasim_bringup biguasim.launch.py
```

| Arg | Default | |
|---|---|---|
| `run_sim` | `true` | start BiguaSim |
| `use_biguasim_main` | `true` | start it via `biguasim_main`'s own launch file (the ROS-owned `biguasim_node` bridge — see `../biguasim_main/README.md`). Ignored if `biguasim_cmd` is set. |
| `biguasim_params_file` | `""` | scenario/parameters YAML passed to `biguasim_main` (default: its own `config/config.yaml`) |
| `biguasim_cmd` | `""` | **override**: command that starts BiguaSim as an external process instead — falls back to `$BIGUASIM_CMD`, then `DEFAULT_BIGUASIM_CMD` in the launch file. Takes priority over `use_biguasim_main` when set. |
| `world` | `""` | passed as `--world <world>` if set (`biguasim_cmd` only) |
| `biguasim_extra_args` | `""` | appended to the sim command (`biguasim_cmd` only) |
| `run_bridge` | `true` | also run `biguasim_bridge` (sonar/odom → standard types) |
| `run_check` | `true` | also run `biguasim_sensor_check` (rate/status table) |
| `rviz` | `false` | open RViz2 with `config/biguasim.rviz` (Image on `/son`, TF) |
| `use_sim_time` | `false` | set `true` when replaying a bag |

### How BiguaSim is started

**Default (`use_biguasim_main:=true`):** this launch includes
`biguasim_main/launch/biguasim_main.launch.py`, which owns the
`biguasim.make(...)` environment directly — no external process or
`$BIGUASIM_CMD` needed. If `biguasim_main` isn't built yet, this logs a
clear message and skips the sim rather than failing the whole launch.

**Override:** to run BiguaSim as an external process instead (a different
package installed by `install_biguasim.sh`, or a script you don't want
managed by this launch), set `biguasim_cmd` (or `$BIGUASIM_CMD`) — it runs
`<biguasim_cmd> [--world <world>] [extra args]` as a plain subprocess and
takes priority over `use_biguasim_main`:

```bash
ros2 launch biguasim_bringup biguasim.launch.py biguasim_cmd:="ros2 run <biguasim_pkg> <sim_executable>"
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
