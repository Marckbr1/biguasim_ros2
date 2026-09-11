# biguasim_main

Core simulation bridge for BiguaSim: owns the `biguasim.make(...)` environment,
ticks it, publishes per-agent sensor topics, and accepts per-agent control
commands. This is the piece that `biguasim_bridge` / `biguasim_sensor_check` /
`biguasim_bringup` were designed around but never shipped with (see the repo
root `BIGUASIM.md`, §2, which names it as missing and points at an external
`biguasim_main`). It was found already implemented in a separate local repo,
`biguasim-ros2-develop`, and is merged here unchanged in architecture, with
the fixes below.

```bash
ros2 launch biguasim_main biguasim_main.launch.py
```

## Node Reference: `biguasim_node`

Loads a scenario from a YAML params file (a `biguasim_scenario:` key,
found recursively — see `config/config.yaml`), starts BiguaSim, and ticks it
on a ROS timer at `get_time_warp_period()` (`frames_per_sec` / `ticks_per_sec`
ratio from the scenario).

### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `<agent>/command_control` | `std_msgs/Float64MultiArray` | Raw command vector for one agent, one subscription per agent found in the scenario. Length/order depends on that agent's `control_abstraction` (see Known gaps below) — **not** the typed `AgentCommand`/Fossen topics HoloOcean uses. |

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `<agent>/<sensor_name>` | varies | Sensor data per agent, one publisher per sensor in the scenario (see `sensor_data_encode.py`) |
| `/clock` | `rosgraph_msgs/Clock` | Sim time, published from `state['t']` if present — **added in this merge**, not present in the original biguasim-ros2-develop version. Logs a warning once and stops trying if `'t'` is absent from the state dict. |

### Services

| Service | Type | Description |
|---|---|---|
| `reset` | `std_srvs/Trigger` | Calls `env.reset()` — **added in this merge**. Relies on `env.reset()` existing, which is confirmed by the example scenario script in the repo root `BIGUASIM.md` (`with biguasim.make(...) as env: env.reset()`), not independently re-verified against a running BiguaSim install. |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `params_file` | string | `""` | Full path to a YAML file containing a `biguasim_scenario` key |

## What changed vs. the original `biguasim-ros2-develop` version

1. **Fixed `ImagingSonar` geometry fields.** This workspace's `biguasim_interfaces/msg/ImagingSonar.msg`
   (already present under `biguasim_interfaces/`, from the pre-existing `biguasim_bridge` package)
   has four extra fields — `range_min`, `range_max`, `azimuth_aperture_deg`, `elevation_aperture_deg`
   — that `biguasim_bridge/sonar_bridge_node.py` reads to build its Cartesian fan image. The
   original `ImagingSonarEncoder` never set them (its own copy of the message didn't have them).
   `sensor_data_encode.py`'s `ImagingSonarEncoder` now fills them from the sensor's own
   `configuration` block (`RangeMin`/`RangeMax`/`Azimuth`/`Elevation`), which is already present
   in `config/config.yaml`. **This bridge was previously silently broken if pointed at
   `biguasim_bridge`'s message definition** — the fan would always render as if `range_max=0`.
2. **Fixed a dead `multi_publisher_sensors['IMUSensor']` entry.** The original had
   `'IMUSensor': ['', 'Bias']`, but no `'IMUSensorBias'` encoder was ever registered in the
   `encoders` dict — any scenario with an `IMUSensor` would have raised `TypeError:
   'NoneType' object is not callable` at startup. Removed the dangling `'Bias'` suffix
   (no `IMUBiasEncoder` port — see Known gaps).
3. **Fixed three no-op `ValueError(...)` statements** in `interface.py`
   (`get_tick_rate`/`get_frame_rate`/`get_time_warp`) that constructed an exception but never
   raised it, so a missing `ticks_per_sec`/`frames_per_sec` in the scenario would silently
   return `None` instead of failing loudly. Now `raise`d.
4. **Added `/clock` and the `reset` service** (see tables above), mirroring `holoocean_node.py`.
5. **Added a clear error instead of a silent `None` crash** in `create_sensor_list()` when a
   sensor type has no registered encoder (`encoders.get(full_type)` returning `None`) — now
   raises a `KeyError` naming the missing encoder instead of crashing later inside
   `encoder_class(sensor_copy)` with an unhelpful traceback.

## Known gaps (intentionally not ported / not guessed)

- **No `IMUBiasEncoder`.** HoloOcean's version reads a 4-row `sensor_data` array (rows 2/3 =
  accel/gyro bias) that only appears when its IMU sensor config sets `ReturnBias`. Whether
  BiguaSim's `IMUSensor` exposes an equivalent bias row/flag isn't verifiable without a
  BiguaSim install or docs — porting it blind risks silently wrong bias values, so it's left
  out. Add it back once the state layout is confirmed.
- **No Fossen-style autopilot** (`depth`/`heading`/`speed` setpoint topics). BiguaSim agents use
  a per-agent `control_abstraction` (`cmd_motor_speeds`, `cmd_depth_heading_rpm_surge`, ...)
  instead of HoloOcean's fixed Fossen dynamics model — there is nothing here to translate a
  depth/heading/speed setpoint into. `COMMAND_MAP` in `interface.py` only encodes vector
  *lengths* per abstraction, not the meaning of each index — that ordering is not documented
  anywhere available in this workspace. See `../biguasim_examples/README.md` for how the
  example nodes handle this.
- **No `draw_arrow`/`draw_debug_points`** (Unreal viewport debug drawing) — no evidence in the
  code that BiguaSim's `env` exposes equivalent calls.
- **`show_viewport`/`draw_arrow` in `config.yaml`** are scenario keys carried over from the
  original, but `interface.py` never reads them — unclear whether `biguasim.make()` consumes
  them itself or whether they're inert. Not verified.
