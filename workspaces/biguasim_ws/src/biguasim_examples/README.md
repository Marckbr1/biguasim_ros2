# biguasim_examples

Example teleoperation/visualization nodes for BiguaSim, ported from
`holoocean_examples`. Requires `biguasim_main` (the core simulation bridge)
to be running — these nodes are ROS-side clients, they don't start BiguaSim
themselves.

| Launch file | Description |
|---|---|
| `joy_launch.py` | `biguasim_main` + `joy_linux` + `joy_biguasim` + `camera_hud` |
| `waypoint_launch.py` | `biguasim_main` + `waypoint_follower` |
| `camera_hud_launch.py` | `camera_hud` alone (against an already-running `biguasim_main` or a bag) |

```bash
ros2 launch biguasim_examples joy_launch.py
ros2 launch biguasim_examples waypoint_launch.py
```

## The one thing you must configure before these actually drive a vehicle

HoloOcean's joystick/waypoint examples publish a **typed** `AgentCommand`
message; HoloOcean's own bridge (`holoocean_node`) and its Fossen dynamics
layer decide what each field means. **BiguaSim has no such typed command or
autopilot layer** — `biguasim_node` (in `biguasim_main`) expects one flat
`std_msgs/Float64MultiArray` per agent on `<agent>/command_control`, whose
length is fixed by that agent's `control_abstraction`
(`cmd_motor_speeds`, `cmd_depth_heading_rpm_surge`, ...), but **whose
per-index meaning is not documented anywhere available in this workspace**
(no BiguaSim install, no upstream docs reachable from here).

Rather than guess a thruster-mixing matrix or an axis order and risk
silently wrong control on a real/simulated vehicle, both `joy_biguasim` and
`waypoint_follower` ship with **safe no-op defaults** and an explicit
parameter you fill in once you've confirmed the real layout:

- **`joy_biguasim`**: parameter `axis_map.<agent_name>` — a list of joystick
  axis indices, one per output vector slot (`-1` = always `0.0`). Defaults
  to all `-1`.
- **`waypoint_follower`**: parameter `command_slots` — a list of tokens
  (`"surge"`, `"yaw_rate"`, `"zero"`, or a literal number), one per output
  vector slot. Defaults to `["zero"]` (single no-op slot).

### Confirmed example: `cmd_depth_heading_rpm_surge`

The repo root `BIGUASIM.md` (§1, "Option A") has a worked example that
names the order directly:

```python
state = env.step([5, 45, 1000, 1])   # depth, heading, rpm, surge
```

If your scenario's agent uses `control_abstraction: 'cmd_depth_heading_rpm_surge'`,
`waypoint_follower` can drive it with a confirmed (not guessed) mapping:

```yaml
command_slots: ['depth', 'heading_deg', '1000', 'surge']   # 1000 = a fixed cruise RPM
target_depth: 5.0
```

`joy_biguasim` doesn't have a depth/heading autopilot concept (it's a raw
axis passthrough), so this shortcut only applies to `waypoint_follower`.
Every other `control_abstraction` (`cmd_motor_speeds` — used by the shipped
`biguasim_main/config/config.yaml` example scenario — `cmd_vel`,
`cmd_vel_yaw`, `cmd_pos_yaw`, `cmd_rudders_sterns_motor_speed`, `accel`) is
still unverified index-by-index; use the procedure below for those.

### How to find the real mapping

1. Run `biguasim_main` against your scenario (`ros2 launch biguasim_main biguasim_main.launch.py`).
2. `ros2 topic echo <agent>/command_control` in one terminal.
3. Move **one** joystick axis at a time (with `axis_map` set to route it to
   a single slot) or manually `ros2 topic pub` single-slot test vectors, and
   watch which physical motion results in the sim / on `<agent>/DynamicsSensor/Odom`.
4. Fill in `axis_map`/`command_slots` once confirmed, and note it down
   somewhere durable (scenario docs, a comment in your launch config) — this
   mapping is scenario- and vehicle-specific, not a constant.

This mirrors how `biguasim_bridge/sonar_bridge_node.py` already handles its
own unverified geometry (`range_axis_near_first`, `azimuth_flip` — "not
documented and should be confirmed visually once the sim runs").

## `camera_hud` has no such caveat

`camera_hud` only reads `<agent>/RGBCamera` + `<agent>/DynamicsSensor/Odom`
and republishes an annotated image on `<agent>/CameraHUD` — it never issues
a command, so it's a direct, safe port. View it with:

```bash
ros2 run rqt_image_view rqt_image_view
# select /biguasim/<agent>/CameraHUD
```

## What was NOT ported from `holoocean_examples`

- **`command_node.py`** (depth/heading/speed setpoints via the Fossen
  autopilot) — BiguaSim has no Fossen-style autopilot to target; there is
  nothing for this to translate into. Skipped rather than faked.
- **`multi_joy_launch.py`** (two controllers, two agents simultaneously) —
  `joy_biguasim` already supports selecting among multiple agents by button
  (like HoloOcean's single-controller `joy_holoocean.py`), but simultaneous
  independent control from two physical controllers was not ported; add a
  second `joy_linux_node`/`joy_biguasim` pair under a different topic
  remap if you need it.
- **`RK45_state_est.py`** — in HoloOcean itself this script exists in the
  package but isn't wired into any launch file or referenced by the README;
  treated as out of scope here too.
- **Camera tilt via `SensorCommand`** — HoloOcean's joystick can rotate a
  agent's camera sensor at runtime (`rotate_sensor` on the HoloOcean
  interface). `biguasim_main/interface.py` (ported from
  `biguasim-ros2-develop`) has no equivalent method, so this was dropped
  rather than added speculatively.
