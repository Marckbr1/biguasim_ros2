# Installing and running BiguaSim

BiguaSim itself is **not part of this repository**. It is its own git repo
(`github.com/hydrone-furg/biguasim`) and you install it **by hand, once**,
into the `biguasim/` folder at the repo root.

`biguasim/` is git-ignored here (`/biguasim/` in `.gitignore`) and
**bind-mounted into the container** at `/home/biguauser/biguasim` by
`docker/run_container.sh`. Because it is a host-side bind mount, the clone
and the downloaded world assets survive `docker rm` + `run_container.sh`.

---

## 1. Install (inside the container)

BiguaSim is a plain Python package (Unreal Engine 5 client, OpenAI-Gym-style
API). Python **≥ 3.11** — the `osrf/ros:jazzy-desktop` base already has 3.12.

```bash
git clone https://github.com/hydrone-furg/biguasim.git ~/biguasim
cd ~/biguasim
pip install -e . --break-system-packages      # pip falls back to ~/.local

# world assets (several GB) — required to actually load a scenario
python3 -c "import biguasim; biguasim.install('SkyDive')"
```

Notes:
- `--break-system-packages` is required on Ubuntu 24.04 / Python 3.12
  (PEP 668). `pip` prints *"Defaulting to user installation…"* and installs
  the editable link into `~/.local`, which lives in the **container layer**
  (not this mount). After recreating the container, re-run only
  `pip install -e . --break-system-packages`.
- The clone itself and `~/.local/share/biguasim/worlds/` (SkyDive assets)
  are on the host — you do **not** re-download them after `docker rm`.
- BiguaSim needs the GPU (`--gpus all`, already in `run_container.sh`) and,
  for a visible window, X11 (also already forwarded).

---

## 2. Run BiguaSim ("launch de ejecución")

**BiguaSim has no ROS 2 launch file of its own.** It is driven entirely from
Python via `biguasim.make(scenario_cfg=...)`. This repo (`biguasim-ros2`)
deliberately does **not** ship a sim-bringup node — it only provides:

| Package | Command | Purpose |
|---|---|---|
| `biguasim_sensor_check` | `ros2 launch biguasim_sensor_check sensor_check.launch.py` | rate/status table of the expected sensor topics |
| `biguasim_bridge` | `ros2 launch biguasim_bridge biguasim_bridge.launch.py` | normalize sonar + odom into standard RViz types |

Both of those assume **something is already publishing** the BiguaSim sensor
topics. You have two ways to get that:

### Option A — run BiguaSim from a Python script (no ROS)

Minimal scenario runner (adapt agent/sensors/world to your case):

```python
# ~/biguasim/run_scenario.py
import biguasim

cfg = {
    "package_name": "SkyDive",
    "world": "Pier-Harbor",
    "main_agent": "auv0",
    "ticks_per_sec": 20,
    "agents": [{
        "agent_name": "auv0",
        "agent_type": "TorpedoAUV",
        "sensors": [
            {"sensor_type": "DynamicsSensor"},
            {"sensor_type": "IMUSensor"},
            {"sensor_type": "DVLSensor"},
            {"sensor_type": "DepthSensor"},
            {"sensor_type": "ImagingSonar"},
        ],
        "control_abstraction": "cmd_depth_heading_rpm_surge",
        "location": [0, 0, -3],
        "rotation": [0, 0, -90],
    }],
}

with biguasim.make(scenario_cfg=cfg) as env:
    env.reset()
    for _ in range(2000):
        state = env.step([5, 45, 1000, 1])   # depth, heading, rpm, surge
        # state["auv0"][0]["DVLSensor"] ... etc.
```

```bash
python3 ~/biguasim/run_scenario.py
```

This gives you the sim + sensor data **in Python only**. Nothing reaches
ROS 2, so `biguasim_sensor_check` will report every topic `SILENT`.

### Option B — bridge BiguaSim → ROS 2 topics

To feed `biguasim_sensor_check` / `biguasim_bridge` / RViz you need a node
that ticks `biguasim.make(...)` and republishes each `state[...]` entry as
the matching ROS message (see the expected topic/type table in
`workspaces/biguasim_ws/README.md`).

That node is **not part of this repo yet**. The reference environment
`marckloc-localization` implements it as the `biguasim_main` package
(`ros2 launch biguasim_main biguasim.launch.py`) — port it here if you need
a full ROS 2 pipeline, or record a bag once and replay it:

```bash
ros2 bag play <bag> --clock
ros2 launch biguasim_sensor_check sensor_check.launch.py use_sim_time:=true
```

---

## 3. Full check, once the topics are flowing

```bash
cd ~/workspaces/biguasim_ws
source env.sh
colcon build --symlink-install
source install/setup.bash

ros2 launch biguasim_sensor_check sensor_check.launch.py     # sensors OK?
ros2 launch biguasim_bridge biguasim_bridge.launch.py        # optional: RViz-ready
```
