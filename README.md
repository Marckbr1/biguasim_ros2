# biguasim-ros2

Reproducible **ROS 2 Jazzy** Docker environment to run **BiguaSim** and
**verify its sensors over ROS 2** — i.e. test the BiguaSim ↔ ROS 2
connection. Same layout style as `marckloc-aracati-ros2` (image = system
only; workspaces bind-mounted), trimmed to this one job.

```
biguasim-ros2/
├── docker/                     # image + scripts + install_biguasim.sh  (see docker/README.md)
└── workspaces/
    └── biguasim_ws/            # the ROS 2 interface workspace  (see its README.md)
        └── src/
            ├── biguasim_interfaces/    # custom msgs (ImagingSonar, DVLSensorRange)
            ├── biguasim_bridge/        # polar/custom msgs -> standard types for RViz
            ├── biguasim_sensor_check/  # rate/status table for every sensor topic
            └── biguasim_bringup/       # one launch: BiguaSim world + bridge + checker (+ RViz)
```

- **Image** (`docker/Dockerfile`): `osrf/ros:jazzy-desktop` + build tools +
  non-root user **`biguauser`** (host UID/GID) + cv_bridge / image_transport
  / numpy / rosbag2. No BiguaSim, no workspace source baked in. No nav2 —
  this env only checks sensors.
- **BiguaSim**: cloned + built once inside the container with
  `install_biguasim.sh` (set `BIGUASIM_REPO`).

## Quick start

```bash
# 1. build the image (once, or when docker/ changes)
cd docker
chmod +x *.sh
./build_image.sh

# 2. create the container (once) — mounts workspaces/, GPU, X11, joystick
./run_container.sh

# later sessions:
./start_container.sh          # start
./enter_container.sh          # extra shell
```

Inside the container:

```bash
# 3. install BiguaSim (once)
BIGUASIM_REPO=https://github.com/<org>/<biguasim>.git install_biguasim.sh

# 4. build the interface workspace
cd ~/workspaces/biguasim_ws
source env.sh
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

# 5. tell the bringup how BiguaSim starts (once), e.g.:
export BIGUASIM_CMD="ros2 run <biguasim_pkg> <sim_executable>"

# 6. one command: BiguaSim world + bridge + sensor checker
ros2 launch biguasim_bringup biguasim.launch.py world:=<world_name> rviz:=true
```

Or run the pieces separately: `ros2 launch biguasim_sensor_check
sensor_check.launch.py` (just the checker), `ros2 launch biguasim_bridge
biguasim_bridge.launch.py` (just the normalizers).

If BiguaSim runs elsewhere / you only have a bag:

```bash
ros2 launch biguasim_bringup biguasim.launch.py run_sim:=false use_sim_time:=true &
ros2 bag play <bag> --clock
```

## Notes

- `biguasim_interfaces` here is a **local re-creation** of the sim's custom
  messages (from a recorded bag schema). If `install_biguasim.sh` brings in
  the real `biguasim_interfaces`, delete this copy so there is one
  definition.
- GPU (`--gpus all`) is requested for BiguaSim's Unreal renderer; `/dev/dri`
  is passed so RViz2 gets hardware OpenGL; `/dev/input` + host `input` group
  for a USB joystick.
