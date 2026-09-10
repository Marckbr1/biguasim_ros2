# biguasim-ros2

Reproducible **ROS 2 Jazzy** Docker environment to run **BiguaSim** and
**verify its sensors over ROS 2** — i.e. test the BiguaSim ↔ ROS 2
connection. Same layout style as `marckloc-aracati-ros2` (image = system
only; workspaces bind-mounted), trimmed to this one job.

```
biguasim-ros2/
├── docker/                     # image + host/container scripts  (see docker/README.md)
├── biguasim/                   # you clone + install BiguaSim here (gitignored, empty on checkout)
├── BIGUASIM.md                 # how to install BiguaSim into biguasim/ + how to run it
├── LICENSE                     # BSD-3-Clause
└── workspaces/
    └── biguasim_ws/            # the ROS 2 interface workspace  (see its README.md)
        └── src/
            ├── biguasim_interfaces/    # custom msgs (ImagingSonar, DVLSensorRange)
            ├── biguasim_bridge/        # polar/custom msgs -> standard types for RViz
            └── biguasim_sensor_check/  # rate/status table for every sensor topic
```

- **Image** (`docker/Dockerfile`): `osrf/ros:jazzy-desktop` + build tools +
  non-root user **`biguauser`** (host UID/GID) + cv_bridge / image_transport
  / numpy / rosbag2. No BiguaSim, no workspace source baked in. No nav2 —
  this env only checks sensors.
- **BiguaSim**: not bundled and not installed automatically. After you
  create the container you clone and `pip install -e` it **by hand**, once,
  into the bind-mounted `biguasim/` folder — full steps in
  [`BIGUASIM.md`](BIGUASIM.md). That folder is mounted at
  `/home/biguauser/biguasim`, so the install survives `docker rm` and stays
  editable from the host.
- **No sim bringup here**: BiguaSim has no ROS 2 launch of its own (it is a
  Python API). This repo only provides the sensor **check** and **bridge**;
  getting the sensor topics onto ROS 2 (a Python scenario script, a bag
  replay, or porting `biguasim_main` from `marckloc-localization`) is
  described in `BIGUASIM.md` §2.

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
# 3. install BiguaSim BY HAND into ~/biguasim (once) -- see BIGUASIM.md
git clone https://github.com/hydrone-furg/biguasim.git ~/biguasim
cd ~/biguasim
pip install -e . --break-system-packages
python3 -c "import biguasim; biguasim.install('SkyDive')"   # world assets, several GB

# 4. build the interface workspace
cd ~/workspaces/biguasim_ws
source env.sh
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

# 5. get BiguaSim's sensors onto ROS 2 -- BIGUASIM.md §2
#    (Python scenario script + a bridge, or `ros2 bag play <bag> --clock`)

# 6. verify the sensors are flowing
ros2 launch biguasim_sensor_check sensor_check.launch.py
```

`--break-system-packages` is required on Ubuntu 24.04 / Python 3.12 (PEP
668). The clone in `~/biguasim` is on the host (bind mount), so it survives
`docker rm` + `run_container.sh`; re-run only `pip install -e` after
recreating the container. Full detail — including a runnable scenario
example — is in [`BIGUASIM.md`](BIGUASIM.md).

## Notes

- `biguasim_interfaces` here is a **local re-creation** of the sim's custom
  messages (from a recorded bag schema). If a BiguaSim install ever provides
  the real `biguasim_interfaces` as a ROS 2 package, delete this copy so
  there is one definition.
- GPU (`--gpus all`) is requested for BiguaSim's Unreal renderer; `/dev/dri`
  is passed so RViz2 gets hardware OpenGL; `/dev/input` + host `input` group
  for a USB joystick.
