# Docker environment for biguasim-ros2

Builds and runs a reproducible ROS 2 Jazzy container to launch **BiguaSim**
and verify its sensor topics over ROS 2. No workspace source and no BiguaSim
are baked into the image.

## Files

| File | Runs where | Purpose |
|---|---|---|
| `Dockerfile` | `docker build` | ROS 2 Jazzy base + build tools + `biguauser` (host UID/GID) + sonar/image deps |
| `build.sh` | container (automatic) | entrypoint: sources ROS 2, workspace overlays and BiguaSim env |
| `install_biguasim.sh` | inside the container (once) | clone + install BiguaSim from `$BIGUASIM_REPO` |
| `build_image.sh` | host | build the image |
| `run_container.sh` | host | create the container (workspaces/ mount, GPU, X11, `/dev/dri`, joystick) |
| `start_container.sh` | host | start the existing container |
| `enter_container.sh` | host | extra shell in the running container |

## Build

```bash
cd docker
./build_image.sh            # biguasim-ros2:latest
```

Re-run only when `Dockerfile` / `build.sh` / `install_biguasim.sh` change.

## Create / start / enter

```bash
./run_container.sh          # first time (creates 'biguasim-ros2')
./start_container.sh        # later sessions
./enter_container.sh        # extra shell
```

`run_container.sh` mounts this repo's `workspaces/` at
`/home/biguauser/workspaces`, requests the NVIDIA GPU (BiguaSim's Unreal
renderer), forwards X11 + `/dev/dri` (hardware GL for RViz2), and passes
`/dev/input` + the host `input` group (USB joystick).

## Install BiguaSim (inside the container, once)

```bash
BIGUASIM_REPO=https://github.com/<org>/<biguasim>.git install_biguasim.sh
```

The script clones to `~/biguasim/src`, auto-detects the install type
(ROS 2 package → link into `workspaces/biguasim_ws/src`; Python →
`pip install -e`; CMake → build), and writes `~/biguasim/setup.bash`
(sourced automatically on the next container entry). Edit that file to add
BiguaSim-specific exports (world path, sim host/port…).

## Remove

```bash
docker stop biguasim-ros2 && docker rm biguasim-ros2
docker rmi biguasim-ros2:latest        # also drop the image
```
