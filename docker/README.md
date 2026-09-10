# Docker environment for biguasim-ros2

Builds and runs a reproducible ROS 2 Jazzy container to launch **BiguaSim**
and verify its sensor topics over ROS 2. No workspace source and no BiguaSim
are baked into the image.

## Files

| File | Runs where | Purpose |
|---|---|---|
| `Dockerfile` | `docker build` | ROS 2 Jazzy base + build tools + `biguauser` (host UID/GID) + sonar/image deps |
| `build.sh` | container (automatic) | entrypoint: sources ROS 2 and the workspace overlays |
| `build_image.sh` | host | build the image |
| `run_container.sh` | host | create the container (`workspaces/` + `biguasim/` mounts, GPU, X11, `/dev/dri`, joystick) |
| `start_container.sh` | host | start the existing container |
| `enter_container.sh` | host | extra shell in the running container |

## Build

```bash
cd docker
./build_image.sh            # biguasim-ros2:latest
```

Re-run only when `Dockerfile` or `build.sh` change.

## Create / start / enter

```bash
./run_container.sh          # first time (creates 'biguasim-ros2')
./start_container.sh        # later sessions
./enter_container.sh        # extra shell
```

`run_container.sh` mounts this repo's `workspaces/` at
`/home/biguauser/workspaces` and `biguasim/` at `/home/biguauser/biguasim`,
requests the NVIDIA GPU (BiguaSim's Unreal renderer), forwards X11 +
`/dev/dri` (hardware GL for RViz2), and passes `/dev/input` + the host
`input` group (USB joystick).

## Install BiguaSim (by hand, inside the container, once)

BiguaSim is **not** installed by any script. After the container exists you
clone + `pip install -e` it yourself into the bind-mounted `~/biguasim`
folder, and separately get its sensors onto ROS 2 (it has no ROS launch of
its own). Full instructions, including a runnable scenario example:
[`../BIGUASIM.md`](../BIGUASIM.md).

Short version:

```bash
git clone https://github.com/hydrone-furg/biguasim.git ~/biguasim
cd ~/biguasim
pip install -e . --break-system-packages
python3 -c "import biguasim; biguasim.install('SkyDive')"   # world assets, several GB
```

`~/biguasim` is bind-mounted from `<repo>/biguasim` on the host, so the clone
and the world assets survive `docker rm`. After recreating the container,
re-run only `pip install -e . --break-system-packages` (the editable link
lives in the container layer, not the mount).

## Remove

```bash
docker stop biguasim-ros2 && docker rm biguasim-ros2
docker rmi biguasim-ros2:latest        # also drop the image
```

The `biguasim/` and `workspaces/*/build|install|log` folders on the host are
untouched by `docker rm`.
