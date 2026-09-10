#!/bin/bash
# Creates the biguasim-ros2 development container. Run ONCE per machine (or
# after `docker rm`); for day-to-day work use start_container.sh /
# enter_container.sh. Running this again while the container exists is a
# guarded no-op.
#
# What it sets up:
#   - Mounts <repo>/workspaces -> /home/biguauser/workspaces (single volume
#     for the whole folder; biguasim_ws/src is versioned with the repo,
#     build/ install/ log/ persist across restarts, gitignored).
#   - GPU (NVIDIA Container Toolkit) for BiguaSim's Unreal renderer.
#   - X11 + /dev/dri for RViz2 / GUI windows with hardware OpenGL.
#   - /dev/input + host "input" group so a USB joystick works
#     (`ros2 run joy joy_node`).
set -e

IMAGE_NAME="biguasim-ros2:latest"
CONTAINER_NAME="biguasim-ros2"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACES_DIR="${REPO_ROOT}/workspaces"
mkdir -p "$WORKSPACES_DIR"

if [ "$(docker ps -aq -f name="^${CONTAINER_NAME}$")" ]; then
    echo "Container '${CONTAINER_NAME}' already exists."
    echo "Use start_container.sh to start it, or enter_container.sh to open a shell in it."
    exit 1
fi

xhost +local:docker >/dev/null 2>&1 || \
    echo "Warning: could not run 'xhost +local:docker' (no X server on host?)"

GROUP_ADD_ARGS=()

# Joystick: pass the host "input" group so /dev/input/js* / event* are
# readable without root.
INPUT_GID="$(getent group input | cut -d: -f3)"
[ -n "$INPUT_GID" ] && GROUP_ADD_ARGS+=(--group-add "$INPUT_GID")
JOYSTICK_ARGS=()
if [ -d /dev/input ]; then
    JOYSTICK_ARGS=(-v /dev/input:/dev/input)
else
    echo "Warning: /dev/input not present -- joystick will not be available."
fi

# Hardware OpenGL for RViz2: without a DRM render node the container's Mesa
# falls back to llvmpipe (slow, miscompiles RViz's Map shader). /dev/dri +
# host video/render GIDs give it the same GPU that draws the host desktop.
# (--gpus all alone only provides the NVIDIA CUDA/GL userspace.)
DRI_ARGS=()
if [ -d /dev/dri ]; then
    DRI_ARGS=(-v /dev/dri:/dev/dri)
    for grp in video render; do
        gid="$(getent group "$grp" | cut -d: -f3)"
        [ -n "$gid" ] && GROUP_ADD_ARGS+=(--group-add "$gid")
    done
else
    echo "Warning: /dev/dri not present -- RViz2 will fall back to slow software GL."
fi

docker run -it \
    --name "${CONTAINER_NAME}" \
    --gpus all \
    --network host \
    --ipc host \
    -e DISPLAY="${DISPLAY}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${WORKSPACES_DIR}:/home/biguauser/workspaces" \
    "${DRI_ARGS[@]}" \
    "${JOYSTICK_ARGS[@]}" \
    "${GROUP_ADD_ARGS[@]}" \
    "${IMAGE_NAME}"
