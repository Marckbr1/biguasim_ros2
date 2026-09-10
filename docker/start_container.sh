#!/bin/bash
# Starts the already-created (but stopped) biguasim-ros2 container and
# attaches to it. Use this for every session after the first (which uses
# run_container.sh to create the container).
set -e

CONTAINER_NAME="biguasim-ros2"

if [ -z "$(docker ps -aq -f name="^${CONTAINER_NAME}$")" ]; then
    echo "Container '${CONTAINER_NAME}' does not exist yet. Run run_container.sh first."
    exit 1
fi

xhost +local:docker >/dev/null 2>&1 || true
docker start -ai "${CONTAINER_NAME}"
