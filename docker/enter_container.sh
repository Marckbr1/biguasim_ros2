#!/bin/bash
# Opens an extra interactive shell in the already-running biguasim-ros2
# container (started via run_container.sh / start_container.sh in another
# terminal). ROS 2 + workspace overlays + BiguaSim env are set up via
# .bashrc.
set -e

CONTAINER_NAME="biguasim-ros2"

if [ -z "$(docker ps -q -f name="^${CONTAINER_NAME}$")" ]; then
    echo "Container '${CONTAINER_NAME}' is not running. Start it with start_container.sh."
    exit 1
fi

docker exec -it "${CONTAINER_NAME}" bash
