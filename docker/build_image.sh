#!/bin/bash
# Builds the biguasim-ros2 Docker image.
#
# Usage:
#   ./build_image.sh [tag] [extra docker build args...]
#
# Re-run whenever docker/Dockerfile or docker/build.sh change. NOT needed
# for workspace edits (bind-mounted) nor for BiguaSim updates (cloned +
# installed by hand inside the container).
set -e

IMAGE_NAME="biguasim-ros2"
TAG="${1:-latest}"
shift || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Building ${IMAGE_NAME}:${TAG} ..."
echo "Build context: ${SCRIPT_DIR}"

docker build \
    --build-arg UID="$(id -u)" \
    --build-arg GID="$(id -g)" \
    "$@" \
    -t "${IMAGE_NAME}:${TAG}" \
    "${SCRIPT_DIR}"

echo "Done: ${IMAGE_NAME}:${TAG}"
