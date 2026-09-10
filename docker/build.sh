#!/bin/bash
# Container entrypoint. Copied into the image by the Dockerfile and run
# automatically on every container start (ENTRYPOINT) -- you should not need
# to run it by hand.
#
# It only prepares the shell environment (sources ROS 2, every already-built
# workspace overlay under workspaces/, and BiguaSim if installed; exports a
# few standard ROS/GUI variables). It does NOT build anything: `colcon
# build` and install_biguasim.sh are explicit one-time steps you run inside
# the container so start/stop stays fast.
set -e

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Compiled overlay of every workspace under workspaces/ that has been built
# (an unbuilt workspace just has no install/setup.bash yet -> skipped).
for overlay in /home/biguauser/workspaces/*/install/setup.bash; do
    [ -f "$overlay" ] && source "$overlay"
done

# BiguaSim environment, if install_biguasim.sh has been run and produced one.
[ -f /home/biguauser/biguasim/setup.bash ] && source /home/biguauser/biguasim/setup.bash || true

# Standard ROS env vars (":=" respects values passed via `docker run -e`).
: "${ROS_DOMAIN_ID:=0}"
: "${RCUTILS_COLORIZED_OUTPUT:=1}"
: "${ROS_LOG_DIR:=/home/biguauser/.ros/log}"
export ROS_DOMAIN_ID RCUTILS_COLORIZED_OUTPUT ROS_LOG_DIR
mkdir -p "$ROS_LOG_DIR"

# XDG_RUNTIME_DIR is required by GUI tools (RViz2, any OpenCV window).
export XDG_RUNTIME_DIR=/tmp/runtime-$(id -u)
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

# exec (not a child process) so `docker stop` / Ctrl+C reach the real
# process and PID 1 stays this script.
exec "$@"
