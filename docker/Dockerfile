# biguasim-ros2 development image
#
# OS-level environment only (ROS 2 Jazzy + build tools + a non-root user
# `biguauser` with sudo). It deliberately does NOT bake in workspace source
# code nor BiguaSim itself:
#
#   - The workspace lives in this repo's workspaces/ folder on the host and
#     is bind-mounted at runtime (see run_container.sh) at
#     /home/biguauser/workspaces -- editing on the host is visible inside
#     the container immediately, no image rebuild.
#   - BiguaSim is a fast-moving research dependency: YOU clone and install it
#     by hand ONCE inside the running container, after creating it (see the
#     "Installing BiguaSim" section of the READMEs). run_container.sh
#     bind-mounts <repo>/biguasim -> /home/biguauser/biguasim so that clone
#     persists across `docker rm` and is editable from the host. Keeping it
#     out of the image means updating it is `git pull` in place, not a
#     rebuild.
#
# Purpose of this environment: bring up BiguaSim + ROS 2 and verify the
# simulator's sensor topics are flowing (biguasim_sensor_check), i.e. test
# the BiguaSim <-> ROS 2 connection. Simple on purpose.
#
# Build with build_image.sh (needs --build-arg UID/GID to match your host
# user -- see that script; do not call `docker build` directly).

FROM osrf/ros:jazzy-desktop

LABEL project="biguasim-ros2" \
      description="ROS 2 Jazzy env to run BiguaSim and verify its sensors over ROS 2"

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

ARG UID=1000
ARG GID=1000

# Build tools for ROS 2 workspaces + cloning/installing BiguaSim (git, pip),
# plus the few ROS 2 packages the sensor-interface workspace needs that are
# not in the desktop base:
#   - cv-bridge / image-transport(+plugins) : the sonar bridge decodes the
#     ImagingSonar sub-images and republishes a sensor_msgs/Image.
#   - python3-numpy : biguasim_bridge (polar->fan remap) and sensor_check.
#   - mesa-utils : `glxinfo -B` to confirm RViz2 got hardware GL.
# (No nav2 here -- this environment only checks sensors, it does not
# localize.)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    wget \
    sudo \
    build-essential \
    python3-pip \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-numpy \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-storage-mcap \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Non-root user `biguauser`, UID/GID parameterized to match the host user so
# files written on the bind-mounted workspace (colcon build/install/log, the
# BiguaSim clone) are not owned by a foreign UID. osrf/ros:jazzy-desktop
# ships a default "ubuntu" user at 1000 -> remove whatever occupies the
# target UID/GID first, then create biguauser cleanly.
RUN existing_user="$(getent passwd "${UID}" | cut -d: -f1)"; \
    if [ -n "$existing_user" ] && [ "$existing_user" != "root" ]; then userdel -r "$existing_user" 2>/dev/null || userdel "$existing_user"; fi; \
    existing_group="$(getent group "${GID}" | cut -d: -f1)"; \
    if [ -n "$existing_group" ]; then groupdel "$existing_group" 2>/dev/null || true; fi; \
    groupadd -g "${GID}" biguauser && \
    useradd -m -u "${UID}" -g "${GID}" -s /bin/bash biguauser && \
    usermod -aG sudo biguauser && \
    echo "biguauser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/biguauser && \
    chmod 440 /etc/sudoers.d/biguauser

# Joystick / GPU groups: run_container.sh passes the real host GIDs via
# --group-add at runtime; here we only make sure the groups exist and
# biguauser is a member so name lookups succeed.
RUN for grp in input video render; do getent group "$grp" >/dev/null || groupadd "$grp"; done; \
    usermod -aG input,video,render biguauser

# rosdep database initialized at build time (does not depend on the
# workspace, which is mounted at runtime).
RUN rosdep init 2>/dev/null || true; \
    for i in 1 2 3 4 5; do rosdep update && break || { echo "rosdep update retry $i ..."; sleep 10; }; done; true

COPY build.sh /build.sh
RUN chmod +x /build.sh

USER biguauser
WORKDIR /home/biguauser

# Extra interactive shells (`docker exec ... bash`) get the same env.
# (BiguaSim is a pip editable install -- importable with no sourcing.)
RUN printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    'for o in /home/biguauser/workspaces/*/install/setup.bash; do [ -f "$o" ] && source "$o"; done' \
    >> /home/biguauser/.bashrc

ENTRYPOINT ["/build.sh"]
CMD ["bash"]
