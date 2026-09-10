# biguasim-ros2

Docker environment for running **BiguaSim with ROS 2 Jazzy** and checking
its sensor topics.

## Quick Start

### 1. Build the Docker image

```bash
cd docker
chmod +x *.sh
./build_image.sh
```

### 2. Create the container

```bash
./run_container.sh
```

### 3. Start the container

For later sessions:

```bash
./start_container.sh
```

### 4. Enter the container

```bash
./enter_container.sh
```

## Install BiguaSim

Inside the container:

```bash
git clone https://github.com/hydrone-furg/biguasim.git ~/biguasim
cd ~/biguasim
pip install -e . --break-system-packages
```

## Build the ROS 2 workspace

```bash
cd ~/workspaces/biguasim_ws
source env.sh
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Check sensors

```bash
ros2 launch biguasim_sensor_check sensor_check.launch.py
```

More details about BiguaSim and the ROS 2 connection are available in
[`BIGUASIM.md`](BIGUASIM.md).
