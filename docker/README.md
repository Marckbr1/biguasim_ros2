# biguasim-ros2

Docker environment for running BiguaSim with ROS 2 Jazzy.

## Build

```bash
cd docker
./build_image.sh
```

## Run

First time:

```bash
./run_container.sh
```

Next sessions:

```bash
./start_container.sh
```

Enter the container:

```bash
./enter_container.sh
```

## BiguaSim

BiguaSim is installed manually inside the container.

```bash
git clone https://github.com/hydrone-furg/biguasim.git ~/biguasim
cd ~/biguasim
pip install -e . --break-system-packages
```

## Stop and remove

```bash
docker stop biguasim-ros2
docker rm biguasim-ros2
```

The `workspaces/` and `biguasim/` folders are mounted from the host, so their contents are preserved when the container is removed.
