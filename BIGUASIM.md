# Installing and Running BiguaSim

BiguaSim is not included in this repository. It is installed manually inside
the `biguasim/` folder.

## 1. Install BiguaSim

Inside the container:

```bash
git clone https://github.com/hydrone-furg/biguasim.git ~/biguasim
cd ~/biguasim
pip install -e . --break-system-packages
```

Install the required world:

```bash
python3 -c "import biguasim; biguasim.install('SkyDive')"
```

## 2. Run BiguaSim with Python

BiguaSim can be executed directly from Python.

Example:

```python
import biguasim

cfg = {
    "package_name": "SkyDive",
    "world": "Pier-Harbor",
    "main_agent": "auv0",
    "ticks_per_sec": 20,
    "agents": [{
        "agent_name": "auv0",
        "agent_type": "TorpedoAUV",
        "sensors": [
            {"sensor_type": "DynamicsSensor"},
            {"sensor_type": "IMUSensor"},
            {"sensor_type": "DVLSensor"},
            {"sensor_type": "DepthSensor"},
            {"sensor_type": "ImagingSonar"},
        ],
        "control_abstraction": "cmd_depth_heading_rpm_surge",
        "location": [0, 0, -3],
        "rotation": [0, 0, -90],
    }],
}

with biguasim.make(scenario_cfg=cfg) as env:
    env.reset()

    for _ in range(2000):
        state = env.step([5, 45, 1000, 1])
        print(state["auv0"][0])
```

Save the script, for example:

```bash
python3 ~/biguasim/run_scenario.py
```

This runs BiguaSim and its sensors directly in Python.

## 3. Use BiguaSim with ROS 2

Build the ROS 2 workspace:

```bash
cd ~/workspaces/biguasim_ws
source env.sh
colcon build --symlink-install
source install/setup.bash
```

Start the BiguaSim → ROS 2 interface:

```bash
ros2 launch biguasim_main biguasim.launch.py
```

Then check the sensor topics:

```bash
ros2 launch biguasim_sensor_check sensor_check.launch.py
```

Optional: visualize the sensor data:

```bash
ros2 launch biguasim_bridge biguasim_bridge.launch.py
```

### Important

BiguaSim itself is a Python API and does not provide a ROS 2 launch file.
The `biguasim_main` package is responsible for connecting BiguaSim to ROS 2.
