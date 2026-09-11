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
import biguasim, cv2
import numpy as np

config = {
    "package_name": "SkyDive",
    "world": "Pier-Harbor",
    "main_agent": "uav0",
    "agents":[
        {
            "agent_name": "uav0",
            "agent_type": "DjiMatrice",
            "sensors": [
                {
                    "sensor_type": "DynamicsSensor",
                    "socket": "IMUSocket",
                    "configuration": {
                        "UseCOM": True,
                        "UseRPY": False
                    }
                },
                {
                    "sensor_type": "RGBCamera",
                    "sensor_name": "RGBCamera",
                    "socket": "CameraSocket",
                    "Hz": 5,
                    "configuration": {
                        "CaptureWidth": 512,
                        "CaptureHeight": 512
                    }
                }
            ],
            "dynamics" : {
                "batch_size" : 1,
            },
            "control_abstraction": 'cmd_vel',
            "location" : [ -21, -136, 15],
            "rotation": [0.0, 0.0, 0.0]
        }
    ],
}

env = biguasim.make(scenario_cfg = config)
command = [0, 0, 10]

for _ in range(200):
    state = env.step(command)["uav0"][0]
    if "RGBCamera" in state:
        pixels = state["RGBCamera"]
        frame = pixels[:, :, 0:3].astype(np.uint8)

        cv2.imshow("Camera Output", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
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
