# biguasim_ws

ROS 2 workspace for the **BiguaSim ↔ ROS 2 interface**. Minimal on purpose:
bring the simulator up and confirm its sensors are publishing.

```
biguasim_ws/
├── env.sh                     # exports BIGUASIM_WS
└── src/
    ├── biguasim_interfaces/   # custom msgs (ImagingSonar, DVLSensorRange),
    │                          #   recreated from a recorded bag schema.
    │                          #   Delete once the upstream package is installed.
    ├── biguasim_bridge/       # polar/custom msgs -> standard ROS 2 types for RViz
    ├── biguasim_sensor_check/ # subscribe to every sensor, print a rate/status table
    └── biguasim_bringup/      # one launch: BiguaSim world + bridge + checker (+ RViz)
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
source env.sh
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Use

```bash
# everything at once (BiguaSim world + bridge + sensor checker + RViz):
export BIGUASIM_CMD="ros2 run <biguasim_pkg> <sim_executable>"   # once
ros2 launch biguasim_bringup biguasim.launch.py world:=<world_name> rviz:=true
```

Pieces separately:

```bash
ros2 launch biguasim_sensor_check sensor_check.launch.py   # just the rate/status table
ros2 launch biguasim_bridge biguasim_bridge.launch.py      # just the normalizers
```

Recorded bag, no live sim:

```bash
ros2 launch biguasim_bringup biguasim.launch.py run_sim:=false use_sim_time:=true &
ros2 bag play <bag> --clock
```

## Expected sensor topics

| Topic | Type | ~Hz |
|---|---|---|
| `/biguasim/rov0_id0/DynamicsSensor/Odom` | `nav_msgs/Odometry` | 30 |
| `/biguasim/rov0_id0/ImagingSonar` | `biguasim_interfaces/ImagingSonar` | 5 |
| `/biguasim/rov0_id0/IMUSensor` | `sensor_msgs/Imu` | 30 |
| `/biguasim/rov0_id0/DVLSensor/Velocity` | `geometry_msgs/TwistWithCovarianceStamped` | 5 |
| `/biguasim/rov0_id0/DVLSensor/Range` | `biguasim_interfaces/DVLSensorRange` | 5 |
| `/biguasim/rov0_id0/DepthSensor` | `geometry_msgs/PoseWithCovarianceStamped` | 30 |
| `/biguasim/rov0_id0/MagnetometerSensor` | `sensor_msgs/MagneticField` | 30 |

Adjust names/types to your BiguaSim build via the `sensors` parameter of
`biguasim_sensor_check` (its README).
