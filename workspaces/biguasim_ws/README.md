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
    └── biguasim_trajectory/   # move an agent in a square/circle (cmd_pos_yaw),
                                #   optionally with the sonar view turned on
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
# 1. install + start BiguaSim by hand (see the repo-root README.md
#    -> "Installing BiguaSim")

# 2. check the sensors are flowing
ros2 launch biguasim_sensor_check sensor_check.launch.py

# 3. (optional) normalize sonar + odom for RViz
ros2 launch biguasim_bridge biguasim_bridge.launch.py

# 4. (optional) move the robot in a square or circle -- see biguasim_trajectory/README.md
ros2 launch biguasim_trajectory trajectory_launch.py trajectory_type:=square
ros2 launch biguasim_trajectory trajectory_sonar_launch.py trajectory_type:=circle   # + sonar view
```

If you only have a recorded bag (no live sim):

```bash
ros2 bag play <bag> --clock
ros2 launch biguasim_sensor_check sensor_check.launch.py use_sim_time:=true
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
