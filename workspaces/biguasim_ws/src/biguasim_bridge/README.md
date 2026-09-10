# biguasim_bridge

Normalizes BiguaSim's custom / polar sensor messages into standard ROS 2
types you can view in RViz2.

| Node | In | Out |
|---|---|---|
| `odom_bridge_node` | `/biguasim/rov0_id0/DynamicsSensor/Odom` (`nav_msgs/Odometry`) | `/pose_gt` (`PoseStamped`), `/cmd_vel` (`TwistStamped`) |
| `sonar_bridge_node` | `/biguasim/rov0_id0/ImagingSonar` (`biguasim_interfaces/ImagingSonar`, polar 32FC1) | `/son` (`sensor_msgs/Image`, mono8 Cartesian fan) |

```bash
ros2 launch biguasim_bridge biguasim_bridge.launch.py
# then, in RViz2: add an Image display on /son
```

All topic names, frames and the sonar geometry conventions
(`range_axis_near_first`, `azimuth_flip`, `intensity_scale`, …) are node
parameters — see each node's docstring.
