# biguasim_sensor_check

Verifies the **BiguaSim ↔ ROS 2 connection**: subscribes to the simulator's
sensor topics, measures each one's rate, and prints a status table.

```bash
ros2 launch biguasim_sensor_check sensor_check.launch.py
```

```
topic                                            type                                        msgs     rate    last status
------------------------------------------------------------------------------------------------------------------------
/biguasim/rov0_id0/DynamicsSensor/Odom           nav_msgs/msg/Odometry                       1240   30.1Hz   0.03s OK
/biguasim/rov0_id0/ImagingSonar                  biguasim_interfaces/msg/ImagingSonar         207    5.0Hz    0.19s OK
/biguasim/rov0_id0/IMUSensor                     sensor_msgs/msg/Imu                            0     --       --    SILENT
------------------------------------------------------------------------------------------------------------------------
-> 6/7 sensors OK
```

Status: `OK` · `LOW` (< 50 % of expected Hz) · `STALE` (no msg for
`stale_after` s) · `SILENT` (never received) · `ERR` (message type not
found — build/​source `biguasim_interfaces`).

## Parameters

| Param | Default | |
|---|---|---|
| `sensors` | the 7 BiguaSim sensor topics | list of `"topic\|type\|expected_hz"` |
| `report_period` | `2.0` | seconds between tables |
| `window` | `5.0` | sliding window for the rate estimate |
| `stale_after` | `2.0` | no message for this long → `STALE` |
| `exit_after` | `0.0` | if `>0`, print one final table after N s and exit (rc `0` if all OK, else `1`) — for scripts/CI |

Custom topic list:

```bash
ros2 run biguasim_sensor_check sensor_check_node --ros-args \
  -p 'sensors:=["/biguasim/rov0_id0/IMUSensor|sensor_msgs/msg/Imu|30",
                "/biguasim/rov0_id0/ImagingSonar|biguasim_interfaces/msg/ImagingSonar|5"]'
```
