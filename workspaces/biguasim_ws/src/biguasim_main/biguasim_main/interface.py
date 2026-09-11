"""BiguaSim <-> ROS 2 interface.

Owns the biguasim.make(...) environment, discovers agents/sensors from the
scenario config, ticks the sim, and turns each tick's state dict into ROS
messages via sensor_data_encode.encoders. Structural port of
holoocean_main/interface/holoocean_interface.py, adapted to BiguaSim's real
Python API (biguasim.make / env.step / env._dynamics_dict / per-agent
control_abstraction) instead of HoloOcean's (holoocean.make / env.tick /
env.act / FossenInterface). Originally authored in the biguasim-ros2-develop
repo; merged here unchanged apart from the ImagingSonar geometry fix
documented in sensor_data_encode.py.
"""
import biguasim
import json
import yaml

import numpy as np

from pathlib import Path

from biguasim_main.sensor_data_encode import encoders, multi_publisher_sensors

# control_abstraction -> command vector length. NOTE: this list is copied
# from the reference implementation (biguasim-ros2-develop); vector *length*
# has not been independently cross-checked against BiguaSim's own source
# (no BiguaSim install available here), and per-index *meaning* is unknown
# for most of these -- see biguasim_examples/README.md.
#
# CONFIRMED exception: 'cmd_depth_heading_rpm_surge' order is
# [depth, heading, rpm, surge] -- taken directly from the worked example in
# the repo root BIGUASIM.md ("Option A"):
#   state = env.step([5, 45, 1000, 1])   # depth, heading, rpm, surge
# This is real evidence (a comment in a runnable example script), not a
# guess. See waypoint_follower.py's `depth`/`heading_deg` command_slots
# tokens, which implement exactly this order.
COMMAND_MAP = {
    'accel': 6,
    'cmd_vel': 3,
    'cmd_vel_yaw': 4,
    'cmd_pos_yaw': 4,
    'cmd_rudders_sterns_motor_speed': 5,
    'cmd_depth_heading_rpm_surge': 4,  # confirmed order: [depth, heading, rpm, surge]
}

MOTOR_SPEEDS = {
    "BlueBoat": 2,
    "BlueROV2": 6,
    "BlueROVHeavy": 8,
    "DjiMatrice": 4,
}


class BiguaSimInterface():
    '''
    Class for abstracting the biguasim Python interface.
    Lists sensors and formats sensor data.
    '''

    def __init__(self, scenario_path, init=True, node=None):
        """
        Initialize the BiguaSim environment with a path to a YAML file
        containing a `biguasim_scenario` block. Create the sensor list.
        """
        self.node = node
        scenario_path = Path(scenario_path)

        scenario = self.parse_scenario_yaml(scenario_path)

        # TODO: expose as a ROS parameter instead of hardcoding system time.
        self.system_time = True

        self.r2b = str.maketrans('_', '-')
        self.b2r = str.maketrans('-', '_')

        self.command = dict()

        if init:
            self.env = biguasim.make(scenario_cfg=scenario)
            self.scenario = self.env._scenario
            self.initialized = True
            self.sensors = self.create_sensor_list()
        else:
            self.scenario = scenario
            self.initialized = False

    def _get_agent_id(self, agent_name: str):
        split = agent_name.find('_')
        name = agent_name[:split]
        idx = int(agent_name[split + 3:])
        return name, idx

    def _get_command_shape(self, agent_name, agent_type):
        control_abstraction = self.env._dynamics_dict[agent_name].control_abstraction
        if control_abstraction == 'cmd_motor_speeds':
            return MOTOR_SPEEDS[agent_type]
        return COMMAND_MAP[control_abstraction]

    def parse_scenario(self, path):
        file_path = Path(path)
        with file_path.open() as params_file:
            scenario = json.load(params_file)
        return scenario

    def find_biguasim_scenario(self, yaml_content):
        """Recursively search for 'biguasim_scenario' in the YAML content."""
        if isinstance(yaml_content, dict):
            for key, value in yaml_content.items():
                if key == "biguasim_scenario":
                    return value
                else:
                    result = self.find_biguasim_scenario(value)
                    if result is not None:
                        return result
        elif isinstance(yaml_content, list):
            for item in yaml_content:
                result = self.find_biguasim_scenario(item)
                if result is not None:
                    return result
        return None

    def parse_scenario_yaml(self, scenario_path):
        with open(scenario_path, 'r') as file:
            yaml_content = yaml.safe_load(file)

        biguasim_scenario_yaml = self.find_biguasim_scenario(yaml_content)

        if biguasim_scenario_yaml is None:
            raise KeyError("Could not find 'biguasim_scenario' in the YAML file.")

        return biguasim_scenario_yaml

    def create_sensor_list(self):
        scenario = self.scenario

        if len(scenario["agents"]) > 1:
            self.multi_agent_scenario = True
            print("Give sensors unique names that are reported on multiple agents")
        else:
            self.multi_agent_scenario = False

        sensors = []

        for agent in scenario["agents"]:
            agent_name = agent['agent_name'].translate(self.b2r)
            dynamics_name, _ = self._get_agent_id(agent_name)
            if dynamics_name not in self.command:
                batch_size = self.env._dynamics_dict[dynamics_name].batch_size
                cmd_shape = self._get_command_shape(dynamics_name, agent['agent_type'])

                if batch_size > 1:
                    self.command[dynamics_name] = np.zeros((batch_size, cmd_shape)).tolist()
                else:
                    self.command[dynamics_name] = np.zeros(cmd_shape).tolist()

            if "publish_commands" in agent and agent["publish_commands"] == True:
                encoder_class = encoders.get("ControlCommand")
                config = {}
                config['sensor_name'] = "ControlCommand"
                config['sensor_type'] = "ControlCommand"
                config['agent_name'] = agent_name
                config['state_name'] = 'ControlCommand'
                sensors.append(encoder_class(config))

            for sensor in agent["sensors"]:
                if sensor['ros_publish']:
                    sensor_type = sensor['sensor_type']

                    if sensor_type in multi_publisher_sensors:
                        for suffix in multi_publisher_sensors[sensor_type]:
                            full_type = f"{sensor['sensor_type']}{suffix}"
                            sensor_name = sensor['sensor_name'] if 'sensor_name' in sensor else sensor['sensor_type']
                            full_name = f"{sensor_name}/{suffix}" if suffix != "" else sensor_name

                            encoder_class = encoders.get(full_type)
                            if encoder_class is None:
                                raise KeyError(
                                    f"No encoder registered for '{full_type}' "
                                    f"(sensor_type={sensor_type!r}, suffix={suffix!r}). "
                                    "Check multi_publisher_sensors / encoders in sensor_data_encode.py."
                                )
                            sensor_copy = sensor.copy()
                            sensor_copy['sensor_name'] = full_name
                            sensor_copy['agent_name'] = agent_name
                            sensor_copy['state_name'] = sensor_name
                            sensors.append(encoder_class(sensor_copy))
                    else:
                        sensor_copy = sensor.copy()
                        sensor_copy['agent_name'] = agent_name
                        sensor_copy['state_name'] = sensor['sensor_name'] if 'sensor_name' in sensor else sensor['sensor_type']
                        encoder = encoders[sensor['sensor_type']]
                        sensors.append(encoder(sensor_copy))

        return sensors

    def publish_sensor_data(self, state: dict):
        self._state = state.copy()
        for sensor in self.sensors:
            try:
                agent_name, idx = sensor.agent_name.split('_id')
                msg = sensor.encode(state[agent_name][int(idx)][sensor.state_name])

                if self.system_time:
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                else:
                    msg.header.stamp.sec = int(state['t'])
                    msg.header.stamp.nanosec = int((state['t'] - msg.header.stamp.sec) * 1e9)

                sensor.publisher.publish(msg)
            except KeyError:
                # Sensor data not available this tick (e.g. rate-limited sensor).
                pass
            except Exception as e:
                print(f"Error processing sensor: {sensor.name}, type: {sensor.type}, error: {str(e)}")

    def send_control_command(self, agent_name: str, command: list):
        dynamics_name, idx = self._get_agent_id(agent_name)

        if all(isinstance(x, list) for x in self.command[dynamics_name]):
            assert len(self.command[dynamics_name][idx]) == len(command)
            self.command[dynamics_name][idx] = command
        else:
            assert len(self.command[dynamics_name]) == len(command)
            self.command[dynamics_name] = command

    def tick(self):
        """
        Step the BiguaSim environment. Returns the resulting state dict.
        """
        cmd = self.command
        if len(self.command) == 1:
            cmd = self.command[next(iter(self.command))]

        state = self.env.step(cmd)

        return state

    def get_scenario(self):
        if self.initialized:
            return self.env._scenario
        else:
            return self.scenario

    def get_tick_rate(self):
        if self.initialized:
            return self.env._ticks_per_sec
        else:
            if "ticks_per_sec" in self.scenario:
                return self.scenario['ticks_per_sec']
            else:
                raise ValueError('ticks_per_sec not specified in scenario')

    def get_frame_rate(self):
        if self.initialized:
            return self.env._frames_per_sec
        else:
            if "frames_per_sec" in self.scenario:
                return self.scenario['frames_per_sec']
            else:
                raise ValueError('frames_per_sec not specified in scenario')

    def get_period(self):
        return 1.0 / self.get_tick_rate()

    def get_time_warp(self):
        time_warp = self.get_frame_rate() / self.get_tick_rate()
        if time_warp <= 0:
            raise ValueError("frames_per_sec cannot be 0 for time warping. Set a value > 0 ")
        return time_warp

    def get_time_warp_period(self):
        return self.get_period() / self.get_time_warp()
