#!/usr/bin/env python3
"""Joystick -> BiguaSim Float64MultiArray command converter.

Ported from holoocean_examples/joy_holoocean.py, but the output shape is
fundamentally different: HoloOcean has a typed AgentCommand plus per-vehicle
thruster-mixing logic baked into joy_holoocean.py (e.g. 8-motor BlueROV2
mixing). BiguaSim instead expects one flat std_msgs/Float64MultiArray per
agent on `<agent>/command_control`, whose length and per-index meaning
depend on that agent's `control_abstraction` (see biguasim_main/interface.py
COMMAND_MAP / MOTOR_SPEEDS) -- and that per-index meaning (which slot is
surge, which is yaw, whether it's raw per-motor RPM, ...) is NOT documented
anywhere available in this workspace (no BiguaSim install, no upstream
docs). Fabricating a thruster-mixing matrix here would silently produce
wrong control on a real/simulated vehicle, which is worse than not having
one.

Instead, axis -> command-vector-index is a plain, per-agent parameter
(`axis_map.<agent_name>`, a list of joystick axis indices, one per output
slot; -1 = always 0). This is a direct passthrough you fill in once you
know your BiguaSim scenario's real command semantics (confirm by echoing
`<agent>/command_control` while testing single axes at a time), the same
"parameters, not assumptions" approach biguasim_bridge/sonar_bridge_node.py
already uses for its polar-axis geometry.

Arm/disarm/reset/agent-selection button handling is ported as-is from
joy_holoocean.py -- none of that depends on vehicle-specific physics.
"""
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from biguasim_main.interface import BiguaSimInterface, COMMAND_MAP, MOTOR_SPEEDS


def _ax(axes: list, index: int) -> float:
    return axes[index] if 0 <= index < len(axes) else 0.0


def _command_length(agent_type: str, control_abstraction: str) -> int:
    if control_abstraction == 'cmd_motor_speeds':
        if agent_type not in MOTOR_SPEEDS:
            raise ValueError(
                f"Unknown agent_type '{agent_type}' for control_abstraction "
                f"'cmd_motor_speeds' -- add it to MOTOR_SPEEDS in "
                f"biguasim_main/interface.py."
            )
        return MOTOR_SPEEDS[agent_type]
    if control_abstraction not in COMMAND_MAP:
        raise ValueError(
            f"Unknown control_abstraction '{control_abstraction}' -- add it "
            f"to COMMAND_MAP in biguasim_main/interface.py."
        )
    return COMMAND_MAP[control_abstraction]


class AgentController:
    """Per-agent joystick -> command-vector state."""

    def __init__(self, name: str, agent_type: str, axis_map: list):
        self.name = name
        self.type = agent_type
        self.axis_map = axis_map  # list[int]; -1 = always 0.0

    def build_command(self, axes: list) -> list:
        return [_ax(axes, idx) if idx >= 0 else 0.0 for idx in self.axis_map]


class JoyToBiguaSimCommand(Node):
    def __init__(self):
        super().__init__('joy_biguasim')

        self.pub_by_agent = {}
        self.reset_client = self.create_client(Trigger, 'reset')

        self.declare_parameter('button.arm', 11)
        self.declare_parameter('button.disarm', 10)
        self.declare_parameter('button.reset', 12)
        self.declare_parameter('agents.buttons', [0])
        self.declare_parameter('agents.default', '')

        self.declare_parameter('params_file', '')
        self.declare_parameter('relative_path', True)

        def gi(p):
            return self.get_parameter(p).get_parameter_value().integer_value

        self.btn_arm = gi('button.arm')
        self.btn_disarm = gi('button.disarm')
        self.btn_reset = gi('button.reset')

        params_file = self.get_parameter('params_file').get_parameter_value().string_value
        relative_path = self.get_parameter('relative_path').get_parameter_value().bool_value
        if relative_path and params_file:
            params_file = os.path.join(get_package_share_directory('biguasim_main'), params_file)
        if not params_file or not os.path.isfile(params_file):
            raise ValueError(f"Scenario file not found with path: {params_file!r}")

        # init=False: parse the scenario YAML only, do not start BiguaSim.
        scenario = BiguaSimInterface(params_file, init=False).scenario
        scenario_agents = scenario.get('agents', [])
        names = [a['agent_name'] for a in scenario_agents]
        types = [a['agent_type'] for a in scenario_agents]
        abstractions = [a.get('control_abstraction', '') for a in scenario_agents]

        buttons = list(self.get_parameter('agents.buttons').get_parameter_value().integer_array_value)
        default = self.get_parameter('agents.default').get_parameter_value().string_value

        if len(buttons) < len(names):
            self.get_logger().warn(
                f'agents.buttons has {len(buttons)} entries but scenario has '
                f'{len(names)} agents. Extra agents will have no button assigned.'
            )

        self.controllers = {}
        self.button_to_agent = {}

        for i, (name, atype, abstraction) in enumerate(zip(names, types, abstractions)):
            try:
                cmd_len = _command_length(atype, abstraction)
            except ValueError as e:
                self.get_logger().error(f"agent [{name}]: {e} -- skipping.")
                continue

            self.declare_parameter(f'axis_map.{name}', [-1] * cmd_len)
            axis_map = list(self.get_parameter(f'axis_map.{name}').get_parameter_value().integer_array_value)
            if len(axis_map) != cmd_len:
                self.get_logger().warn(
                    f"agent [{name}]: axis_map.{name} has {len(axis_map)} entries, "
                    f"expected {cmd_len} for control_abstraction '{abstraction}'. "
                    "Padding/truncating with -1 (always 0.0)."
                )
                axis_map = (axis_map + [-1] * cmd_len)[:cmd_len]
            if all(idx < 0 for idx in axis_map):
                self.get_logger().warn(
                    f"agent [{name}]: axis_map.{name} is all -1 (every command slot "
                    "pinned to 0.0). Set it to real joystick axis indices to drive "
                    "this agent -- see README.md."
                )

            ctrl = AgentController(name, atype, axis_map)
            self.controllers[name] = ctrl
            self.pub_by_agent[name] = self.create_publisher(
                Float64MultiArray, f'{name}/command_control', 10)

            if i < len(buttons):
                self.button_to_agent[int(buttons[i])] = ctrl
                self.get_logger().info(
                    f'  agent [{name}]  type={atype}  control={abstraction}  '
                    f'cmd_len={cmd_len}  button={buttons[i]}')
            else:
                self.get_logger().info(
                    f'  agent [{name}]  type={atype}  control={abstraction}  '
                    f'cmd_len={cmd_len}  (no button)')

        if not self.controllers:
            raise RuntimeError('No usable agents found in scenario -- nothing to control.')

        if default not in self.controllers:
            default = names[0]
        self.active = self.controllers[default]

        self.enabled = True

        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self.get_logger().info(f'Joy node ready. Default agent: {self.active.name}')

    def _btn(self, msg: Joy, index: int) -> bool:
        return len(msg.buttons) > index and bool(msg.buttons[index])

    def _joy_cb(self, msg: Joy):
        if self._btn(msg, self.btn_arm):
            self.enabled = True
            self.get_logger().info('ENABLED')

        if self._btn(msg, self.btn_disarm):
            self.enabled = False
            # Publish an explicit zero command so the last nonzero command
            # doesn't stay latched in biguasim_node's command buffer (it has
            # no independent "stop" semantics -- it just keeps applying the
            # last value received every tick).
            zero = Float64MultiArray()
            zero.data = [0.0] * len(self.active.axis_map)
            self.pub_by_agent[self.active.name].publish(zero)
            self.get_logger().info('DISABLED')

        if self._btn(msg, self.btn_reset):
            self.reset_client.call_async(Trigger.Request())

        for btn, ctrl in self.button_to_agent.items():
            if self._btn(msg, btn):
                self.active = ctrl
                self.get_logger().info(f'Active agent -> {ctrl.name} ({ctrl.type})')

        if not self.enabled:
            return

        axes = list(msg.axes)
        cmd = Float64MultiArray()
        cmd.data = self.active.build_command(axes)
        self.pub_by_agent[self.active.name].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToBiguaSimCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
