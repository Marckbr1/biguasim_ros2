"""BiguaSim core ROS 2 node.

Owns the BiguaSim environment lifecycle: loads the scenario, ticks the sim on
a timer, publishes per-agent sensor topics, and accepts per-agent control
commands. Structural port of holoocean_main/holoocean_node.py, adapted to
BiguaSim's real control interface (a single std_msgs/Float64MultiArray per
agent on `<agent>/command_control`, sized/ordered per that agent's
control_abstraction -- see interface.py) instead of HoloOcean's typed
AgentCommand + Fossen autopilot topics.

Added on top of the original biguasim-ros2-develop version (which had
neither): a best-effort /clock publisher and a `reset` service, both mirrors
of holoocean_node.py. Both degrade to "log once and skip" rather than crash
if the underlying BiguaSim API doesn't support them -- this was not
verifiable without a BiguaSim install.
"""
from biguasim_main.interface import BiguaSimInterface

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock


class BiguaSimNode(Node):
    def __init__(self):
        super().__init__('biguasim_node')
        self.declare_parameter('params_file', '')

        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.subscribers = dict()
        self._clock_warned = False

        self.interface = BiguaSimInterface(file_path, node=self)
        self.sensor_publisher_create()
        self.control_subscribers_create()

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.reset_srv = self.create_service(Trigger, 'reset', self._reset_callback)

        # TODO: Make sure it doesn't tick too fast relative to real BiguaSim
        # step latency (i.e. that the sim can keep up with this period).
        period = self.interface.get_time_warp_period()
        self.get_logger().info(f'Time Warp Period: {period}')
        self.timer = self.create_timer(period, self.tick_callback)
        self.get_logger().info('Tick started')

    def control_subscribers_create(self):
        """
        Create one subscriber per agent (by name) to receive control commands
        as a flat Float64MultiArray on `<agent>/command_control`.
        """
        scenario = self.interface.scenario

        for agent_cfg in scenario['agents']:
            agent_cfg_name = agent_cfg['agent_name'].replace('-', '_')
            topic_base = f"{agent_cfg_name}/command_control"
            _ = self.create_subscription(
                Float64MultiArray,
                topic_base,
                lambda msg, agent_name=agent_cfg_name: self.control_callback(msg, agent_name),
                10
            )

    def sensor_publisher_create(self):
        """
        Create one publisher per agent sensor, named `<agent>/<sensor_name>`.
        """
        for sensor in self.interface.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, f"{sensor.agent_name}/{sensor.name}", 10)

    def adjust_timer(self, new_period):
        self.get_logger().info(f'Adjusting timer period to {new_period} seconds')
        self.timer.cancel()
        self.timer = self.create_timer(new_period, self.tick_callback)

    def control_callback(self, msg, agent_name):
        self.interface.send_control_command(agent_name, list(msg.data))

    def tick_callback(self):
        state = self.interface.tick()
        self.interface.publish_sensor_data(state)
        self._publish_clock(state)

    def _publish_clock(self, state):
        """Best-effort /clock publish from the sim step's own time, if present."""
        if 't' not in state:
            if not self._clock_warned:
                self.get_logger().warn(
                    "BiguaSim state has no 't' key -- /clock will not be published. "
                    "Nodes using use_sim_time will not advance."
                )
                self._clock_warned = True
            return

        t = float(state['t'])
        msg = Clock()
        msg.clock.sec = int(t)
        msg.clock.nanosec = int((t - int(t)) * 1e9)
        self.clock_pub.publish(msg)

    def _reset_callback(self, request, response):
        """Reset the BiguaSim environment, mirroring holoocean_node's `reset` service.

        Relies on env.reset() -- confirmed to exist on BiguaSim's env object
        by the example scenario script in BIGUASIM.md (`with biguasim.make(...)
        as env: env.reset()`), not independently re-verified here.
        """
        try:
            self.interface.env.reset()
            response.success = True
            response.message = 'Resetting the BiguaSim environment'
        except Exception as e:
            response.success = False
            response.message = f'Reset failed: {e}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BiguaSimNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
