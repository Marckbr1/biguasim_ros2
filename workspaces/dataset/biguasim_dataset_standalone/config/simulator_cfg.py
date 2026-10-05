from dataclasses import dataclass, field, asdict
from .agent_cfg import AgentConfig
from .sensor_cfg import SensorConfig

class SimulatorConfig:
    def __init__(self, cfg: dict):
        self.raw = cfg
        self.package_name = cfg["package_name"]
        self.world = cfg.get("world")
        self.main_agent = cfg.get("main_agent")
        self.ticks_per_sec = cfg.get("ticks_per_sec")
        self.agents = {}
        self.build_agents()

    def to_dict(self,):
        return asdict(self)
    def build_agents(self):
        for agent_cfg in self.raw["agents"]:
            agent = AgentConfig(
                name=agent_cfg["agent_name"],
                type=agent_cfg["agent_type"],
                raw=agent_cfg
            )
            for sensor_cfg in agent_cfg["sensors"]:
                sensor_name = sensor_cfg.get(
                    "sensor_name",
                    sensor_cfg["sensor_type"]
                )
                
                sensor = SensorConfig(
                    name=sensor_name,
                    type=sensor_cfg["sensor_type"],
                    socket=sensor_cfg.get("socket"),
                    hz=sensor_cfg.get("Hz"),

                    configuration=sensor_cfg.get("configuration", {}),
                    raw=sensor_cfg,
                )
                agent.sensors[sensor.name] = sensor

            self.agents[agent.name] = agent
    
    def get_agent(self,agent_name: str):
        return self.agents[agent_name]
    
    def get_sensor(self, agent_name: str, sensor_name: str):
        return self.agents[agent_name].sensors[sensor_name]
    