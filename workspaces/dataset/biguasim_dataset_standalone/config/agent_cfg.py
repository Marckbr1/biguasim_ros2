from dataclasses import dataclass, field
from .sensor_cfg import SensorConfig

@dataclass
class AgentConfig:
    name: str
    type: str
    sensors: dict[str, SensorConfig] = field(default_factory=dict)

    # Store the raw configuration for any additional parameters
    raw: dict = field(default_factory=dict)
