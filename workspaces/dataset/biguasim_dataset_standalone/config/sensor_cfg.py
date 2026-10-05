from dataclasses import dataclass, field, asdict
from pathlib import Path
import yaml
import logging

from ..config.sensor_fields import TABULAR_FIELD_NAMES

logger = logging.getLogger(__name__)

@dataclass
class SensorConfig:
    name: str
    type: str

    socket: str | None = None
    hz: int | None = None

    configuration: dict = field(default_factory=dict)

    # Store the raw configuration for any additional parameters
    raw: dict = field(default_factory=dict)

    def to_dict(self,):
        return asdict(self)
     
    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
    ):
        """
        Create a SensorConfig from a sensor metadata YAML file.
        """
        with open(path,
                  "r",
                  encoding="utf-8",
        ) as f:
            data = yaml.safe_load(f)

        return cls(
            name=data["name"],
            type=data["type"],
            socket=data.get("socket"),
            hz=data.get("hz"),
            configuration=data.get("configuration", {}),
            raw=data.get("raw", {}),
        )
    
    # TODO: move this into a lambda in config/sensor_fields.py
    # so SensorConfig no longer needs to know about specific sensors.
    @property
    def field_names(self) -> list[str]:
        """Default column names for tabular sensors."""

        if self.type == "DynamicsSensor":
            fields = TABULAR_FIELD_NAMES.get(self.type)
            return fields[self.use_rpy]

        elif self.type == "RangeFinderSensor":
            laser_count = self.configuration.get(
                "LaserCount",
                1,
            )
            return [
                f"laser_{i}"
                for i in range(laser_count)
            ]
        elif self.type == "IMUSensor":
            fields = TABULAR_FIELD_NAMES.get(self.type)
            return fields[
                self.configuration.get(
                    "ReturnBias",
                    False,
                )
            ]
        elif self.type == "DVLSensor":
            fields = TABULAR_FIELD_NAMES.get(self.type)
            return fields[
                self.configuration.get(
                    "ReturnRange",
                    True,
                )
            ]
        else:
            return TABULAR_FIELD_NAMES.get(self.type)

    @property
    def is_tabular(self) -> bool:
        return self.type in TABULAR_FIELD_NAMES
    
    @property
    def width(self) -> int | None:
        return self.configuration.get("CaptureWidth", None)

    @property
    def height(self) -> int | None:
        return self.configuration.get("CaptureHeight", None)
    
    @property
    def use_rpy(self) -> bool:
        """Check if the sensor configuration specifies to use Roll, Pitch, Yaw (RPY)."""
        return self.configuration.get("UseRPY", False)
    
    @property
    def use_com(self) -> bool:
        """Check if the sensor configuration specifies to use Center of Mass (COM)."""
        return self.configuration.get("UseCOM", False)
    
    @property
    def range_bins(self) -> int | None:
        return self.configuration.get("RangeBins")


    @property
    def azimuth_bins(self) -> int | None:
        return self.configuration.get("AzimuthBins")
    
    @property
    def range_min(self):
        return self.configuration.get("RangeMin")

    @property
    def range_max(self):
        return self.configuration.get("RangeMax")


    @property
    def azimuth(self):
        return self.configuration.get("Azimuth")


    @property
    def elevation(self):
        return self.configuration.get("Elevation")


    @property
    def send_gt_intensity(self) -> bool:
        return self.configuration.get("SendGTIntensity", False)


    @property
    def send_gt_elevation(self) -> bool:
        return self.configuration.get("SendGTElevation", False)


    @property
    def send_gt_point_cloud(self) -> bool:
        return self.configuration.get("SendGTPointCloud", False)


    # def __repr__(self):
    #     return (
    #         f"SensorConfig("
    #         f"name='{self.name}'," 
    #         f"type='{self.type}'," 
    #         f"socket='{self.socket}',"
    #         f"hz='{self.hz}',"
    #         f"configuration='{self.configuration}',"
    #     )