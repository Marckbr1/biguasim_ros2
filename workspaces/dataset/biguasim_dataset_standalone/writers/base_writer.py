from __future__ import annotations

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..config.sensor_cfg import SensorConfig
    from ..config.writer_cfg import WriterConfig

from abc import ABC, abstractmethod
from pathlib import Path

class BaseWriter(ABC):
    """Abstract base class for writers."""
    def __init__(
            self, 
            output_dir:str, 
            sensor_cfg:SensorConfig,
            writer_cfg:WriterConfig,
            ):
        self.output_dir:Path = Path(output_dir)
        self.sensor_cfg = sensor_cfg
        self.writer_cfg = writer_cfg

    @abstractmethod
    def write(self, frame_id, tick, timestamp, state):
        """Abstract method to write data. Must be implemented by subclasses."""
        pass