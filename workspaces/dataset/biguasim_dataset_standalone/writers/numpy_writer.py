from pathlib import Path
import logging
import numpy as np

from .base_writer import BaseWriter

logger = logging.getLogger(__name__)

class NumpyWriter(BaseWriter):
    def __init__(
            self,
            output_dir: Path,
            sensor_cfg,
            writer_cfg,
    ):
        super().__init__(
                output_dir, 
                sensor_cfg,
                writer_cfg,
        )
        self.extension = writer_cfg.extension

        if not self.extension.startswith("."):
            self.extension = "." + self.extension

        self.output_dir.mkdir(
            parents=True, 
            exist_ok=True
        )

    def write(
            self,
            frame_id, 
            tick, 
            timestamp, 
            array,
    ):
        
        if array is None:
            logger.error("Data is none")
            return  # Skip if the array is None

        filename = (
            self.output_dir 
            / f"{frame_id:06d}_{timestamp:.6f}{self.extension}"
        )
        
        np.save(
            filename, 
            array,
        )