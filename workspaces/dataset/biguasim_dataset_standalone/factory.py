from __future__ import annotations

import logging


from pathlib import Path

from .writers.image_writer import ImageWriter
from .writers.tabular_writer import TabularWriter
from .writers.numpy_writer import NumpyWriter

logger = logging.getLogger(__name__)

class WriterFactory:

    WRITERS = {
        "image": ImageWriter,
        "numpy": NumpyWriter,
        "tabular": TabularWriter,
    }

    @classmethod
    def create_writer(
        cls,
        dataset_cfg, 
        simulator_cfg,
        dataset_root
    ):
        """Create writer instances based on the provided configuration."""

        writers = []
        logger.info("Creating dataset writers...")
        for writer_cfg in dataset_cfg.writers.values():
            writer_class = cls.WRITERS.get(writer_cfg.type)

            if writer_class is None:
                logger.error(
                    "Unknown writer type '%s'.",
                    writer_cfg.type,
                )
                
                raise ValueError(
                    f"Unknown writer type: {writer_cfg.type}"
                )
            
            # Look up the matching sensor configuration
            sensor_cfg = simulator_cfg.get_sensor(
                simulator_cfg.main_agent,
                writer_cfg.sensor,
            )

            logger.debug(
                "Creating writer '%s' (%s -> %s).",
                writer_cfg.type,
                writer_cfg.sensor,
                writer_cfg.output,
            )


            writer = writer_class(
                output_dir= dataset_root / writer_cfg.output,
                sensor_cfg= sensor_cfg,
                writer_cfg= writer_cfg,
            )
            writers.append(writer)
        
        logger.info(
            "%d writer(s) successfully created.",
            len(writers),
        )

        return writers