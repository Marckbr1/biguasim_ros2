from pathlib import Path
import csv
import numpy as np
import logging


from .base_writer import BaseWriter

logger = logging.getLogger(__name__)
class TabularWriter(
    BaseWriter
):
    """
    Save tabular data to a CSV file.
    """
    def __init__(
            self, 
            output_dir, 
            sensor_cfg,
            writer_cfg,
        ):
        """
        Parameters
        ----------
        output_dir : str
            Directory where the CSV will be saved.

        sensor_cfg : SensorConfig
            DynamicsSensor sensor configuration.
        
        writer_cfg : WriterConfig
            DynamicsWriter writer configuration.
        """
        super().__init__(output_dir, 
                         sensor_cfg, 
                         writer_cfg
        )

        self.sensor_cfg = sensor_cfg
        self.writer_cfg = writer_cfg

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(
            parents=True, 
            exist_ok=True
        )

        if writer_cfg.field_names is not None:
            self.field_names = writer_cfg.field_names
        else:
            self.field_names = sensor_cfg.field_names

        filename = writer_cfg.filename

        if filename is None:
            filename = f"{sensor_cfg.name}.csv"

        csv_path = self.output_dir / filename
        logger.info(
            "Creating CSV '%s'.",
            csv_path,
        )
        
        # Opening the CSV File
        self.csv_file = open(
            csv_path,
            mode='w',
            newline="",
        )

        # Create a CSV writer object
        self.writer = csv.writer(
            self.csv_file
        )

        header = [
            "frame_id",
            "tick",
            "timestamp",
            *self.field_names,
        ]

        # Write the header row
        self.writer.writerow(header)
    

    def write(
        self, 
        frame_id, 
        tick, 
        timestamp, 
        data,
        ):
        """Write the dynamics data to a file."""
        if data is None:
            logger.error("Data is none")
            return  # Skip if the dynamics data is None
        
        data = np.asarray(data).reshape(-1)
        self.check_data(data)

        # self.writer.writerow(row)
        self.writer.writerow(
            [
                frame_id,
                tick,
                timestamp,
                *data.tolist(),
            ]
        )
        # Flushed (not just buffered) so a reader tailing this CSV during
        # collection (e.g. tools/dataset/visualize_run.py) sees each row as it's
        # written, instead of waiting for close() / the OS buffer to fill.
        self.csv_file.flush()

    def close(self):
        """Close the CSV file."""
        logger.info(
            "Closing '%s'.",
            self.csv_file.name,
        )
        self.csv_file.close()
    
    def check_data(self, data,):
        """
        Checks if data len == self.field_names
        """
        if len(data) != len(self.field_names):
            raise ValueError(
                "Sensor '%s' returned %d values, "
                "but %d field names are defined."
                % (
                    self.sensor_cfg.name,
                    len(data),
                    len(self.field_names),
                )
            )