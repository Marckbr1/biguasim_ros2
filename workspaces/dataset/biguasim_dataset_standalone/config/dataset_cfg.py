from pathlib import Path
from datetime import datetime
from typing import Tuple

from .writer_cfg import WriterConfig

class DatasetConfig:
    def __init__(self, cfg):
        self.raw = cfg

        # Root dir
        self.output_root = Path(
            cfg["output_root"]
        ).expanduser()

        self.dataset_name = cfg["dataset_name"]

        self.auto_create_run = cfg.get(
            "auto_create_run",
            True,
        )

        self.writers = {}
        self._build_writer()
    
    def create_run(self,) -> Tuple[Path, str]:

        dataset_root = (
            self.output_root
            / self.dataset_name
        )
        # Bug fix: 'run_name' was only assigned inside the auto_create_run
        # branch below but always returned -- auto_create_run: false raised
        # UnboundLocalError. Falls back to the dataset name itself, which is
        # at least a stable, inspectable identifier for that case.
        run_name = self.dataset_name
        if self.auto_create_run:

            run_name = datetime.now().strftime(
                "%Y%m%d_%H%M%S_%f"
            )

            dataset_root = (
                dataset_root
                / "runs"
                / run_name
            )

        dataset_root.mkdir(
            parents=True,
            exist_ok=True,
        )

        return dataset_root, run_name


    def _build_writer(self):
        for name, writer_cfg in self.raw["writers"].items():
            writer = WriterConfig(
                name=name,
                type=writer_cfg["type"],
                sensor=writer_cfg["sensor"],
                output=writer_cfg["output"],
                raw=writer_cfg
            )
            self.writers[name] = writer
    
    def get_writer(self, writer_name: str):
        return self.writers[writer_name]
    