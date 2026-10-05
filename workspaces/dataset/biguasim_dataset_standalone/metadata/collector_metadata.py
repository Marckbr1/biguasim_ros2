from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..config.dataset_cfg import DatasetConfig
    from ..config.simulator_cfg import SimulatorConfig

import shutil
import yaml
from pathlib import Path
#from dataclasses import asdict

from .dataset_metadata import DatasetMetadata
from .dataset_manifest import DatasetManifest

class RunMetadata:
    """
    Responsible for saving the metadata of a collection run.

    Currently saves:

    - The configs used (simulator.yaml and dataset.yaml)
    - One YAML file per simulated sensor
    """
    def __init__(self, run_dir: Path, run_name:str):
        self.run_dir = run_dir
        self.run_name = run_name

        self.config_dir:Path = self.run_dir / "configs"
        self.sensor_dir:Path = self.run_dir / "sensors"

        self.config_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.sensor_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

    def save(
        self,
        simulator_cfg:SimulatorConfig,
        dataset_cfg:DatasetConfig,
        simulator_cfg_path:str,
        dataset_cfg_path:str,
        trajectory_cfg_path:str | None = None,
        trajectory_type:str = "",
        trajectory_params:dict | None = None,
    ) -> None:
        """Save all metadata from dataset (configs used, per-sensor YAML,
        dataset_metadata.yaml, dataset_manifest.yaml). Call this once at
        the START of a run (before the collection loop) to fail fast on
        bad config; call `update_run_summary()` again at the END once
        duration/frame-count are known (see runner.py)."""
        self.save_configs(
            simulator_cfg_path,
            dataset_cfg_path,
            trajectory_cfg_path,
        )

        self.save_sensors(
            simulator_cfg,
        )

        self._simulator_cfg = simulator_cfg
        self._dataset_cfg = dataset_cfg
        self._trajectory_type = trajectory_type
        self._trajectory_params = trajectory_params or {}

        self.save_dataset_metadata(
            simulator_cfg,
            dataset_cfg,
            trajectory_type=trajectory_type,
            trajectory_params=trajectory_params,
        )

        self.save_dataset_manifest(
            dataset_cfg,
        )

    def update_run_summary(
        self,
        duration_s: float,
        start_time_s: float,
        end_time_s: float,
        num_frames: int,
    ) -> None:
        """Re-write dataset_metadata.yaml with the final duration/frame
        count, known only once the collection loop has ended."""
        self.save_dataset_metadata(
            self._simulator_cfg,
            self._dataset_cfg,
            trajectory_type=self._trajectory_type,
            trajectory_params=self._trajectory_params,
            duration_s=duration_s,
            start_time_s=start_time_s,
            end_time_s=end_time_s,
            num_frames=num_frames,
        )

    def save_configs(
        self,
        simulator_cfg_path: Path,
        dataset_cfg_path: Path,
        trajectory_cfg_path: Path | None = None,
    ) -> None:
        """
        Copies the YAML files used during the collection.
        """
        shutil.copy2(
            simulator_cfg_path,
            self.config_dir / "simulator.yaml"
        )

        shutil.copy2(
            dataset_cfg_path,
            self.config_dir / "dataset.yaml",
        )

        if trajectory_cfg_path:
            shutil.copy2(
                trajectory_cfg_path,
                self.config_dir / "trajectory.yaml",
            )

    def save_sensors(
        self,
        simulator_cfg,
    ) -> None:
        """"""
        for agent in simulator_cfg.agents.values():

            agent_dir = self.sensor_dir / agent.name
            agent_dir.mkdir(
                exist_ok=True,
            )

            for sensor in agent.sensors.values():

                filename = (
                    agent_dir
                    / f"{sensor.name}.yaml"
                )

                with open(filename, "w") as f:

                    yaml.safe_dump(
                        sensor.to_dict(),
                        f,
                        sort_keys=False,
                    )
    
    def save_dataset_metadata(
    self,
    simulator_cfg,
    dataset_cfg,
    trajectory_type:str = "",
    trajectory_params:dict | None = None,
    duration_s:float = 0.0,
    start_time_s:float = 0.0,
    end_time_s:float = 0.0,
    num_frames:int = 0,
    ) -> None:
        """
        Save dataset metadata.
        """

        metadata = DatasetMetadata.from_configs(
            simulator_cfg,
            dataset_cfg,
            self.run_name,
            trajectory_type=trajectory_type,
            trajectory_params=trajectory_params,
            duration_s=duration_s,
            start_time_s=start_time_s,
            end_time_s=end_time_s,
            num_frames=num_frames,
        )

        metadata.save(self.run_dir)
    
    def save_dataset_manifest(
        self,
        dataset_cfg,
    ) -> None:
        """
        Save dataset directory manifest.
        """

        manifest = DatasetManifest.from_dataset_cfg(
            dataset_cfg,
        )

        manifest.save(
            self.run_dir,
        )