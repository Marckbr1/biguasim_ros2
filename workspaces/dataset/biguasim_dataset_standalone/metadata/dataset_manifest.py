from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..config.dataset_cfg import DatasetConfig

from dataclasses import dataclass, field, asdict
from pathlib import Path

import yaml


@dataclass
class DatasetManifest:
    """
    Dataset directory manifest.

    This file describes where each modality is stored inside
    the generated dataset, allowing loaders to locate data
    without scanning the directory tree.
    """

    paths: dict[str, str] = field(default_factory=dict)

    @classmethod
    def from_dataset_cfg(
        cls,
        dataset_cfg:DatasetConfig,
    ):
        """
        Build the dataset manifest from the dataset configuration.
        """

        paths = {
            "configs": "configs",
            "metadata": "metadata",
        }

        for writer in dataset_cfg.writers.values():
            paths[writer.name] = writer.output

        return cls(paths=paths)

    def to_dict(self):
        """Return the manifest as a dictionary."""
        return asdict(self)

    def save(
        self,
        output_dir: Path,
    ):
        """
        Save the dataset manifest.
        """

        with open(
            output_dir / "dataset_manifest.yaml",
            "w",
            encoding="utf-8",
        ) as f:

            yaml.safe_dump(
                self.to_dict(),
                f,
                sort_keys=False,
            )