# collector/utils/files.py

# collector/utils/files.py

from pathlib import Path
import shutil


def copy_metadata_files(
    simulator_cfg_path,
    dataset_cfg_path,
    output_dir,
):
    """
    Copies the configuration files used in the collection.
    """

    metadata_dir = (
        Path(output_dir)
        / "configs"
    )

    metadata_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    shutil.copy2(
        simulator_cfg_path,
        metadata_dir / "simulator.yaml",
    )

    shutil.copy2(
        dataset_cfg_path,
        metadata_dir / "dataset.yaml",
    )