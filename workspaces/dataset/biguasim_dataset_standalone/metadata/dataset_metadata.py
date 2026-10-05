from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..config.simulator_cfg import SimulatorConfig
    from ..config.dataset_cfg import DatasetConfig

from dataclasses import dataclass, asdict, field
from datetime import datetime
from pathlib import Path
import yaml
from importlib.metadata import PackageNotFoundError, version


def get_package_version() -> str:
    """
    Return the installed package version.

    During development the package may not be installed,
    so return 'dev' instead.
    """
    try:
        return version("biguasim-dataset-standalone")
    except PackageNotFoundError:
        return "dev"


def get_biguasim_version() -> str:
    """Return the installed `biguasim` engine version, or 'unknown'.

    Recorded explicitly because the dataset's numerical content (sensor
    models, noise, dynamics) depends on this SDK version, not just on this
    collector's own version -- needed to interpret/reproduce a run later.
    """
    try:
        return version("biguasim")
    except PackageNotFoundError:
        return "unknown"


# Fixed by BiguaSim itself (biguasim_docs/usage/units-and-coordinates.rst,
# see docs/biguasim_dataset_audit.md section 8): right-handed, meters,
# +x forward / +y left / +z up, rotations in degrees (roll,pitch,yaw) for
# spawn/teleport but the DynamicsSensor/IMUSensor angular-velocity outputs
# are in rad/s -- both units coexist and must be converted explicitly.
# Ground truth (DynamicsSensor) is always in the GLOBAL frame at the
# vehicle's COM, regardless of the socket it's declared under.
COORDINATE_SYSTEM = (
    "world frame: right-handed, meters, +x forward / +y left / +z up "
    "(BiguaSim convention; NOT ENU/NED -- see docs/sensors.md). "
    "location/rotation and waypoint yaw_deg are in degrees; "
    "DynamicsSensor/IMUSensor angular outputs are in rad/s. "
    "Ground truth (DynamicsSensor) is always global-frame, at the "
    "vehicle's center of mass, independent of its declared socket."
)

TIMESTAMP_SOURCE = (
    "simulation time: state['t'] = env._num_ticks / ticks_per_sec, read "
    "once per tick and shared by every sensor written that tick. Never "
    "wall-clock (datetime.now()/time.time()) -- see docs/synchronization.md "
    "and the bug this fixes in docs/AUDITORIA_STANDALONE_DATASET.md section K."
)


@dataclass
class DatasetMetadata:
    """
    Stores general information about a generated dataset.
    """

    name: str
    version: str

    simulator: str
    world: str

    agent_name: str
    agent_type: str

    created_at: str

    ticks_per_sec: int

    sensors: list[str]

    # --- Reproducibility fields (see docs/dataset_format.md) ---
    biguasim_version: str = "unknown"
    control_abstraction: str = ""
    coordinate_system: str = COORDINATE_SYSTEM
    timestamp_source: str = TIMESTAMP_SOURCE
    sensor_hz: dict = field(default_factory=dict)
    trajectory_type: str = ""
    trajectory_params: dict = field(default_factory=dict)
    duration_s: float = 0.0
    start_time_s: float = 0.0
    end_time_s: float = 0.0
    num_frames: int = 0

    @classmethod
    def from_configs(
        cls,
        simulator_cfg:SimulatorConfig,
        dataset_cfg:DatasetConfig,
        run_name:str,
        trajectory_type:str = "",
        trajectory_params:dict | None = None,
        duration_s:float = 0.0,
        start_time_s:float = 0.0,
        end_time_s:float = 0.0,
        num_frames:int = 0,
    ):
        """
        Create dataset metadata from the simulator and dataset
        configuration objects, plus the trajectory/run summary (only known
        once the collection loop has finished, see runner.py).
        """

        agent = simulator_cfg.get_agent(
            simulator_cfg.main_agent,
        )

        sensor_hz = {
            sensor.name: (sensor.hz if sensor.hz is not None else simulator_cfg.ticks_per_sec)
            for sensor in agent.sensors.values()
        }

        return cls(
            name=dataset_cfg.dataset_name,
            version=get_package_version(),

            simulator=simulator_cfg.package_name,
            world=simulator_cfg.world,

            agent_name=agent.name,
            agent_type=agent.type,

            created_at=run_name,

            ticks_per_sec=simulator_cfg.ticks_per_sec,

            sensors=list(agent.sensors.keys()),

            biguasim_version=get_biguasim_version(),
            control_abstraction=agent.raw.get("control_abstraction", ""),
            sensor_hz=sensor_hz,
            trajectory_type=trajectory_type,
            trajectory_params=trajectory_params or {},
            duration_s=duration_s,
            start_time_s=start_time_s,
            end_time_s=end_time_s,
            num_frames=num_frames,
        )

    def save(
        self,
        output_dir: Path,
    ):
        """
        Save dataset metadata as a YAML file.
        """

        with open(
            output_dir / "dataset_metadata.yaml",
            "w",
            encoding="utf-8",
        ) as f:

            yaml.safe_dump(
                asdict(self),
                f,
                sort_keys=False,
            )
    
    def to_dict(self) -> dict:
        return asdict(self)