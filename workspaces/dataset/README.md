# colecta_simple

> Guía rápida en español con los comandos listos: [README_COMANDOS.md](README_COMANDOS.md)

Minimal copy of `colecta_biguasim/` for running **a simple BiguaSim data
collection that follows waypoints**, with the georeferenced map of the
`Pier-Harbor` world and a ready-to-run example. No ROS 2 needed.

```
waypoints/*.csv ──> BiguaSim (BlueROV2 ROV) ──> sensors + ground truth ──> dataset/
        │                                                                    │
        └──────────── map/mapa.png + mapa.yaml (georeferenced) <──────────────┘
                      (route before simulating / desired vs actual afterwards)
```

## Layout

```
colecta_simple/
├── start_collection.sh       # full collection: map -> simulation -> validation -> comparison
├── test_route.sh             # follows the route in BiguaSim WITHOUT recording a dataset (--dry-run)
├── create_route.sh           # creates a route by clicking on the map
├── config/
│   ├── simulator_rov.yaml    # world, vehicle (BlueROV2), sensors, spawn
│   └── dataset.yaml          # which sensors are written to disk, and where
├── waypoints/
│   ├── example.csv           # 6 waypoints (~35 m, ~45 s) near the spawn
│   └── example.yaml          # trajectory config pointing to the CSV
├── map/
│   ├── mapa.png              # top-down orthophoto of the world (3990x3990 px)
│   └── mapa.yaml             # georeference: 0.1 m/px, origin (-140.5, -444.5)
├── tools/
│   ├── collect_dataset.py    # collection entry point
│   ├── validate_dataset.py   # checks actual Hz, gaps, NaN, timestamps
│   ├── plot_route_on_map.py  # draws the route + spawn on the map (no BiguaSim needed)
│   ├── compare_trajectory.py # desired route vs ground truth on the map
│   ├── trajectory_editor.py  # click-on-map editor -> CSV
│   └── map_utils.py          # map pixel <-> world conversion
├── biguasim_dataset_standalone/  # collection package
├── dataset/                  # recorded runs: dataset/colecta_simple/runs/<run_id>/
└── results/                  # route and comparison PNGs
```

## Simple example (step by step)

Inside the container (BiguaSim is only installed there):

```bash
# on the host
cd ~/marckloc-dataset-ros2/docker
./start_container.sh && ./enter_container.sh

# inside the container
cd ~/workspaces/colecta_simple
./start_collection.sh                               # uses waypoints/example.yaml
./start_collection.sh waypoints/example.yaml --show-view   # with the Unreal viewport
```

`start_collection.sh` checks that `import biguasim` works first. If it
doesn't, install it inside the container with `pip install -e ~/biguasim`
(see `../../BIGUASIM.md`).

This runs, in order:

1. `results/route_example.png`: the route and the spawn drawn on the map.
2. The collection: the ROV spawns at `[59, -245, -1]`, goes to the first
   waypoint (automatic `TRANSIT` leg) and follows the 6 points.
3. The dataset is written to `dataset/colecta_simple/runs/<run_id>/`
   (`ground_truth/`, `imu/`, `dvl/`, `depth/`, `magnetometer/`, `rgb/`,
   `segmentation/`, `sonar/`, metadata).
4. Run validation (`tools/validate_dataset.py`).
5. `results/<run_id>_comparison.png`: desired vs actual route (ground
   truth) on the map, plus the minimum distance to each waypoint printed
   in the terminal.

To test only the motion before recording:

```bash
./test_route.sh                      # writes no dataset
```

## Your own route

### Option A: write the CSV by hand

`waypoints/my_route.csv`:

```csv
waypoint_id,x,y,z,yaw_deg,speed_mps,hold_time_s
1,61.0,-245.0,-1.5,-18.43,1.0,0.0
2,70.0,-248.0,-1.5,0.0,1.0,2.0
```

| Column | Unit | Meaning |
|---|---|---|
| `waypoint_id` | -- | Informative only (the real order is the row order) |
| `x`, `y` | m | **Absolute** world position (same frame as `mapa.yaml`) |
| `z` | m | Depth; negative = underwater |
| `yaw_deg` | degrees | Heading at that point (-180..180) |
| `speed_mps` | m/s | Speed of the leg that ends at this point |
| `hold_time_s` | s | Pause on arrival |

Copy `waypoints/example.yaml` to `waypoints/my_route.yaml` and set
`waypoints_csv_path: my_route.csv`.

### Option B: click on the map (needs a display)

```bash
./create_route.sh my_route           # writes waypoints/my_route.csv + .yaml
Z=-3 SPEED=0.5 ./create_route.sh my_route
```

### Then

```bash
python3 tools/plot_route_on_map.py --waypoints-csv waypoints/my_route.csv --out results/my_route.png
./start_collection.sh waypoints/my_route.yaml
```

If the route is far from the spawn, move `location:` in
`config/simulator_rov.yaml` close to the first waypoint, so the initial
`TRANSIT` leg is short.

## Georeferenced map

`map/mapa.yaml` (same schema as ROS `map_server`):

- `resolution: 0.1` m/px, `origin` = **lower-left** corner in the world.
- Covers x ∈ [-140.5, 258.5], y ∈ [-444.5, -45.5].
- Row 0 of the PNG = maximum y.

To convert:

```bash
python3 tools/map_utils.py --map-yaml map/mapa.yaml --world 59 -245    # world -> pixel
python3 tools/map_utils.py --map-yaml map/mapa.yaml --pixel 1995 1995  # pixel -> world
```

## Useful options

| What | How |
|---|---|
| Show the simulator | `./start_collection.sh <yaml> --show-view` |
| Time limit | `MAX_DURATION=300 ./start_collection.sh <yaml>` (default 120 s of simulation time) |
| Run name | `./start_collection.sh <yaml> --run-name test1` |
| Wait for actual arrival at each waypoint | `./start_collection.sh <yaml> --closed-loop-waypoints` |
| Record fewer sensors | Remove writers from `config/dataset.yaml` (e.g. `rgb`, `sonar_*`) |
| Other simulator/dataset config | `SIM_CONFIG=... DATASET_CONFIG=... ./start_collection.sh <yaml>` |
| All flags | `python3 tools/collect_dataset.py --help` |

Dataset paths (`output_root: dataset`) are relative to this folder. The
`.sh` scripts `cd` here first, so they can be launched from anywhere. If
you call `tools/collect_dataset.py` directly, run it from `colecta_simple/`.

## More

Comments in the code that mention `docs/...` refer to the full
documentation in `../colecta_biguasim/docs/`. For rosbag conversion, drone
collection, sonar FOV, segmentation, dashboards, etc., see the full project
in `../colecta_biguasim/`.
