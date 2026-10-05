#!/bin/bash
# Runs a BiguaSim data collection following a waypoint route and, once it
# finishes, validates the dataset and compares the desired vs actual route
# on the georeferenced map. Run INSIDE the container (where 'biguasim' is).
#
# Usage:
#   ./start_collection.sh                                   # example (waypoints/example.yaml)
#   ./start_collection.sh waypoints/my_route.yaml           # your route
#   ./start_collection.sh waypoints/my_route.yaml --show-view --max-duration 300
#
# Optional environment variables:
#   SIM_CONFIG=config/simulator_rov.yaml   DATASET_CONFIG=config/dataset.yaml
#   MAX_DURATION=600  (safety cap, in simulation seconds; the route stops on its own when done)
#   SPAWN_AT_START=1  (spawn on waypoint 1; 0 = use the location in SIM_CONFIG)
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

TRAJ="${1:-waypoints/example.yaml}"
[ $# -gt 0 ] && shift
SIM_CONFIG="${SIM_CONFIG:-config/simulator_rov.yaml}"
DATASET_CONFIG="${DATASET_CONFIG:-config/dataset.yaml}"
MAX_DURATION="${MAX_DURATION:-600}"

# Spawn on the first waypoint of the route (default). Disable with SPAWN_AT_START=0
# to use the spawn written in SIM_CONFIG as-is.
if [ "${SPAWN_AT_START:-1}" != "0" ]; then
    SIM_CONFIG="$(python3 tools/spawn_at_first_waypoint.py "$TRAJ" --base "$SIM_CONFIG" \
        --out "config/generated/simulator_$(basename "${TRAJ%.yaml}").yaml")"
fi

if ! python3 -c "import biguasim" 2>/dev/null; then
    echo "ERROR: module 'biguasim' not found for this python3." >&2
    echo "Run this inside the container (docker/enter_container.sh)." >&2
    exit 1
fi

# CSV referenced by the trajectory YAML (needed for the final comparison)
CSV="$(python3 - "$TRAJ" <<'EOF'
import sys, yaml
from pathlib import Path
p = Path(sys.argv[1])
print((p.resolve().parent / yaml.safe_load(p.read_text())["waypoints_csv_path"]).resolve())
EOF
)"

echo "== Trajectory: $TRAJ ($CSV)"
echo "== Simulator:  $SIM_CONFIG"
echo "== Dataset:    $DATASET_CONFIG"

# 1) Route on the map (before simulating)
python3 tools/plot_route_on_map.py --waypoints-csv "$CSV" \
    --simulator-config "$SIM_CONFIG" \
    --out "results/route_$(basename "${CSV%.csv}").png"

# 2) Collection. collect_dataset.py prints the run directory as the last
#    line of stdout (tee: everything is still shown in the terminal).
RUN_DIR="$(python3 tools/collect_dataset.py \
    --trajectory-config "$TRAJ" \
    --simulator-config "$SIM_CONFIG" \
    --dataset-config "$DATASET_CONFIG" \
    --max-duration "$MAX_DURATION" \
    "$@" | tee /dev/stderr | tail -n 1)"

if [ ! -d "$RUN_DIR" ]; then
    echo "No run directory was created (dry-run or error)." >&2
    exit 0
fi
RUN_ID="$(basename "$RUN_DIR")"
echo "== Run: $RUN_DIR"

# 3) Validation (actual Hz, gaps, NaN, timestamps)
python3 tools/validate_dataset.py "$RUN_DIR" || true

# 4) Desired vs actual on the georeferenced map
python3 tools/compare_trajectory.py \
    --waypoints-csv "$CSV" \
    --run-dir "$RUN_DIR" \
    --map-yaml map/mapa.yaml \
    --out "results/${RUN_ID}_comparison.png"
