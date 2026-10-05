#!/bin/bash
# Quick test of a route WITHOUT recording a dataset (--dry-run): the ROV
# follows the waypoints in BiguaSim and only its actual position is logged
# to results/dryrun_ground_truth.csv. Run inside the container.
#
# Usage:
#   ./test_route.sh                                # waypoints/example.yaml
#   ./test_route.sh waypoints/my_route.yaml --show-view
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

TRAJ="${1:-waypoints/example.yaml}"
[ $# -gt 0 ] && shift
SIM_CONFIG="${SIM_CONFIG:-config/simulator_rov.yaml}"

# Spawn on the first waypoint of the route (default). Disable with SPAWN_AT_START=0
# to use the spawn written in SIM_CONFIG as-is.
if [ "${SPAWN_AT_START:-1}" != "0" ]; then
    SIM_CONFIG="$(python3 tools/spawn_at_first_waypoint.py "$TRAJ" --base "$SIM_CONFIG" \
        --out "config/generated/simulator_$(basename "${TRAJ%.yaml}").yaml")"
fi

python3 tools/collect_dataset.py \
    --trajectory-config "$TRAJ" \
    --simulator-config "$SIM_CONFIG" \
    --max-duration "${MAX_DURATION:-600}" \
    --ground-truth-log results/dryrun_ground_truth.csv \
    --dry-run "$@"
