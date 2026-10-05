#!/bin/bash
# Creates a new route by clicking on the georeferenced map (needs a
# display). Writes waypoints/<name>.csv and waypoints/<name>.yaml, ready for
# ./start_collection.sh waypoints/<name>.yaml
#
# Controls: left click = add, right click / 'z' = undo, Enter = save.
# Zoom/pan with the toolbar; while zoom/pan is active clicks add nothing.
# If waypoints/<name>.csv already exists, it is loaded so you can edit it.
#
# Usage:
#   ./create_route.sh my_route            # z=-1.5, 1 m/s
#   Z=-3 SPEED=0.5 ./create_route.sh my_route
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

NAME="${1:?Usage: ./create_route.sh <name>}"

LOAD=()
if [ -f "waypoints/${NAME}.csv" ]; then
    echo "Editing existing route waypoints/${NAME}.csv"
    cp "waypoints/${NAME}.csv" "waypoints/${NAME}.csv.bak"
    LOAD=(--load "waypoints/${NAME}.csv.bak")
fi

python3 tools/trajectory_editor.py --map-yaml map/mapa.yaml \
    --z "${Z:--1.5}" --speed "${SPEED:-1.0}" --out "waypoints/${NAME}.csv" "${LOAD[@]}"

cat > "waypoints/${NAME}.yaml" <<EOF
type: waypoints
waypoints_csv_path: ${NAME}.csv
loop_trajectory: false
arrival_tolerance_m: 1.0
closed_loop_waypoints: true   # wait for the real arrival at each point
EOF

echo "Done: waypoints/${NAME}.yaml"
echo "Then: ./start_collection.sh waypoints/${NAME}.yaml"
