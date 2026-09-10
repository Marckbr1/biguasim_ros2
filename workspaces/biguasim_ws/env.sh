#!/usr/bin/env bash
# Self-locating environment for biguasim_ws.
#
#   source env.sh
#
# Exports BIGUASIM_WS -> this workspace's own root (computed from this
# script's location, not hardcoded). Launch files read it to find data/
# and bags/ by default; explicit command-line args always override.

export BIGUASIM_WS="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
echo "BIGUASIM_WS=${BIGUASIM_WS}"
