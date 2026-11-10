#!/usr/bin/env bash
# Sobe a simulação completa: prepara o contêiner (se faltar algo) e lança tudo.
#
# Uso (dentro do contêiner):
#   ./run_sim.sh                     # usa ../config/sim.yaml
#   ./run_sim.sh outro.yaml          # outra configuração
#   ./run_sim.sh --rebuild           # força recompilar o pacote antes
#
# Ctrl+C neste terminal encerra tudo.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$HERE/../config/sim.yaml"
SETUP_ARGS=()
for a in "$@"; do
  case "$a" in
    --rebuild) SETUP_ARGS+=("--rebuild") ;;
    *) CONFIG="$a" ;;
  esac
done
CONFIG="$(realpath "$CONFIG")"
[ -f "$CONFIG" ] || { echo "Config não encontrada: $CONFIG"; exit 1; }

[ -n "$DISPLAY" ] || echo "AVISO: DISPLAY vazio — xterm, mapa e Unreal não vão abrir."

"$HERE/setup_container.sh" "$CONFIG" "${SETUP_ARGS[@]}"

WS="$(python3 -c "import os,sys,yaml; c=yaml.safe_load(open(sys.argv[1])) or {}; \
print(os.path.expanduser(c.get('workspace') or '~/workspaces/biguasim_ws'))" "$CONFIG")"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo -e "\n==> Lançando simulação (config: $CONFIG)\n"
exec ros2 launch biguasim_sitl full_sim.launch.py config:="$CONFIG"
