#!/usr/bin/env bash
# Prepara o contêiner para o BiguaSim SITL. Idempotente: só instala o que falta,
# então pode rodar sempre (o run_sim.sh chama automaticamente).
#
# Uso: ./setup_container.sh [caminho/sim.yaml] [--rebuild]
#   --rebuild  força o colcon build do biguasim_sitl
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$HERE/../config/sim.yaml"
REBUILD=0
for a in "$@"; do
  case "$a" in
    --rebuild) REBUILD=1 ;;
    *) CONFIG="$a" ;;
  esac
done

SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"
step() { echo -e "\n==> $*"; }
ok()   { echo "    [ok] $*"; }

# lê um campo do YAML (chave.com.pontos) com valor padrão
cfg() {
  python3 - "$CONFIG" "$1" "$2" <<'EOF'
import os, sys, yaml
c = yaml.safe_load(open(sys.argv[1])) or {}
v = c
for k in sys.argv[2].split('.'):
    v = v.get(k) if isinstance(v, dict) else None
print(os.path.expanduser(str(v if v not in (None, '') else sys.argv[3])))
EOF
}

WS="$(cfg workspace ~/workspaces/biguasim_ws)"
AP="$(cfg ardupilot.path ~/ardupilot-sitl)"
VEH="$(cfg ardupilot.vehicle ArduSub)"
BIN="$AP/build/sitl/bin/$(echo "$VEH" | tr 'A-Z' 'a-z')"
WAF_TARGET="$(echo "$VEH" | sed 's/^Ardu//' | tr 'A-Z' 'a-z')"   # ArduSub -> sub

# 1) Python
step "Dependências Python (SITL/MAVProxy)"
if python3 -c "import pexpect, pymavlink, MAVProxy, future, em" 2>/dev/null; then
  ok "já instaladas"
else
  pip install --break-system-packages pexpect pymavlink MAVProxy future empy
fi

# 2) apt
step "Pacotes apt (MAVROS, rsync, xterm)"
MISSING=""
for p in rsync xterm ros-jazzy-mavros ros-jazzy-mavros-extras; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"
done
if [ -z "$MISSING" ]; then
  ok "já instalados"
else
  $SUDO apt update && $SUDO apt install -y $MISSING
fi

# 3) geoide (sem ele o MAVROS aborta na inicialização)
step "Dataset GeographicLib (geoide)"
if ls /usr/share/GeographicLib/geoids/egm96-5.* >/dev/null 2>&1; then
  ok "já presente"
else
  $SUDO /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
fi

# 4) ArduPilot próprio (o embutido no BiguaSim não compila)
step "ArduPilot SITL em $AP"
if [ ! -d "$AP/.git" ]; then
  git clone --recursive https://github.com/ArduPilot/ardupilot.git "$AP"
fi
if [ -x "$BIN" ]; then
  ok "binário $BIN já existe"
else
  (cd "$AP" && ./waf configure --board sitl && ./waf "$WAF_TARGET")
fi

# 5) pacote ROS 2
step "Pacote biguasim_sitl em $WS"
LAUNCH_INSTALLED="$WS/install/biguasim_sitl/share/biguasim_sitl/launch/full_sim.launch.py"
if [ "$REBUILD" -eq 1 ] || [ ! -e "$LAUNCH_INSTALLED" ]; then
  (
    set +u
    source /opt/ros/jazzy/setup.bash
    cd "$WS" && colcon build --packages-select biguasim_sitl --symlink-install
  )
else
  ok "já compilado (use --rebuild para forçar)"
fi

echo -e "\nContêiner pronto."
