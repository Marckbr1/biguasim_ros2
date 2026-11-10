"""
full_sim.launch.py — cadeia completa BiguaSim SITL + MAVROS + waypoints (+ QGC opcional).

Sequência:
  1. mata restos de execuções anteriores (sim_vehicle, MAVProxy, binário SITL)
  2. ArduPilot SITL (sim_vehicle.py) em janela xterm própria
  3. espera o ArduPilot abrir a TCP do SERIAL0 (garante a ordem)
  4. nó SITL do BiguaSim (sitl.launch.py, use_mavros:=false) -> abre o Unreal
  5. quando o log do nó mostra `sitl.ready_text` ("online"): MAVROS
  6. quando o FCU avisa que o EKF usa GPS (`waypoints.wait_text`): waypoints.launch.py
  Ao encerrar (Ctrl+C), limpa ArduPilot/MAVProxy para a próxima execução.

Toda a configuração vem de config/sim.yaml:
  ros2 launch biguasim_sitl full_sim.launch.py config:=/caminho/sim.yaml
"""
import os
import shlex
import subprocess
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'biguasim_sitl'

# Espera o binário SITL abrir a porta TCP do SERIAL0 (5760). Só lê /proc/net/tcp,
# sem conectar, para não roubar a conexão do MAVProxy. Aceita qualquer estado
# (LISTEN ou já conectado ao MAVProxy).
WAIT_PORT_PY = r'''
import sys, time
port, timeout = int(sys.argv[1]), float(sys.argv[2])
suffix = ':%04X' % port
t0 = time.time()
while time.time() - t0 < timeout:
    for f in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            with open(f) as fh:
                if any(l.split()[1].endswith(suffix) for l in fh.readlines()[1:]):
                    print('[full_sim] ArduPilot SITL no ar (TCP %d)' % port, flush=True)
                    sys.exit(0)
        except OSError:
            pass
    time.sleep(0.5)
print('[full_sim] AVISO: TCP %d nao abriu em %.0f s, seguindo mesmo assim' % (port, timeout), flush=True)
'''


# ------------------------------------------------------------------ util
def _x(p):
    return os.path.expanduser(os.path.expandvars(str(p)))


def _get(d, *keys, default=None):
    for k in keys:
        if not isinstance(d, dict) or d.get(k) is None:
            return default
        d = d[k]
    return d


def _str_args(d):
    out = {}
    for k, v in (d or {}).items():
        out[str(k)] = ('true' if v else 'false') if isinstance(v, bool) else str(v)
    return out


def _binary(cfg):
    ap = cfg.get('ardupilot', {})
    path = _x(ap.get('path', '~/ardupilot-sitl'))
    return os.path.join(path, 'build', 'sitl', 'bin', ap.get('vehicle', 'ArduSub').lower())


def _kill_leftovers(cfg):
    for pat in ('sim_vehicle.py', 'mavproxy.py', _binary(cfg)):
        subprocess.run(['pkill', '-f', pat], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _in_terminal(cfg, title, shell_cmd):
    if _get(cfg, 'terminal', 'program', default='xterm') == 'none':
        return ['bash', '-c', shell_cmd]
    cmd = ['xterm', '-T', title, '-geometry', '130x40']
    if _get(cfg, 'terminal', 'hold_on_exit', default=True):
        cmd.append('-hold')
    return cmd + ['-e', 'bash', '-c', shell_cmd]


# ------------------------------------------------------------- ArduPilot
def _ardupilot_shell(cfg):
    ap = cfg.get('ardupilot', {})
    path = _x(ap.get('path', '~/ardupilot-sitl'))
    args = ['sim_vehicle.py', '-v', ap.get('vehicle', 'ArduSub'),
            '-f', ap.get('model', 'JSON:127.0.0.1')]

    if ap.get('custom_location'):
        args.append(f"--custom-location={ap['custom_location']}")
    elif ap.get('location'):
        args += ['-L', ap['location']]

    if ap.get('console', True):
        args.append('--console')
    if ap.get('map', True):
        args.append('--map')
    if ap.get('skip_rebuild', True) and os.path.isfile(_binary(cfg)):
        args.append('-N')
    if ap.get('wipe_params', False):
        args.append('-w')

    args.append(f"--out=udp:127.0.0.1:{_get(cfg, 'mavros', 'port', default=14551)}")
    if _get(cfg, 'terminal', 'program', default='xterm') == 'none':
        args.append('--mavproxy-args=--non-interactive')

    cmd = ' '.join(shlex.quote(a) for a in args)
    if ap.get('extra_args'):
        cmd += ' ' + ap['extra_args']

    run_dir = _x(ap.get('run_dir') or '~')
    os.makedirs(run_dir, exist_ok=True)
    return (f'export PATH={shlex.quote(path)}/Tools/autotest:$HOME/.local/bin:$PATH; '
            f'cd {shlex.quote(run_dir)} && {cmd}')


# ---------------------------------------------------- MAVROS + waypoints
def _downstream(cfg, share):
    acts = []

    if _get(cfg, 'mavros', 'enabled', default=True):
        params = {'fcu_url': f"udp://:{_get(cfg, 'mavros', 'port', default=14551)}@"}
        params.update(_get(cfg, 'mavros', 'extra_params', default={}) or {})
        acts.append(LogInfo(msg=f"[full_sim] subindo MAVROS ({params['fcu_url']})"))
        acts.append(Node(package='mavros', executable='mavros_node',
                         output='screen', parameters=[params]))

    if _get(cfg, 'waypoints', 'enabled', default=True):
        wp_args = _str_args(_get(cfg, 'waypoints', 'launch_args'))
        wp_args['modo'] = str(_get(cfg, 'waypoints', 'modo', default='guided'))
        if _get(cfg, 'waypoints', 'params_file'):
            wp_args['params_file'] = _x(cfg['waypoints']['params_file'])
        wp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'waypoints.launch.py')),
            launch_arguments=wp_args.items())
        delay = float(_get(cfg, 'waypoints', 'delay', default=2.0))
        wait_text = _get(cfg, 'waypoints', 'wait_text', default='is using GPS')
        wait_timeout = float(_get(cfg, 'waypoints', 'wait_timeout', default=120))

        # Só sobe quando o EKF tem posição; antes disso o GUIDED é recusado
        # ("Guided requires position"). Disparo único: texto do FCU no log do
        # MAVROS ou timeout.
        wstate = {'started': False}

        def start_wp(reason):
            if wstate['started']:
                return None
            wstate['started'] = True
            return [LogInfo(msg=f'[full_sim] subindo waypoints ({reason})'),
                    TimerAction(period=delay, actions=[wp])]

        if not wait_text:
            acts.extend(start_wp('sem espera'))
        else:
            def on_io(event):
                if wait_text in event.text.decode(errors='ignore'):
                    return start_wp(f'"{wait_text}"')
                return None
            acts.append(LogInfo(msg=f'[full_sim] waypoints aguardando "{wait_text}" do FCU'))
            acts.append(RegisterEventHandler(OnProcessIO(on_stdout=on_io, on_stderr=on_io)))
            acts.append(TimerAction(period=wait_timeout, actions=[OpaqueFunction(
                function=lambda ctx: start_wp(f'timeout de {wait_timeout:.0f} s'))]))
    return acts


# ----------------------------------------------------------------- setup
def _setup(context):
    cfg_path = _x(LaunchConfiguration('config').perform(context))
    if not os.path.isfile(cfg_path):
        raise RuntimeError(f'Config não encontrada: {cfg_path}')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f) or {}

    share = get_package_share_directory(PKG)

    _kill_leftovers(cfg)
    time.sleep(0.5)

    actions = [LogInfo(msg=f'[full_sim] config: {cfg_path}')]

    # QGroundControl (opcional)
    qgc = _get(cfg, 'qgroundcontrol', 'path', default='')
    if qgc:
        actions.append(ExecuteProcess(cmd=[_x(qgc)], name='qgroundcontrol', output='log'))

    # 1) ArduPilot
    actions.append(ExecuteProcess(
        cmd=_in_terminal(cfg, 'ArduPilot SITL', _ardupilot_shell(cfg)),
        name='ardupilot', output='screen'))

    # 2) espera a porta JSON
    wait = ExecuteProcess(
        cmd=['python3', '-c', WAIT_PORT_PY,
             str(_get(cfg, 'ardupilot', 'serial0_port', default=5760)),
             str(_get(cfg, 'sitl', 'port_timeout', default=60))],
        name='espera_ardupilot', output='screen')
    actions.append(wait)

    # 3) nó SITL (use_mavros sempre false: o MAVROS é gerido aqui)
    sitl_args = _str_args(_get(cfg, 'sitl', 'launch_args'))
    sitl_args.update(_str_args({
        'enable_sonar': bool(_get(cfg, 'sitl', 'enable_sonar', default=False)),
        'enable_camera': bool(_get(cfg, 'sitl', 'enable_camera', default=False)),
    }))
    if _get(cfg, 'sitl', 'params_file'):
        sitl_args['params_file'] = _x(cfg['sitl']['params_file'])
    sitl_args['use_mavros'] = 'false'
    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sitl.launch.py')),
        launch_arguments=sitl_args.items())

    # 4) MAVROS + waypoints: disparo único, por texto no log ou por timeout
    state = {'started': False}
    ready_text = _get(cfg, 'sitl', 'ready_text', default='online')
    ready_timeout = float(_get(cfg, 'sitl', 'ready_timeout', default=90))

    def start_once(reason):
        if state['started']:
            return None
        state['started'] = True
        return [LogInfo(msg=f'[full_sim] nó SITL pronto ({reason})')] + _downstream(cfg, share)

    def on_io(event):
        if ready_text and ready_text in event.text.decode(errors='ignore'):
            return start_once(f'"{ready_text}"')
        return None

    actions.append(RegisterEventHandler(OnProcessIO(on_stdout=on_io, on_stderr=on_io)))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=wait,
        on_exit=[
            LogInfo(msg='[full_sim] subindo nó SITL do BiguaSim'),
            sitl,
            TimerAction(period=ready_timeout, actions=[OpaqueFunction(
                function=lambda ctx: start_once(f'timeout de {ready_timeout:.0f} s'))]),
        ])))

    # limpeza ao encerrar
    actions.append(RegisterEventHandler(OnShutdown(
        on_shutdown=lambda event, ctx: _kill_leftovers(cfg))))

    return actions


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory(PKG), 'config', 'sim.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg,
                              description='Caminho do sim.yaml'),
        OpaqueFunction(function=_setup),
    ])