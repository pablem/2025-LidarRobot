#!/usr/bin/env python3
"""
robot_menu.py
Menú GUI (Tkinter) para operar el robot sin escribir comandos.

Corre en la PC (Windows con RoboStack/pixi, o Linux). Los launch del stack y
el motor del LiDAR necesitan los puertos serie de la Raspberry, así que se
ejecutan allá por SSH; las herramientas (teleop, rviz, monitores) corren en
la PC y se comunican por DDS como siempre.

    python robot_menu.py
    python robot_menu.py --ros-args -p ssh_host:=robot_lidar@192.168.1.50

Requisitos: `ssh <ssh_host>` debe entrar SIN password (clave SSH instalada en
~/.ssh/authorized_keys de la Raspberry). Ver "Pasos en Windows" al final.

Grupos:

  Raspberry
    - Motor LiDAR: manda `printf 'MotorOn\\n' > <puerto>` / MotorOff por SSH.

  Stack (encadenados, por SSH): cada uno se habilita recién cuando el anterior
  está ACTIVO, verificado consultando el grafo ROS (nodos, publishers, estado
  lifecycle) y no por un timer fijo.
    1. launch_robot.launch.py  → hardware, ros2_control, LiDAR, EKF
    2. slam_nav.launch.py      → slam_toolbox + Nav2
    3. explore.launch.py       → undock + explore_lite + return_to_base
  Destildar manda Ctrl+C al launch remoto (byte ^C por el pty de ssh) y en
  cascada a los que dependen de él. Si un launch muere solo, el checkbox se
  destilda y queda en ERROR. Si ya estaba corriendo desde otra terminal se
  muestra "activo (externo)" y habilita el siguiente, pero no se detiene desde acá.

  Herramientas (locales, independientes): teleop en una consola nueva, rviz2,
  monitor de batería y monitor de odometría. Checked = proceso abierto.

La salida de cada proceso se guarda en <tmp>/robot_menu_<nombre>.log.

Pasos en Windows (una sola vez, en PowerShell):
    ssh-keygen -t ed25519                       # Enter a todo
    type $env:USERPROFILE\\.ssh\\id_ed25519.pub | ssh robot_lidar@pi "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
    ssh robot_lidar@pi hostname                 # debe responder sin pedir password
"""

import os
import signal
import subprocess
import sys
import tempfile
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State

IS_WINDOWS = sys.platform.startswith('win')
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RVIZ_CONFIG = os.path.join(SCRIPT_DIR, '..', 'config', 'nav.rviz')
LOG_DIR = tempfile.gettempdir()

COLOR_OFF = '#9e9e9e'
COLOR_STARTING = '#ef6c00'
COLOR_ACTIVE = '#2e7d32'
COLOR_ERROR = '#c62828'

# s en STARTING sin pasar las verificaciones antes de avisar (no se mata)
START_WARN_TIMEOUT = 90.0
# s de espera tras Ctrl+C antes de escalar (pkill remoto / kill local)
STOP_GRACE = 10.0
# Período de re-consulta del estado lifecycle de Nav2
LIFECYCLE_PROBE_PERIOD = 2.0
# s: una herramienta que termina con error antes de este tiempo se marca ERROR;
# después se asume que el usuario cerró la ventana
TOOL_ERROR_WINDOW = 5.0

# Defaults de conexión a la Raspberry (sobreescribibles por parámetro ROS)
DEFAULT_SSH_HOST = 'robot_lidar@pi'
DEFAULT_REMOTE_WS = '~/robotLidar'
DEFAULT_LIDAR_PORT = '/dev/serial/by-id/usb-Arduino_LLC_Arduino_Leonardo-if00'
SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
            '-o', 'StrictHostKeyChecking=accept-new']

# Estados de un proceso gestionado
OFF, STARTING, ACTIVE, EXTERNAL, STOPPING, ERROR = range(6)
STATE_TEXT = {
    OFF: 'detenido',
    STARTING: 'arrancando',
    ACTIVE: 'activo',
    EXTERNAL: 'activo (externo)',
    STOPPING: 'deteniendo',
    ERROR: 'ERROR',
}
STATE_COLOR = {
    OFF: COLOR_OFF,
    STARTING: COLOR_STARTING,
    ACTIVE: COLOR_ACTIVE,
    EXTERNAL: COLOR_ACTIVE,
    STOPPING: COLOR_STARTING,
    ERROR: COLOR_ERROR,
}


class MenuNode(Node):
    """Nodo rclpy: parámetros, consulta del grafo y estado lifecycle de Nav2."""

    def __init__(self, lifecycle_nodes):
        super().__init__('robot_menu')
        self.declare_parameter('ssh_host', DEFAULT_SSH_HOST)
        self.declare_parameter('remote_ws', DEFAULT_REMOTE_WS)
        self.declare_parameter('lidar_port', DEFAULT_LIDAR_PORT)
        self.ssh_host = self.get_parameter('ssh_host').value
        self.remote_ws = self.get_parameter('remote_ws').value
        self.lidar_port = self.get_parameter('lidar_port').value

        self._lc_clients = {n: self.create_client(GetState, f'{n}/get_state')
                            for n in lifecycle_nodes}
        self._lc_futures = {}
        self._lc_state = {n: None for n in lifecycle_nodes}
        self._lc_last_probe = 0.0

    def node_exists(self, name):
        return name in ['/' + n if ns == '/' else f'{ns}/{n}'
                        for n, ns in self.get_node_names_and_namespaces()]

    def topic_has_publisher(self, topic):
        return self.count_publishers(topic) > 0

    def probe_lifecycle(self):
        """Lanza (sin bloquear) un GetState a cada nodo lifecycle cada
        LIFECYCLE_PROBE_PERIOD s y recoge las respuestas que ya llegaron."""
        for name, fut in list(self._lc_futures.items()):
            if fut.done():
                try:
                    self._lc_state[name] = fut.result().current_state.id
                except Exception:
                    self._lc_state[name] = None
                del self._lc_futures[name]

        now = time.monotonic()
        if now - self._lc_last_probe < LIFECYCLE_PROBE_PERIOD:
            return
        self._lc_last_probe = now
        for name, client in self._lc_clients.items():
            if name in self._lc_futures:
                continue
            if not client.service_is_ready():
                self._lc_state[name] = None
                continue
            self._lc_futures[name] = client.call_async(GetState.Request())

    def lifecycle_active(self, name):
        return self._lc_state.get(name) == State.PRIMARY_STATE_ACTIVE


# ── Ejecución de procesos (local y remoto) ─────────────────────────────

def _popen_kwargs(new_console=False):
    """Flags de creación según plataforma."""
    kw = {}
    if IS_WINDOWS:
        if new_console:
            kw['creationflags'] = subprocess.CREATE_NEW_CONSOLE
    else:
        # Sesión propia para poder mandar SIGINT a todo el grupo (= Ctrl+C)
        kw['start_new_session'] = True
    return kw


def _kill_tree(proc, sig=None):
    """Mata el proceso local y sus hijos. En POSIX manda `sig` al grupo
    (SIGINT por defecto); en Windows no hay señales: taskkill del árbol."""
    if IS_WINDOWS:
        subprocess.run(['taskkill', '/PID', str(proc.pid), '/T', '/F'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        try:
            os.killpg(proc.pid, sig or signal.SIGINT)
        except ProcessLookupError:
            pass


def ssh_cmd(host, remote_command, tty=False):
    """Comando ssh para ejecutar `remote_command` en la Raspberry.
    tty=True fuerza pty: así el Ctrl+C escrito en stdin llega como ^C al
    proceso remoto y ros2 launch cierra limpio."""
    cmd = ['ssh'] + SSH_OPTS
    if tty:
        cmd += ['-tt']
    return cmd + [host, remote_command]


class ManagedProcess:
    """Proceso de larga duración lanzado desde el menú (local o ssh)."""

    def __init__(self, name, cmd, ready_check=None, remote_pattern=None,
                 host=None, new_console=False):
        self.name = name
        self.cmd = cmd
        self.ready_check = ready_check  # None → activo apenas arranca
        # Para procesos remotos: patrón pkill -f para escalar el apagado
        self.remote_pattern = remote_pattern
        self.host = host
        self.new_console = new_console
        self.proc = None
        self.state = OFF
        self.detail = ''
        self.t_start = 0.0
        self.t_stop = 0.0
        self._escalated = False
        self._log = None

    @property
    def owned(self):
        return self.proc is not None

    @property
    def is_remote(self):
        return self.remote_pattern is not None

    def start(self):
        log_path = os.path.join(LOG_DIR, f'robot_menu_{self.name}.log')
        self._log = open(log_path, 'w')
        try:
            self.proc = subprocess.Popen(
                self.cmd, stdout=self._log, stderr=subprocess.STDOUT,
                stdin=subprocess.PIPE if self.is_remote else subprocess.DEVNULL,
                **_popen_kwargs(self.new_console))
        except OSError as e:
            self._log.close()
            self._log = None
            self.state = ERROR
            self.detail = str(e)
            return
        self.state = STARTING
        self.detail = ''
        self.t_start = time.monotonic()
        self._escalated = False

    def stop(self):
        if not self.owned or self.state == STOPPING:
            return
        self.state = STOPPING
        self.t_stop = time.monotonic()
        if self.is_remote:
            # ^C por el pty remoto → SIGINT a ros2 launch en la Raspberry
            try:
                self.proc.stdin.write(b'\x03')
                self.proc.stdin.flush()
            except (OSError, ValueError):
                pass
        else:
            _kill_tree(self.proc)

    def _escalate(self):
        """Pasados STOP_GRACE s sin cerrar: pkill remoto + kill local."""
        if self._escalated:
            return
        self._escalated = True
        if self.is_remote:
            subprocess.Popen(ssh_cmd(self.host, f"pkill -TERM -f '{self.remote_pattern}'"),
                             stdout=self._log, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, **_popen_kwargs())
        _kill_tree(self.proc, signal.SIGTERM if not IS_WINDOWS else None)

    def _reap(self):
        self.proc.wait()
        self.proc = None
        if self._log:
            self._log.close()
            self._log = None

    def poll(self, node):
        """Actualiza state/detail. Devuelve True si el proceso terminó en
        este ciclo (para que la GUI destilde y propague)."""
        if self.owned:
            rc = self.proc.poll()
            if rc is not None:
                self._reap()
                if self.state == STOPPING:
                    self.state = OFF
                    self.detail = ''
                else:
                    self.state = ERROR
                    self.detail = f'terminó con código {rc} (ver log)'
                return True

            if self.state == STOPPING:
                waited = time.monotonic() - self.t_stop
                if waited > STOP_GRACE:
                    self._escalate()
                if waited > 2 * STOP_GRACE and not IS_WINDOWS:
                    _kill_tree(self.proc, signal.SIGKILL)
                self.detail = f'{waited:.0f} s'
                return False

            if self.ready_check is None or self.ready_check(node):
                self.state = ACTIVE
                self.detail = ''
            else:
                self.state = STARTING
                waited = time.monotonic() - self.t_start
                self.detail = f'{waited:.0f} s'
                if waited > START_WARN_TIMEOUT:
                    self.detail += ' — sin respuesta, revisar log'
            return False

        # No es nuestro: ¿está corriendo desde otra terminal?
        if self.ready_check is not None and self.ready_check(node):
            self.state = EXTERNAL
            self.detail = ''
        elif self.state == EXTERNAL:
            self.state = OFF
            self.detail = ''
        return False


class OneShot:
    """Comando corto (p. ej. MotorOn por ssh): se ejecuta y se espera el
    código de salida sin bloquear la GUI."""

    def __init__(self, name):
        self.name = name
        self.proc = None
        self._log = None
        self.running = False
        self.last_rc = None
        self.last_label = ''

    def run(self, cmd, label):
        if self.running:
            return
        log_path = os.path.join(LOG_DIR, f'robot_menu_{self.name}.log')
        self._log = open(log_path, 'w')
        self.last_label = label
        try:
            self.proc = subprocess.Popen(
                cmd, stdout=self._log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, **_popen_kwargs())
        except OSError as e:
            self._log.write(str(e))
            self._log.close()
            self.last_rc = -1
            return
        self.running = True

    def poll(self):
        """True cuando el comando acaba de terminar."""
        if not self.running:
            return False
        rc = self.proc.poll()
        if rc is None:
            return False
        self.proc.wait()
        self._log.close()
        self.running = False
        self.last_rc = rc
        return True


# ── Verificaciones de "listo" para cada launch ─────────────────────────

NAV2_LIFECYCLE_NODES = ['/bt_navigator', '/controller_server', '/planner_server']


def hardware_ready(node):
    return (node.node_exists('/controller_manager')
            and node.node_exists('/diff_cont')
            and node.node_exists('/neato_laser')
            and node.topic_has_publisher('/scan')
            and node.topic_has_publisher('/odom'))


def slam_nav_ready(node):
    return (node.topic_has_publisher('/map')
            and all(node.lifecycle_active(n) for n in NAV2_LIFECYCLE_NODES))


def explore_ready(node):
    return node.node_exists('/explore_node')


class MenuGui:
    def __init__(self, node):
        self.node = node
        self.closing = False
        host = node.ssh_host

        # Entorno ROS en la Raspberry antes de cada launch
        remote_setup = (f'source /opt/ros/humble/setup.bash && '
                        f'source {node.remote_ws}/install/setup.bash')

        def remote_launch(name, launch_file, ready):
            launch = f'ros2 launch robot {launch_file}'
            # '[r]os2' evita que pkill -f mate al propio shell que lo ejecuta
            pattern = '[r]' + launch[1:]
            return ManagedProcess(
                name, ssh_cmd(host, f'{remote_setup} && exec {launch}', tty=True),
                ready, remote_pattern=pattern, host=host)

        # Cadena principal: cada uno depende del anterior
        self.stack = [
            remote_launch('hardware', 'launch_robot.launch.py', hardware_ready),
            remote_launch('slam_nav', 'slam_nav.launch.py', slam_nav_ready),
            remote_launch('explore', 'explore.launch.py', explore_ready),
        ]
        stack_labels = [
            'Hardware  (launch_robot.launch.py)',
            'SLAM + Nav2  (slam_nav.launch.py)',
            'Exploración  (explore.launch.py)',
        ]

        # Herramientas locales
        py = sys.executable
        teleop = ['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                  '--ros-args', '-r', '/cmd_vel:=/cmd_vel_key']
        if not IS_WINDOWS:
            teleop = ['xterm', '-T', 'teleop_twist_keyboard', '-e'] + teleop
        self.tools = [
            ManagedProcess('teleop', teleop, new_console=True),
            ManagedProcess('rviz', ['rviz2', '-d', os.path.normpath(RVIZ_CONFIG)]),
            ManagedProcess('battery', [py, os.path.join(SCRIPT_DIR, 'battery_monitor.py')]),
            ManagedProcess('odom', [py, os.path.join(SCRIPT_DIR, 'odom_monitor.py')]),
        ]
        tool_labels = [
            'Teleop teclado  (consola → /cmd_vel_key)',
            'RViz2  (nav.rviz)',
            'Monitor de batería',
            'Monitor de velocidad y pose',
        ]

        # Comandos cortos por ssh
        self.motor = OneShot('motor')
        self.ssh_check = OneShot('ssh_check')

        self.root = tk.Tk()
        self.root.title('Robot LiDAR — Menú')
        self.root.minsize(440, 380)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self._build_raspberry_group(host)
        self.stack_rows = self._build_group('Stack (por SSH)', self.stack, stack_labels,
                                            self._on_stack_toggle)
        self.tool_rows = self._build_group('Herramientas (PC)', self.tools, tool_labels,
                                           self._on_tool_toggle)

        tk.Label(self.root, text=f'logs en {LOG_DIR}{os.sep}robot_menu_<nombre>.log',
                 fg=COLOR_OFF, font=('TkDefaultFont', 8)).pack(pady=(4, 6))

        # Prueba de conexión al arrancar: ssh sin password y puerto del LiDAR
        self.ssh_check.run(ssh_cmd(host, f'ls -l {node.lidar_port}'), 'ssh')

        self.root.after(100, self._tick)

    def _build_raspberry_group(self, host):
        frame = ttk.LabelFrame(self.root, text=f'Raspberry  ({host})')
        frame.pack(fill='x', padx=10, pady=(8, 2))
        frame.columnconfigure(1, weight=1)

        tk.Label(frame, text='Conexión SSH').grid(row=0, column=0, sticky='w', padx=8, pady=3)
        self.ssh_status = tk.Label(frame, text='verificando…', fg=COLOR_STARTING,
                                   anchor='e', font=('TkDefaultFont', 9, 'bold'))
        self.ssh_status.grid(row=0, column=1, sticky='e', padx=8)

        self.motor_var = tk.BooleanVar(value=False)
        self.motor_cb = ttk.Checkbutton(frame, text='Motor LiDAR  (MotorOn / MotorOff)',
                                        variable=self.motor_var, command=self._on_motor_toggle)
        self.motor_cb.grid(row=1, column=0, sticky='w', padx=8, pady=3)
        self.motor_status = tk.Label(frame, text='desconocido', fg=COLOR_OFF,
                                     anchor='e', font=('TkDefaultFont', 9, 'bold'))
        self.motor_status.grid(row=1, column=1, sticky='e', padx=8)

    def _build_group(self, title, procs, labels, callback):
        frame = ttk.LabelFrame(self.root, text=title)
        frame.pack(fill='x', padx=10, pady=(8, 2))
        frame.columnconfigure(1, weight=1)
        rows = []
        for i, (p, label) in enumerate(zip(procs, labels)):
            var = tk.BooleanVar(value=False)
            cb = ttk.Checkbutton(frame, text=label, variable=var,
                                 command=lambda i=i: callback(i))
            cb.grid(row=i, column=0, sticky='w', padx=8, pady=3)
            status = tk.Label(frame, text=STATE_TEXT[OFF], fg=COLOR_OFF,
                              anchor='e', font=('TkDefaultFont', 9, 'bold'))
            status.grid(row=i, column=1, sticky='e', padx=8)
            rows.append({'var': var, 'cb': cb, 'status': status})
        return rows

    # ── Callbacks de checkboxes ────────────────────────────────────────

    def _on_motor_toggle(self):
        word = 'MotorOn' if self.motor_var.get() else 'MotorOff'
        remote = f"printf '{word}\\n' > {self.node.lidar_port}"
        self.motor.run(ssh_cmd(self.node.ssh_host, remote), word)
        self.motor_status.config(text=f'enviando {word}…', fg=COLOR_STARTING)
        self.motor_cb.state(['disabled'])

    def _on_stack_toggle(self, i):
        p = self.stack[i]
        if self.stack_rows[i]['var'].get():
            if p.state == OFF or p.state == ERROR:
                p.start()
        else:
            # Apagar en cascada: primero los dependientes, del último al i
            for j in range(len(self.stack) - 1, i - 1, -1):
                self.stack[j].stop()
                self.stack_rows[j]['var'].set(False)

    def _on_tool_toggle(self, i):
        p = self.tools[i]
        if self.tool_rows[i]['var'].get():
            if not p.owned:
                p.start()
        else:
            p.stop()

    # ── Ciclo de refresco ─────────────────────────────────────────────

    def _tick(self):
        if self.closing:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.probe_lifecycle()
        self._refresh_raspberry()
        self._refresh_stack()
        self._refresh_tools()
        self.root.after(500, self._tick)

    def _refresh_raspberry(self):
        if self.ssh_check.poll():
            if self.ssh_check.last_rc == 0:
                self.ssh_status.config(text='ok', fg=COLOR_ACTIVE)
            else:
                self.ssh_status.config(
                    text='ERROR: sin acceso por clave o sin puerto LiDAR (ver log)',
                    fg=COLOR_ERROR)

        if self.motor.poll():
            self.motor_cb.state(['!disabled'])
            word = self.motor.last_label
            if self.motor.last_rc == 0:
                on = word == 'MotorOn'
                self.motor_status.config(text='encendido' if on else 'apagado',
                                         fg=COLOR_ACTIVE if on else COLOR_OFF)
            else:
                # Falló: el checkbox vuelve al estado anterior
                self.motor_var.set(word != 'MotorOn')
                self.motor_status.config(text=f'ERROR al enviar {word} (ver log)',
                                         fg=COLOR_ERROR)

    def _refresh_stack(self):
        prev_active = True  # el primero no depende de nada
        for i, (p, row) in enumerate(zip(self.stack, self.stack_rows)):
            exited = p.poll(self.node)
            if exited and p.state == ERROR:
                # Murió solo: destildar y apagar lo que dependía de él
                for j in range(len(self.stack) - 1, i, -1):
                    self.stack[j].stop()
                    self.stack_rows[j]['var'].set(False)
                row['var'].set(False)
            elif exited:
                row['var'].set(False)

            if p.state == EXTERNAL:
                row['var'].set(True)

            self._set_status(row, p)

            # Habilitado si el anterior está activo; un proceso externo o en
            # apagado no se puede (des)tildar
            enabled = prev_active and p.state not in (EXTERNAL, STOPPING)
            row['cb'].state(['!disabled'] if enabled else ['disabled'])
            prev_active = p.state in (ACTIVE, EXTERNAL)

    def _refresh_tools(self):
        for p, row in zip(self.tools, self.tool_rows):
            if p.poll(self.node):
                # Cerrado por el usuario (rviz, consola) o murió: destildar.
                # Una salida no-0 después de un rato es la ventana cerrada,
                # no un error real; solo cuenta como error si murió enseguida
                # (p. ej. paquete no instalado).
                if (p.state == ERROR
                        and time.monotonic() - p.t_start > TOOL_ERROR_WINDOW):
                    p.state = OFF
                    p.detail = ''
                row['var'].set(False)
            self._set_status(row, p)
            row['cb'].state(['disabled'] if p.state == STOPPING else ['!disabled'])

    def _set_status(self, row, p):
        text = STATE_TEXT[p.state]
        if p.detail:
            text += f'  ({p.detail})'
        row['status'].config(text=text, fg=STATE_COLOR[p.state])

    # ── Cierre ────────────────────────────────────────────────────────

    def _on_close(self):
        running = [p for p in self.stack + self.tools if p.owned]
        if running:
            ans = messagebox.askyesnocancel(
                'Cerrar menú',
                f'Hay {len(running)} proceso(s) lanzados desde el menú.\n\n'
                'Sí: detenerlos y cerrar.\n'
                'No: dejarlos corriendo y cerrar.\n')
            if ans is None:
                return
            if ans:
                for p in reversed(running):
                    p.stop()
                self._wait_stopped(running)
        self.closing = True
        self.root.destroy()

    def _wait_stopped(self, procs):
        deadline = time.monotonic() + 2 * STOP_GRACE + 2.0
        while time.monotonic() < deadline and any(p.owned for p in procs):
            for p in procs:
                p.poll(self.node)
            self.root.update()
            time.sleep(0.2)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = MenuNode(NAV2_LIFECYCLE_NODES)
    gui = MenuGui(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
