#!/usr/bin/env python3
"""
return_to_base.py
Dispara el retorno a odom(0,0,0) por cualquiera de tres causas:
  1. Timer: transcurrió exploration_time segundos.
  2. Idle:  Nav2 lleva idle_timeout segundos sin goals activos (explore_lite terminó)
            pero solo después de min_exploration_time segundos desde el arranque.
  3. Batería baja (opcional, activar con battery_topic).
Al llegar guarda el mapa via /slam_toolbox/serialize_map.
"""

import datetime
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, BackUp
from slam_toolbox.srv import SerializePoseGraph
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from explore_lite_msgs.msg import ExploreStatus


class ReturnToBase(Node):
    def __init__(self):
        super().__init__('return_to_base')

        # ── Parámetros configurables ──────────────────────────────────────
        self.declare_parameter('exploration_time', 150.0)
        self.declare_parameter('min_exploration_time', 30.0)  # no activar idle antes de esto
        self.declare_parameter('idle_timeout', 10.0)           # segundos sin goals → exploración terminada
        self.declare_parameter('map_dir', '/home/pablo')
        self.declare_parameter('map_base_name', 'explore')
        self.declare_parameter('overwrite_map', True)
        self.declare_parameter('battery_topic', '')
        self.declare_parameter('battery_threshold', 10.90)
        # Maniobra de docking final (Nav2 BackUp action)
        self.declare_parameter('dock_x_offset', 0.40)    # meta en X en lugar de 0 (m)
        self.declare_parameter('dock_yaw_offset', 0.0)   # corrección de yaw del goal (rad)
        self.declare_parameter('dock_reverse_dist', 0.45) # distancia de retroceso al dock (m)
        self.declare_parameter('dock_speed', 0.10)        # velocidad de la maniobra (m/s)
        self.declare_parameter('dock_startup_delay', 1.0) # s — espera tras goal Nav2 antes de BackUp
        # ─────────────────────────────────────────────────────────────────

        self.exploration_time      = self.get_parameter('exploration_time').value
        self.min_exploration_time  = self.get_parameter('min_exploration_time').value
        self.idle_timeout          = self.get_parameter('idle_timeout').value
        self.map_dir               = self.get_parameter('map_dir').value
        self.map_base_name         = self.get_parameter('map_base_name').value
        self.overwrite_map         = self.get_parameter('overwrite_map').value
        self.battery_topic         = self.get_parameter('battery_topic').value
        self.battery_threshold     = self.get_parameter('battery_threshold').value
        self.dock_x_offset         = self.get_parameter('dock_x_offset').value
        self.dock_yaw_offset       = self.get_parameter('dock_yaw_offset').value
        self.dock_reverse_dist     = self.get_parameter('dock_reverse_dist').value
        self.dock_speed            = self.get_parameter('dock_speed').value
        self.dock_startup_delay    = self.get_parameter('dock_startup_delay').value

        self._returning          = False
        self._nav_was_active     = False  # Nav2 tuvo al menos un goal activo
        self._start_time         = self.get_clock().now()
        self._last_active_time   = self.get_clock().now()
        self._update_timer       = None

        self._nav_client    = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._backup_client = ActionClient(self, BackUp, 'backup')
        self._map_client    = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self._explore_pub   = self.create_publisher(Bool, '/explore/resume', 10)

        self.get_logger().info(
            f'[return_to_base] Timer: {self.exploration_time:.0f}s | '
            f'Idle: {self.idle_timeout:.0f}s (activo tras {self.min_exploration_time:.0f}s) | '
            f'Overwrite: {self.overwrite_map}'
        )

        # Timer de exploración máxima
        self._explore_timer = self.create_timer(self.exploration_time, self._on_exploration_timeout)

        # Detección de inactividad de Nav2
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._on_nav_status,
            10,
        )
        self._idle_check_timer = self.create_timer(2.0, self._check_idle)

        # Fin de exploración (transient_local: QoS requerida por el topic)
        _status_qos = QoSProfile(depth=10)
        _status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(ExploreStatus, '/explore/status', self._on_explore_status, _status_qos)

        # Batería (opcional)
        if self.battery_topic:
            self.create_subscription(BatteryState, self.battery_topic, self._on_battery, 10)
            self.get_logger().info(
                f'[return_to_base] Monitoreando batería en {self.battery_topic} '
                f'(umbral: {self.battery_threshold:.0f}%)'
            )

    # ── 1. Timeout de exploración ─────────────────────────────────────────
    def _on_exploration_timeout(self):
        self._explore_timer.cancel()
        self._trigger_return('Tiempo de exploración cumplido')

    # ── 2. Detección de inactividad de Nav2 ───────────────────────────────
    def _on_nav_status(self, msg: GoalStatusArray):
        active = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING}
        if any(s.status in active for s in msg.status_list):
            self._last_active_time = self.get_clock().now()
            self._nav_was_active = True

    def _check_idle(self):
        if self._returning or not self._nav_was_active:
            return
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds / 1e9
        if elapsed < self.min_exploration_time:
            return
        idle_secs = (now - self._last_active_time).nanoseconds / 1e9
        if idle_secs >= self.idle_timeout:
            self._explore_timer.cancel()
            self._trigger_return(f'Nav2 inactivo {idle_secs:.1f}s — exploración terminada')

    # ── 3. Fin de exploración ─────────────────────────────────────────────
    def _on_explore_status(self, msg: ExploreStatus):
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE and not self._returning:
            self._explore_timer.cancel()
            self._trigger_return('Exploración completa')

    # ── 4. Batería baja ───────────────────────────────────────────────────
    def _on_battery(self, msg: BatteryState):
        pct = msg.percentage * 100.0
        if math.isnan(pct) or self._returning:
            return
        if pct < self.battery_threshold:
            self._explore_timer.cancel()
            self._trigger_return(f'Batería baja ({pct:.1f}% < {self.battery_threshold:.0f}%)')

    # ── Punto de entrada común para todos los triggers ────────────────────
    def _trigger_return(self, reason: str):
        if self._returning:
            return
        self._returning = True
        self.get_logger().info(f'[return_to_base] {reason}. Retornando a base...')
        self._pause_explore_and_return()

    # ── 7. Pausar explore_lite antes de tomar Nav2 ───────────────────────
    def _pause_explore_and_return(self):
        msg = Bool()
        msg.data = False
        self._explore_pub.publish(msg)
        self.get_logger().info('[return_to_base] explore_lite pausado. Esperando 2s...')
        self._pause_timer = self.create_timer(2.0, self._send_goal_to_base)

    # ── 6. Mandar goal map(0,0,0) a Nav2 ─────────────────────────────────
    def _send_goal_to_base(self):
        self._pause_timer.cancel()

        self.get_logger().info('[return_to_base] Esperando action server navigate_to_pose...')
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[return_to_base] Nav2 no disponible, abortando.')
            return

        self._send_home_goal(self.dock_x_offset, 0.0, initial=True)
        self._update_timer = self.create_timer(5.0, self._update_home_goal)

    def _send_home_goal(self, x, y, initial=False):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Yaw → quaternion (eje Z): qz = sin(yaw/2), qw = cos(yaw/2)
        yaw = self.dock_yaw_offset
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        tag = 'Goal inicial' if initial else 'Re-enviando goal'
        self.get_logger().info(
            f'[return_to_base] {tag}: map({x:.3f}, {y:.3f}, yaw={yaw:.3f} rad)'
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

    def _update_home_goal(self):
        self._send_home_goal(self.dock_x_offset, 0.0)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[return_to_base] Goal rechazado por Nav2.')
            return
        self.get_logger().info('[return_to_base] Goal aceptado. Navegando a base...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_reached)

    def _on_goal_reached(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self._update_timer:
                self._update_timer.cancel()
                self._update_timer = None
            self.get_logger().info('[return_to_base] ¡Punto de dock alcanzado! Iniciando maniobra de retroceso...')
            self._do_dock_reverse()
        elif result.status == GoalStatus.STATUS_CANCELED:
            pass  # preemptado por _update_home_goal — el nuevo goal manejará el resultado
        else:
            self.get_logger().warn(
                f'[return_to_base] Navegación terminó con status {result.status}.'
            )

    # ── 8. Maniobra de retroceso al dock (Nav2 BackUp action) ────────────
    def _do_dock_reverse(self):
        if self.dock_reverse_dist <= 0.0:
            self._save_map()
            return

        # self.get_logger().info(
        #     f'[return_to_base] Esperando {self.dock_startup_delay:.1f} s antes de '
        #     f'retroceder {self.dock_reverse_dist:.2f} m a {self.dock_speed:.2f} m/s vía BackUp'
        # )
        # Breve espera para que Nav2 termine de soltar el control del controller
        self._dock_delay_timer = self.create_timer(self.dock_startup_delay, self._send_backup_goal)

    def _send_backup_goal(self):
        self._dock_delay_timer.cancel()

        if not self._backup_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[return_to_base] action backup no disponible. Guardando mapa sin retroceder.')
            self._save_map()
            return

        goal = BackUp.Goal()
        goal.target.x = float(self.dock_reverse_dist)
        goal.speed = float(self.dock_speed)
        # Multiplicador 5× sobre el tiempo nominal: bajo carga de CPU el behavior_server
        # puede pausarse en medio del BackUp; no abortar la acción.
        allowance = max(5.0, (self.dock_reverse_dist / self.dock_speed) * 5.0)
        goal.time_allowance = Duration(sec=int(allowance))

        self._backup_client.send_goal_async(goal).add_done_callback(self._on_backup_response)

    def _on_backup_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[return_to_base] Goal de BackUp rechazado. Guardando mapa sin retroceder.')
            self._save_map()
            return
        # self.get_logger().info('[return_to_base] BackUp aceptado. Retrocediendo...')
        goal_handle.get_result_async().add_done_callback(self._on_backup_result)

    def _on_backup_result(self, _future):
        # self.get_logger().info('[return_to_base] Maniobra de dock completada. Guardando mapa...')
        self._save_map()

    # ── 9. Guardar mapa ───────────────────────────────────────────────────
    def _save_map(self):
        self.get_logger().info('[return_to_base] Esperando servicio /slam_toolbox/serialize_map...')
        if not self._map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('[return_to_base] slam_toolbox no disponible, mapa no guardado.')
            rclpy.shutdown()
            return

        if self.overwrite_map:
            filename = os.path.join(self.map_dir, f'{self.map_base_name}_temp')
        else:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.map_dir, f'{self.map_base_name}_{ts}')

        req = SerializePoseGraph.Request()
        req.filename = filename

        self.get_logger().info(f'[return_to_base] Guardando mapa en: {filename}')
        future = self._map_client.call_async(req)
        future.add_done_callback(self._on_map_saved)

    def _on_map_saved(self, future):
        try:
            future.result()
            self.get_logger().info('[return_to_base] Mapa guardado correctamente.')
        except Exception as e:
            self.get_logger().error(f'[return_to_base] Error al guardar mapa: {e}')
        finally:
            self.get_logger().info('[return_to_base] Nodo finalizado.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToBase()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
