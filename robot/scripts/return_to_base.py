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
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose
from slam_toolbox.srv import SerializePoseGraph
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
import tf2_ros


class ReturnToBase(Node):
    def __init__(self):
        super().__init__('return_to_base')

        # ── Parámetros configurables ──────────────────────────────────────
        self.declare_parameter('exploration_time', 150.0)
        self.declare_parameter('min_exploration_time', 60.0)  # no activar idle antes de esto
        self.declare_parameter('idle_timeout', 8.0)           # segundos sin goals → exploración terminada
        self.declare_parameter('map_dir', '/home/pablo')
        self.declare_parameter('map_base_name', 'explore')
        self.declare_parameter('overwrite_map', True)
        self.declare_parameter('battery_topic', '')
        self.declare_parameter('battery_threshold', 20.0)
        # ─────────────────────────────────────────────────────────────────

        self.exploration_time      = self.get_parameter('exploration_time').value
        self.min_exploration_time  = self.get_parameter('min_exploration_time').value
        self.idle_timeout          = self.get_parameter('idle_timeout').value
        self.map_dir               = self.get_parameter('map_dir').value
        self.map_base_name         = self.get_parameter('map_base_name').value
        self.overwrite_map         = self.get_parameter('overwrite_map').value
        self.battery_topic         = self.get_parameter('battery_topic').value
        self.battery_threshold     = self.get_parameter('battery_threshold').value

        self._returning          = False
        self._nav_was_active     = False  # Nav2 tuvo al menos un goal activo
        self._start_time         = self.get_clock().now()
        self._last_active_time   = self.get_clock().now()
        self._home_x             = None
        self._home_y             = None
        self._update_timer       = None

        self._nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._map_client  = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self._explore_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

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

    # ── 3. Batería baja ───────────────────────────────────────────────────
    def _on_battery(self, msg: BatteryState):
        pct = msg.percentage * 100.0
        if math.isnan(pct) or self._returning:
            return
        if pct < self.battery_threshold:
            self._explore_timer.cancel()
            self._trigger_return(f'Batería baja ({pct:.1f}% < {self.battery_threshold:.0f}%)')

    # ── Punto de entrada común para los tres triggers ─────────────────────
    def _trigger_return(self, reason: str):
        if self._returning:
            return
        self._returning = True
        self.get_logger().info(f'[return_to_base] {reason}. Retornando a base...')
        self._pause_explore_and_return()

    # ── 4. Pausar explore_lite antes de tomar Nav2 ────────────────────────
    def _pause_explore_and_return(self):
        msg = Bool()
        msg.data = False
        self._explore_pub.publish(msg)
        self.get_logger().info('[return_to_base] explore_lite pausado. Esperando 2s...')
        self._pause_timer = self.create_timer(2.0, self._send_goal_to_base)

    # ── 5. Mandar goal odom(0,0,0) a Nav2 ────────────────────────────────
    def _send_goal_to_base(self):
        self._pause_timer.cancel()

        self.get_logger().info('[return_to_base] Esperando action server navigate_to_pose...')
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[return_to_base] Nav2 no disponible, abortando.')
            return

        # Nav2 ignora frame_id y opera en map. lookup_transform da odom(0,0,0) en map.
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=3.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[return_to_base] TF odom→map fallido: {e}')
            return

        self._home_x = t.transform.translation.x
        self._home_y = t.transform.translation.y
        self._send_home_goal(self._home_x, self._home_y, initial=True)

        # Durante el retorno SLAM sigue ajustando map→odom; re-evaluar cada 6s
        self._update_timer = self.create_timer(6.0, self._update_home_goal)

    def _send_home_goal(self, x, y, initial=False):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        tag = 'Goal inicial' if initial else 'Goal actualizado'
        self.get_logger().info(f'[return_to_base] {tag}: odom(0,0,0) → map({x:.3f}, {y:.3f})')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

    def _update_home_goal(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except Exception:
            return

        new_x = t.transform.translation.x
        new_y = t.transform.translation.y
        drift = math.sqrt((new_x - self._home_x) ** 2 + (new_y - self._home_y) ** 2)

        if drift < 0.05:
            return

        self._home_x = new_x
        self._home_y = new_y
        self.get_logger().info(
            f'[return_to_base] odom se desplazó {drift:.3f}m, re-enviando goal'
        )
        self._send_home_goal(new_x, new_y)

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
            self.get_logger().info('[return_to_base] ¡Base alcanzada! Guardando mapa...')
            self._save_map()
        elif result.status == GoalStatus.STATUS_CANCELED:
            pass  # preemptado por _update_home_goal — el nuevo goal manejará el resultado
        else:
            self.get_logger().warn(
                f'[return_to_base] Navegación terminó con status {result.status}.'
            )

    # ── 6. Guardar mapa ───────────────────────────────────────────────────
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
