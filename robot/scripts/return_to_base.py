#!/usr/bin/env python3
"""
return_to_base.py
- Espera N segundos (exploration_time)
- Manda goal (0,0,0) a Nav2
- Al llegar: guarda el mapa via /slam_toolbox/serialize_map
  · Si overwrite_map=True  → guarda como <map_base_name>_temp (sobreescribe)
  · Si overwrite_map=False → guarda como <map_base_name>_<timestamp>
"""

import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from slam_toolbox.srv import SerializePoseGraph
from geometry_msgs.msg import PoseStamped


class ReturnToBase(Node):
    def __init__(self):
        super().__init__('return_to_base')

        # ── Parámetros configurables ──────────────────────────────────────
        self.declare_parameter('exploration_time', 150.0)
        self.declare_parameter('map_dir', '/home/pablo')
        self.declare_parameter('map_base_name', 'labo')
        self.declare_parameter('overwrite_map', False)
        # ─────────────────────────────────────────────────────────────────

        self.exploration_time = self.get_parameter('exploration_time').value
        self.map_dir          = self.get_parameter('map_dir').value
        self.map_base_name    = self.get_parameter('map_base_name').value
        self.overwrite_map    = self.get_parameter('overwrite_map').value

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._map_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.get_logger().info(
            f'[return_to_base] Exploración por {self.exploration_time:.0f}s. '
            f'Overwrite: {self.overwrite_map}'
        )

        # Timer de exploración — dispara una sola vez
        self._explore_timer = self.create_timer(
            self.exploration_time,
            self._on_exploration_timeout
        )

    # ── 1. Timeout de exploración ─────────────────────────────────────────
    def _on_exploration_timeout(self):
        self._explore_timer.cancel()
        self.get_logger().info(
            '[return_to_base] Tiempo de exploración cumplido. Retornando a base...'
        )
        self._send_goal_to_base()

    # ── 2. Mandar goal (0,0,0) a Nav2 ────────────────────────────────────
    def _send_goal_to_base(self):
        self.get_logger().info('[return_to_base] Esperando action server navigate_to_pose...')
        self._nav_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info('[return_to_base] Goal enviado: (0, 0, 0)')
        send_goal_future = self._nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_accepted)

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
        status = result.status
        if status == 4:
            self.get_logger().info('[return_to_base] ¡Base alcanzada! Guardando mapa...')
            self._save_map()
        else:
            self.get_logger().warn(
                f'[return_to_base] Navegación terminó con status {status}. '
                f'Reintentando en 5s con costmap limpio...'
            )
            self._clear_costmap_and_retry()

    def _clear_costmap_and_retry(self):
        clear_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        clear_client.wait_for_service(timeout_sec=5.0)
        clear_client.call_async(ClearEntireCostmap.Request())
        self.get_logger().info('[return_to_base] Costmap limpiado. Reintentando goal en 5s...')
        self.create_timer(5.0, self._send_goal_to_base)

    # ── 3. Guardar mapa ───────────────────────────────────────────────────
    def _save_map(self):
        self.get_logger().info('[return_to_base] Esperando servicio /slam_toolbox/serialize_map...')
        self._map_client.wait_for_service()

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