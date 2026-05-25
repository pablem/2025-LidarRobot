#!/usr/bin/env python3
"""
undock.py
Avanza en línea recta una distancia fija usando la behavior Nav2 DriveOnHeading.
Requiere que Nav2 (behavior_server) ya esté corriendo — se asume que slam_nav
fue lanzado previamente.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from nav2_msgs.action import DriveOnHeading


class Undock(Node):
    def __init__(self):
        super().__init__('undock')

        self.declare_parameter('undock_dist', 0.40)   # metros
        self.declare_parameter('undock_speed', 0.10)  # m/s
        self.declare_parameter('startup_delay', 1.0)  # s — esperar a que Nav2/behavior_server estén listos

        self._dist  = float(self.get_parameter('undock_dist').value)
        self._speed = float(self.get_parameter('undock_speed').value)
        self._delay = float(self.get_parameter('startup_delay').value)

        self._client = ActionClient(self, DriveOnHeading, 'drive_on_heading')

        # self.get_logger().info(
        #     f'[undock] Esperando {self._delay:.1f} s antes de avanzar '
        #     f'{self._dist:.2f} m a {self._speed:.2f} m/s vía DriveOnHeading'
        # )

        # Timer one-shot: dispara _send_goal() tras startup_delay segundos
        self._start_timer = self.create_timer(self._delay, self._on_start_timer)

    def _on_start_timer(self):
        self._start_timer.cancel()
        self._send_goal()

    def _send_goal(self):
        # self.get_logger().info('[undock] Esperando action server drive_on_heading...')
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('[undock] drive_on_heading no disponible, abortando.')
            rclpy.shutdown()
            return

        goal = DriveOnHeading.Goal()
        goal.target.x = self._dist
        goal.speed = self._speed
        # Margen ×3 sobre el tiempo nominal de la maniobra
        allowance = max(5.0, (self._dist / self._speed) * 3.0)
        goal.time_allowance = Duration(sec=int(allowance))

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[undock] Goal rechazado por behavior_server.')
            rclpy.shutdown()
            return
        # self.get_logger().info('[undock] Goal aceptado. Avanzando...')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _future):
        # self.get_logger().info('[undock] Completado. Nodo finalizado.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Undock()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
