#!/usr/bin/env python3
"""
undock.py
Avanza en línea recta una distancia fija para salir del dock, en LAZO ABIERTO
publicando Twist directo en cmd_vel (vía twist_mux). NO usa la behavior Nav2
DriveOnHeading a propósito: esa es consciente de colisiones y, con el robot
modelado como círculo (robot_radius) encastrado contra la pared, su centro queda
dentro del radio del muro → el costmap lo marca en colisión y DriveOnHeading se
niega a avanzar. El lazo abierto ignora el costmap y siempre sale del dock.

Simétrico al retroceso de docking de return_to_base.py: ambos son comandos de
velocidad por tiempo, así que se ejecutan bajo carga similar y sus distancias
reales se mueven parejas con el nivel de batería (la discrepancia se cancela).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Undock(Node):
    def __init__(self):
        super().__init__('undock')

        self.declare_parameter('undock_dist', 0.40)        # metros
        self.declare_parameter('undock_speed', 0.10)       # m/s
        self.declare_parameter('startup_delay', 1.0)       # s — esperar a que twist_mux/controller estén listos
        self.declare_parameter('cmd_vel_topic', 'cmd_vel') # entrada de twist_mux (prioridad navigation)
        self.declare_parameter('publish_rate', 20.0)       # Hz de republicación del Twist (timeout twist_mux = 0.5s)

        self._dist  = float(self.get_parameter('undock_dist').value)
        self._speed = float(self.get_parameter('undock_speed').value)
        self._delay = float(self.get_parameter('startup_delay').value)
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._publish_rate  = float(self.get_parameter('publish_rate').value)

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        # Timer one-shot: dispara _start_undock() tras startup_delay segundos
        self._start_timer = self.create_timer(self._delay, self._on_start_timer)

    def _on_start_timer(self):
        self._start_timer.cancel()
        self._start_undock()

    def _start_undock(self):
        self._duration = self._dist / self._speed
        self._t0       = self.get_clock().now()
        self.get_logger().info(
            f'[undock] Avanzando {self._dist:.2f} m a {self._speed:.2f} m/s '
            f'(~{self._duration:.1f} s, open-loop, ignora obstáculos)...'
        )
        self._tick_timer = self.create_timer(1.0 / self._publish_rate, self._tick)

    def _tick(self):
        elapsed = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        if elapsed >= self._duration:
            self._tick_timer.cancel()
            self._cmd_vel_pub.publish(Twist())  # parada explícita
            self.get_logger().info('[undock] Completado. Nodo finalizado.')
            rclpy.shutdown()
            return
        cmd = Twist()
        cmd.linear.x = self._speed  
        self._cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Undock()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
