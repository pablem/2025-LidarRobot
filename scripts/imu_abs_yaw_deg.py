#!/usr/bin/env python3
"""Verifica el yaw ABSOLUTO de /imu/data (orientacion de Madgwick, con magnetometro).

Muestra lado a lado:
    - yaw_abs:  yaw del quaternion de orientacion (Madgwick + mag). NO debe derivar en reposo.
    - yaw_gyro: integracion del yaw-rate del giroscopo. Deriva lento en reposo (referencia).

Uso:
    python3 scripts/imu_abs_yaw_deg.py

Criterios de verificacion del magnetometro (paso 3):
    1. Robot QUIETO   -> yaw_abs estable (no deriva); yaw_gyro puede irse de a poco.
    2. Giro 360 real  -> yaw_abs recorre ~360 monotono, sin saltos ni retrocesos,
                         y vuelve al valor inicial al cerrar la vuelta.
    3. Signo (REP-103/ENU): girar a la IZQUIERDA (CCW) -> yaw_abs AUMENTA.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class AbsYaw(Node):
    def __init__(self):
        super().__init__('imu_abs_yaw')
        self.prev_t = None
        self.gyro_yaw = 0.0
        self.create_subscription(Imu, '/imu/data', self.cb, 10)

    def cb(self, msg):
        q = msg.orientation
        yaw_abs = quat_to_yaw(q.x, q.y, q.z, q.w)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_t is not None:
            self.gyro_yaw += msg.angular_velocity.z * (t - self.prev_t)
        self.prev_t = t

        print(f'yaw_abs(mag): {math.degrees(yaw_abs):+7.2f}    '
              f'yaw_gyro(int): {math.degrees(self.gyro_yaw):+8.2f}')


def main():
    rclpy.init()
    node = AbsYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
