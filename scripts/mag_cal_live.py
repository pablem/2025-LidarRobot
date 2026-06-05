#!/usr/bin/env python3
"""Calibracion hard-iron del magnetometro EN VIVO (sin rosbag).

Se suscribe a /imu/mag, recolecta mientras girás el robot y al cortar (Ctrl+C)
reporta rango y centro por eje. Complementa a mag_calibration.py (que lee de bag).

Uso:
    python3 scripts/mag_cal_live.py
    # girá 2-3 vueltas LENTAS y COMPLETAS (360) con el robot plano
    # Ctrl+C para ver el reporte

Lectura esperada con el remap de ejes OK (giro en yaw, robot plano):
    - x e y: rango GRANDE y parecido (campo horizontal terrestre rotando)
    - z:     rango CHICO (componente vertical, no cambia al girar en yaw)
El centro(offset) de cada eje son los mag_*_offset para mpu9250driver/params/mpu9250.yaml.
Tras cargar los offsets y relanzar, los centros de x/y deben quedar ~0 (simetricos).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagCal(Node):
    def __init__(self):
        super().__init__('mag_cal_live')
        self.xs, self.ys, self.zs = [], [], []
        self.create_subscription(MagneticField, '/imu/mag', self.cb, 50)
        self.get_logger().info(
            'Recolectando /imu/mag... gira 2-3 vueltas completas. Ctrl+C para reporte.')

    def cb(self, msg):
        x, y, z = msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z
        if x == 0.0 and y == 0.0 and z == 0.0:   # data-not-ready -> descartar
            return
        self.xs.append(x); self.ys.append(y); self.zs.append(z)

    def report(self):
        if not self.xs:
            print('\nSin muestras validas.')
            return
        print(f'\n=== {len(self.xs)} muestras validas ===')
        for name, v in (('x', self.xs), ('y', self.ys), ('z', self.zs)):
            lo, hi = min(v), max(v)
            print(f'{name}:  min={lo:+.3e}  max={hi:+.3e}  '
                  f'rango(amplitud)={hi - lo:.3e}  centro(offset)={(hi + lo) / 2:+.3e}')
        print('\nLectura esperada con remap OK (giro en yaw, robot plano):')
        print('  - x e y: rango GRANDE y parecido (el campo horizontal rotando)')
        print('  - z: rango CHICO (componente vertical, no cambia al girar)')
        print('Los "centro(offset)" son los nuevos mag_*_offset para mpu9250.yaml.')


def main():
    rclpy.init()
    node = MagCal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
