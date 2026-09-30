#!/usr/bin/env python3
"""
wait_for_hardware.py
Espera a que el stack de hardware (launch_robot.launch.py) esté arriba.

Pensado para bringup.launch.py: corre en paralelo al hardware y termina apenas
el grafo ROS muestra todo publicando, para recién entonces arrancar
slam_toolbox + Nav2. 

    ros2 run robot wait_for_hardware --timeout 60
"""

import argparse
import sys
import time

import rclpy

REQUIRED_NODES = ('/controller_manager', '/diff_cont', '/neato_laser')
REQUIRED_TOPICS = ('/scan', '/odom')

# s entre avisos de "todavía falta esto" mientras se espera
PROGRESS_PERIOD = 5.0


def _missing(node):
    """Lista de nodos/tópicos requeridos que todavía no aparecen en el grafo."""
    names = ['/' + n if ns == '/' else f'{ns}/{n}'
             for n, ns in node.get_node_names_and_namespaces()]
    return ([n for n in REQUIRED_NODES if n not in names]
            + [t for t in REQUIRED_TOPICS if node.count_publishers(t) == 0])


def main():
    parser = argparse.ArgumentParser(description='Espera al stack de hardware.')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='segundos antes de rendirse (default: 60)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = rclpy.create_node('wait_for_hardware')
    deadline = time.monotonic() + args.timeout
    last_report = 0.0
    missing = list(REQUIRED_NODES) + list(REQUIRED_TOPICS)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            # spin_once mantiene vivo el descubrimiento del grafo
            rclpy.spin_once(node, timeout_sec=0.25)
            missing = _missing(node)
            if not missing:
                print('[wait_for_hardware] hardware listo', flush=True)
                return 0
            now = time.monotonic()
            if now - last_report > PROGRESS_PERIOD:
                last_report = now
                print(f'[wait_for_hardware] esperando: {" ".join(missing)}',
                      flush=True)
    except KeyboardInterrupt:
        # Ctrl+C al launch entero: salir con 0 para no ensuciar la consola con
        # un "[ERROR] process has died" en el cierre normal. bringup.launch.py
        # no arranca nada más igual, porque chequea context.is_shutdown.
        print('[wait_for_hardware] interrumpido', flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print(f'[wait_for_hardware] timeout tras {args.timeout:.0f} s, '
          f'falta: {" ".join(missing)}', file=sys.stderr, flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
