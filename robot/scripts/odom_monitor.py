#!/usr/bin/env python3
"""
odom_monitor.py
Pequeña GUI (Tkinter) que muestra velocidad y pose del robot.
Se suscribe a:
  - /odom  (nav_msgs/msg/Odometry, salida del EKF): velocidad lineal/angular
           y pose en el frame odom.
Y consulta TF (map → base_link) para mostrar la pose en el mapa cuando
slam_toolbox está corriendo; si no hay transformada muestra "--".
"""

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

COLOR_OK = '#2e7d32'
COLOR_STALE = '#9e9e9e'

STALE_TIMEOUT = 2.0  # s sin /odom para marcar "sin conexión"


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stale_timeout', STALE_TIMEOUT)
        odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.stale_timeout = self.get_parameter('stale_timeout').value

        self.last_odom = None
        self.last_rx = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    def _on_odom(self, msg):
        self.last_odom = msg
        self.last_rx = self.get_clock().now()

    def is_stale(self):
        if self.last_rx is None:
            return True
        elapsed = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9
        return elapsed > self.stale_timeout

    def map_pose(self):
        """(x, y, yaw) en el frame del mapa, o None si no hay TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quat(t.transform.rotation)


class OdomGui:
    def __init__(self, node):
        self.node = node
        self.closing = False

        self.root = tk.Tk()
        self.root.title('Velocidad y Pose')
        self.root.minsize(300, 230)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self.vars = {}
        self._group('Velocidad', [('v', 'lineal'), ('w', 'angular')])
        self._group('Pose (odom)', [('ox', 'x'), ('oy', 'y'), ('oyaw', 'yaw')])
        self._group('Pose (map)', [('mx', 'x'), ('my', 'y'), ('myaw', 'yaw')])

        self.status_var = tk.StringVar(value='sin conexión')
        self.status_lbl = tk.Label(self.root, textvariable=self.status_var,
                                   fg=COLOR_STALE, font=('TkDefaultFont', 10, 'bold'))
        self.status_lbl.pack(pady=(4, 8))

        self.root.after(100, self._tick)

    def _group(self, title, fields):
        frame = ttk.LabelFrame(self.root, text=title)
        frame.pack(fill='x', padx=10, pady=(8, 0))
        for col, (key, label) in enumerate(fields):
            tk.Label(frame, text=label, fg=COLOR_STALE).grid(row=0, column=col, padx=10)
            var = tk.StringVar(value='--')
            tk.Label(frame, textvariable=var, font=('TkFixedFont', 13, 'bold'),
                     width=9).grid(row=1, column=col, padx=10, pady=(0, 4))
            self.vars[key] = var

    def _tick(self):
        if self.closing:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self._refresh()
        self.root.after(100, self._tick)

    def _refresh(self):
        node = self.node
        if node.is_stale() or node.last_odom is None:
            for v in self.vars.values():
                v.set('--')
            self.status_var.set('sin conexión')
            self.status_lbl.config(fg=COLOR_STALE)
            return

        od = node.last_odom
        tw = od.twist.twist
        self.vars['v'].set(f'{tw.linear.x:+.2f} m/s')
        self.vars['w'].set(f'{tw.angular.z:+.2f} rad/s')

        p = od.pose.pose
        self.vars['ox'].set(f'{p.position.x:+.2f} m')
        self.vars['oy'].set(f'{p.position.y:+.2f} m')
        self.vars['oyaw'].set(f'{math.degrees(yaw_from_quat(p.orientation)):+.0f}°')

        mp = node.map_pose()
        if mp is None:
            for k in ('mx', 'my', 'myaw'):
                self.vars[k].set('--')
        else:
            self.vars['mx'].set(f'{mp[0]:+.2f} m')
            self.vars['my'].set(f'{mp[1]:+.2f} m')
            self.vars['myaw'].set(f'{math.degrees(mp[2]):+.0f}°')

        moving = abs(tw.linear.x) > 0.01 or abs(tw.angular.z) > 0.02
        self.status_var.set('EN MOVIMIENTO' if moving else 'QUIETO')
        self.status_lbl.config(fg=COLOR_OK)

    def _on_close(self):
        self.closing = True
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = OdomMonitor()
    gui = OdomGui(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
