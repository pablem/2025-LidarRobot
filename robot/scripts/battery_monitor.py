#!/usr/bin/env python3
"""
battery_monitor.py
Pequeña GUI (Tkinter) que muestra el estado de la batería del robot.
Corre en la PC de desarrollo, no en la Raspberry (que es headless).
Se suscribe por la red DDS a:
  - /battery_state           (sensor_msgs/msg/BatteryState)
  - /battery_time_remaining  (std_msgs/msg/Float32, minutos restantes)
"""

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

# Umbrales de color (porcentaje 0..1)
PCT_OK = 0.50      # > 50 %  → verde
PCT_WARN = 0.20    # 20–50 % → naranja, < 20 % → rojo
COLOR_OK = '#2e7d32'
COLOR_WARN = '#ef6c00'
COLOR_LOW = '#c62828'
COLOR_STALE = '#9e9e9e'

# s sin mensajes para marcar "sin conexión".
# Debe cubrir un tramo de exploración largo sin pausas, porque ahora
# las lecturas solo se hacen con el robot quieto (ver diffbot_system.cpp).
STALE_TIMEOUT = 180.0

# power_supply_status (sensor_msgs/BatteryState)
STATUS_TEXT = {
    0: 'DESCONOCIDO',
    1: 'CARGANDO',
    2: 'DESCARGANDO',
    3: 'NO CARGA',
    4: 'LLENA',
}


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        self.declare_parameter('battery_topic', 'battery_state')
        self.declare_parameter('time_topic', 'battery_time_remaining')
        self.declare_parameter('stale_timeout', STALE_TIMEOUT)
        battery_topic = self.get_parameter('battery_topic').value
        time_topic = self.get_parameter('time_topic').value
        self.stale_timeout = self.get_parameter('stale_timeout').value

        self.last_state = None
        self.last_minutes = None
        self.last_rx = None  # rclpy.time.Time de la última recepción

        self.create_subscription(BatteryState, battery_topic, self._on_state, 10)
        self.create_subscription(Float32, time_topic, self._on_time, 10)

    def _on_state(self, msg):
        self.last_state = msg
        self.last_rx = self.get_clock().now()

    def _on_time(self, msg):
        self.last_minutes = msg.data

    def is_stale(self):
        if self.last_rx is None:
            return True
        elapsed = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9
        return elapsed > self.stale_timeout


class BatteryGui:
    def __init__(self, node):
        self.node = node

        self.root = tk.Tk()
        self.root.title('Estado de Batería')
        self.root.minsize(280, 170)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.closing = False

        pad = {'padx': 12, 'pady': 4}

        self.voltage_var = tk.StringVar(value='-- V')
        self.pct_var = tk.StringVar(value='-- %')
        self.minutes_var = tk.StringVar(value='-- min')
        self.status_var = tk.StringVar(value='sin conexión')

        tk.Label(self.root, textvariable=self.voltage_var,
                 font=('TkDefaultFont', 22, 'bold')).pack(**pad)

        bar_frame = tk.Frame(self.root)
        bar_frame.pack(fill='x', **pad)
        self.bar_w = 240
        self.bar_h = 26
        self.canvas = tk.Canvas(bar_frame, width=self.bar_w, height=self.bar_h,
                                highlightthickness=1, highlightbackground='#555')
        self.canvas.pack()
        self.bar_rect = self.canvas.create_rectangle(0, 0, 0, self.bar_h,
                                                     fill=COLOR_STALE, width=0)
        self.bar_text = self.canvas.create_text(self.bar_w / 2, self.bar_h / 2,
                                                text='', font=('TkDefaultFont', 11, 'bold'))

        tk.Label(self.root, textvariable=self.minutes_var,
                 font=('TkDefaultFont', 13)).pack(**pad)
        tk.Label(self.root, textvariable=self.status_var,
                 font=('TkDefaultFont', 11)).pack(**pad)

        self.root.after(100, self._tick)

    def _color_for(self, pct):
        if pct > PCT_OK:
            return COLOR_OK
        if pct >= PCT_WARN:
            return COLOR_WARN
        return COLOR_LOW

    def _tick(self):
        if self.closing:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self._refresh()
        self.root.after(100, self._tick)

    def _refresh(self):
        node = self.node
        if node.is_stale() or node.last_state is None:
            self.voltage_var.set('-- V')
            self.pct_var.set('-- %')
            self.minutes_var.set('-- min')
            self.status_var.set('sin conexión')
            self.canvas.itemconfig(self.bar_rect, fill=COLOR_STALE)
            self.canvas.coords(self.bar_rect, 0, 0, 0, self.bar_h)
            self.canvas.itemconfig(self.bar_text, text='')
            return

        st = node.last_state
        pct = max(0.0, min(1.0, float(st.percentage)))

        self.voltage_var.set(f'{st.voltage:.1f} V')
        pct_txt = f'{pct * 100:.0f} %'
        self.pct_var.set(pct_txt)

        self.canvas.itemconfig(self.bar_rect, fill=self._color_for(pct))
        self.canvas.coords(self.bar_rect, 0, 0, self.bar_w * pct, self.bar_h)
        self.canvas.itemconfig(self.bar_text, text=pct_txt)

        if node.last_minutes is not None:
            self.minutes_var.set(f'~{node.last_minutes:.0f} min restantes')
        else:
            self.minutes_var.set('-- min')

        self.status_var.set(STATUS_TEXT.get(st.power_supply_status, 'DESCONOCIDO'))

    def _on_close(self):
        self.closing = True
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = BatteryMonitor()
    gui = BatteryGui(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
