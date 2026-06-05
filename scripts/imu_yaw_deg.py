import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class GyroIntegrator(Node):
    def __init__(self):
        super().__init__('gyro_int')
        self.prev_time = None
        self.angle = 0.0
        self.create_subscription(Imu, '/imu/data_raw', self.cb, 10)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time is None:
            self.prev_time = t
            return

        dt = t - self.prev_time
        self.prev_time = t

        wz = msg.angular_velocity.z  # rad/s
        self.angle += wz * dt

        print(f"Ángulo Z: {math.degrees(self.angle):.2f}°")

rclpy.init()
node = GyroIntegrator()
rclpy.spin(node)
