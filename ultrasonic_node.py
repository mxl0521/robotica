import importlib
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.publisher_ = self.create_publisher(Float32, '/distance', 10)

        self.declare_parameter('rate_hz', 20.0)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0:
            rate_hz = 20.0
        self.period_sec = 1.0 / rate_hz

        server_dir = '/home/laton/Freenove_Robot_Dog_Kit_for_Raspberry_Pi-master/Code/Server'
        if server_dir not in sys.path:
            sys.path.append(server_dir)
        os.chdir(server_dir)

        ultrasonic_module = importlib.import_module('Ultrasonic')
        self.sonic = ultrasonic_module.Ultrasonic()

        self.timer = self.create_timer(self.period_sec, self.timer_callback)
        self.get_logger().info('Ultrasonic node started: publishing /distance')

    def timer_callback(self):
        msg = Float32()
        try:
            d = self.sonic.get_distance()
            msg.data = float(d)
        except Exception:
            msg.data = -1.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
