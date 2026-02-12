import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select

class DogController(Node):

    def __init__(self):
        super().__init__('dog_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.state = "MOVE"
        self.stop_time = None
        self.turn_time = None

    def control_loop(self):
        msg = Twist()

        # 键盘检测
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 's':
                self.state = "STOP"
                self.stop_time = self.get_clock().now()

        if self.state == "MOVE":
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info("Moving forward")

        elif self.state == "STOP":
            msg.linear.x = 0.0
            self.get_logger().info("Stopping")

            if self.stop_time is not None:
                now = self.get_clock().now()
                if (now - self.stop_time).nanoseconds / 1e9 > 2:
                    self.state = "TURN"
                    self.turn_time = now

        elif self.state == "TURN":
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.get_logger().info("Turning")

            if self.turn_time is not None:
                now = self.get_clock().now()
                if (now - self.turn_time).nanoseconds / 1e9 > 3:
                    self.state = "MOVE"

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DogController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
