#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class NumberPublisher(Node):
    def __init__(self):
        super().__init__('python_number_publisher')
        self.publisher = self.create_publisher(
            Int32, '/lesson12/input_value', 10)
        self.value = 1
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg = Int32()
        msg.data = self.value
        self.publisher.publish(msg)
        self.get_logger().info(f'publish: {msg.data}')
        self.value += 1


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()