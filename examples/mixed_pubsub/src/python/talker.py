import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, '/chatter', 10)
        self.i = 0
        self.create_timer(1.0, self.tick)

    def tick(self):
        msg = String()
        msg.data = f'Py: {self.i}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.i += 1


def main():
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
