import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class listener(Node):
    def __init__(self):
        super().__init__('listener')
        
        # [AUTO-GEN-PUBS]
        
        
        # [AUTO-GEN-SUBS]
        

        self.sub = self.create_subscription(String, '/chatter', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')
        # [AUTO-GEN-FIRE]
        

def main():
    rclpy.init()
    node = listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
