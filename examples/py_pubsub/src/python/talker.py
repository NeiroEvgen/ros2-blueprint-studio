import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class talker(Node):
    def __init__(self):
        super().__init__('talker')
        
        # [AUTO-GEN-PUBS]
        

        # the node's main publisher
        self.pub = self.create_publisher(String, '/chatter', 10)
        
        # [AUTO-GEN-SUBS]
        
        
        # publishes on its own timer, like the official tutorial's minimal_publisher
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Py: {self.i}"
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
        
        # [AUTO-GEN-FIRE]
        

def main():
    rclpy.init()
    node = talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
