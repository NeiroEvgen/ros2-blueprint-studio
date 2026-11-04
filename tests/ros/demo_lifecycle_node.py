"""Demo managed (lifecycle) node for the bridge integration tests. Fails activation while 'broken' is true."""
import rclpy
from rclpy.lifecycle import Node, TransitionCallbackReturn


class Camera(Node):
    def __init__(self):
        super().__init__("camera_demo")
        self.declare_parameter("broken", False)

    def on_configure(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        if self.get_parameter("broken").value:
            return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)


def main():
    rclpy.init()
    rclpy.spin(Camera())


if __name__ == "__main__":
    main()
