"""Demo node for the parameter bridge integration test (runs inside a ROS 2 container)."""
import rclpy
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange, ParameterDescriptor,
                                SetParametersResult)
from rclpy.node import Node


class Demo(Node):
    def __init__(self):
        super().__init__("pid_demo")
        self.declare_parameter("kp", 1.0, ParameterDescriptor(
            description="proportional gain",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.0)]))
        self.declare_parameter("rate_hz", 50, ParameterDescriptor(
            integer_range=[IntegerRange(from_value=1, to_value=500, step=1)]))
        self.declare_parameter("enabled", True)
        self.declare_parameter("frame", "base_link", ParameterDescriptor(read_only=True))
        self.declare_parameter("weights", [1.0, 2.0])
        self.declare_parameter("mask", [1, 2, 3])
        self.add_on_set_parameters_callback(self._check)

    def _check(self, params):
        for p in params:
            if p.name == "kp" and p.value == 6.66:
                return SetParametersResult(successful=False, reason="6.66 is cursed")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    rclpy.spin(Demo())


if __name__ == "__main__":
    main()
