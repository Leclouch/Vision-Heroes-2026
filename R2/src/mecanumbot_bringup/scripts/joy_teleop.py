#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")

        # Parameters
        self.declare_parameter("axis_linear_x", 1)  # Left stick vertical
        self.declare_parameter("axis_linear_y", 0)  # Left stick horizontal
        self.declare_parameter("axis_angular", 3)  # Right stick horizontal
        self.declare_parameter("scale_linear", 0.5)
        self.declare_parameter("scale_angular", 1.0)
        self.declare_parameter(
            "enable_button", 5
        )  # RB button (deadman switch optional)
        self.declare_parameter(
            "require_enable_button", False
        )  # Set to True to require button press

        self.axis_linear_x = self.get_parameter("axis_linear_x").value
        self.axis_linear_y = self.get_parameter("axis_linear_y").value
        self.axis_angular = self.get_parameter("axis_angular").value
        self.scale_linear = self.get_parameter("scale_linear").value
        self.scale_angular = self.get_parameter("scale_angular").value
        self.enable_button = self.get_parameter("enable_button").value
        self.require_enable_button = self.get_parameter("require_enable_button").value

        # Publishers and Subscribers - using TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.get_logger().info("Mecanum Joy Teleop Node Started")
        self.get_logger().info(
            f"Mapping: LinX={self.axis_linear_x}, LinY={self.axis_linear_y}, AngZ={self.axis_angular}"
        )
        self.get_logger().info("Publishing to /cmd_vel (geometry_msgs/TwistStamped)")

    def joy_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"

        # Check enable button if required
        if self.require_enable_button:
            if (
                len(msg.buttons) <= self.enable_button
                or msg.buttons[self.enable_button] == 0
            ):
                # Stop if button not pressed
                self.publisher_.publish(twist_stamped)
                return

        # Read Joystick Axes
        # Ensure axis indices are valid
        if self.axis_linear_x < len(msg.axes):
            twist_stamped.twist.linear.x = (
                msg.axes[self.axis_linear_x] * self.scale_linear
            )

        if self.axis_linear_y < len(msg.axes):
            # Mecanum drive supports lateral movement
            twist_stamped.twist.linear.y = (
                msg.axes[self.axis_linear_y] * self.scale_linear
            )

        if self.axis_angular < len(msg.axes):
            twist_stamped.twist.angular.z = (
                msg.axes[self.axis_angular] * self.scale_angular
            )

        self.publisher_.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
