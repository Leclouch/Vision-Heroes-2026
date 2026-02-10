#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class RbotMover(Node):
    def __init__(self):
        super().__init__("rbot_mover")
        # We publish to /cmd_vel which is consumed by the mecanumbot_drive_controller
        self.publisher_ = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Timer to run the movement sequence
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

        self.get_logger().info("Rbot Mover Node started. Moving the robot...")

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        elapsed_time = time.time() - self.start_time

        if elapsed_time < 3.0:
            # Move forward for 3 seconds
            msg.twist.linear.x = 0.5
            msg.twist.linear.y = 0.0
            msg.twist.angular.z = 0.0
            self.get_logger().info("Moving Forward...")
        elif elapsed_time < 5.0:
            # Move sideways (Strafe Left) for 2 seconds
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.5
            msg.twist.angular.z = 0.0
            self.get_logger().info("Strafing Left...")
        elif elapsed_time < 7.0:
            # Rotate for 2 seconds
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.angular.z = 1.0
            self.get_logger().info("Rotating...")
        else:
            # Stop
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.angular.z = 0.0
            self.get_logger().info("Sequence complete. Stopping.")
            # Optionally shutdown after sequence
            self.timer.cancel()
            # rclpy.clock.sleep_for(Duration(seconds=1)) # Give it time to send the stop command

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rbot_mover = RbotMover()

    try:
        rclpy.spin(rbot_mover)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before exiting
        stop_msg = TwistStamped()
        stop_msg.header.stamp = rbot_mover.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_link"
        rbot_mover.publisher_.publish(stop_msg)
        rbot_mover.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
