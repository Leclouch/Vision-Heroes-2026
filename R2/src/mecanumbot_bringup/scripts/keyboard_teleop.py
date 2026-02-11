#!/usr/bin/env python3.10
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Cross-platform key reading
if sys.platform == "win32":
    import msvcrt

    def get_key():
        if msvcrt.kbhit():
            # getwch returns a string on Python 3
            return msvcrt.getwch()
        return None
else:
    import termios
    import tty
    import select

    def get_key():
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.scale = 0.5
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Print instructions
        self.get_logger().info("""
Mecanum Keyboard Teleop
---------------------------
Moving around:
   w
a  s  d

Rotation:
q   e

Stop: x
Increase Speed: +
Decrease Speed: -

CTRL-C to quit
""")

        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        try:
            key = get_key()
            if key:
                if key == "w":
                    self.vx = 1.0
                elif key == "s":
                    self.vx = -1.0
                elif key == "a":
                    self.vy = 1.0
                elif key == "d":
                    self.vy = -1.0
                elif key == "q":
                    self.wz = 1.0
                elif key == "e":
                    self.wz = -1.0
                elif key == "x":
                    self.vx = 0.0
                    self.vy = 0.0
                    self.wz = 0.0
                elif key == "+":
                    self.scale = min(self.scale + 0.1, 2.0)
                    self.get_logger().info(f"Speed: {self.scale:.1f}")
                elif key == "-":
                    self.scale = max(self.scale - 0.1, 0.1)
                    self.get_logger().info(f"Speed: {self.scale:.1f}")
                elif key == "\x03":  # Ctrl+C
                    raise KeyboardInterrupt
        except Exception as e:
            self.get_logger().error(f"Error reading key: {e}")

        # Publish TwistStamped
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = self.vx * self.scale
        t.twist.linear.y = self.vy * self.scale
        t.twist.angular.z = self.wz * self.scale
        self.pub.publish(t)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
