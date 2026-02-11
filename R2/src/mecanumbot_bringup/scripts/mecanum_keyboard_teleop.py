#!/usr/bin/env python3

import sys
import threading
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class KeyboardTeleop(Node):
    """Keyboard teleop untuk robot mecanum dengan kontrol omni-directional"""

    def __init__(self):
        super().__init__("keyboard_teleop")

        # Parameters
        self.declare_parameter("speed_linear", 0.5)
        self.declare_parameter("speed_angular", 1.0)
        self.declare_parameter("speed_strafe", 0.5)

        self.speed_linear = self.get_parameter("speed_linear").value
        self.speed_angular = self.get_parameter("speed_angular").value
        self.speed_strafe = self.get_parameter("speed_strafe").value

        # Publisher
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Timer untuk publish terus menerus (10Hz)
        self.twist = TwistStamped()
        self.timer = self.create_timer(0.1, self.publish_twist)

        self.get_logger().info("Mecanum Keyboard Teleop dimulai")
        self.print_help()

    def print_help(self):
        help_msg = """
===================================================
    MECANUM ROBOT KEYBOARD CONTROL
===================================================
Pergerakan:
    w         : Maju
    s         : Mundur
    a         : Geser kiri (strafe left)
    d         : Geser kanan (strafe right)
    q         : Putar kiri
    e         : Putar kanan
    
Diagonal (kombinasi 2 arah):
    w+a       : Maju + kiri
    w+d       : Maju + kanan
    s+a       : Mundur + kiri
    s+d       : Mundur + kanan
    
Stop:
    spasi     : Berhenti
    
Kontrol Kecepatan:
    up/down   : Naik/turun kecepatan linear
    left/right: Naik/turun kecepatan angular
    
Keluar:
    Ctrl+C    : Keluar

Kecepatan saat ini:
    Linear: %.2f m/s
    Strafe: %.2f m/s
    Angular: %.2f rad/s
===================================================
        """ % (self.speed_linear, self.speed_strafe, self.speed_angular)
        print(help_msg)

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.twist.twist.linear.x
        msg.twist.linear.y = self.twist.twist.linear.y
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.twist.twist.angular.z
        self.pub.publish(msg)

    def run(self):
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok():
                # Check if there's input
                if self.is_data():
                    key = sys.stdin.read(1)

                    # Reset semua velocity
                    self.twist.twist.linear.x = 0.0
                    self.twist.twist.linear.y = 0.0
                    self.twist.twist.angular.z = 0.0

                    # Handle keys
                    if key == "\x03":  # Ctrl+C
                        break
                    elif key == " ":
                        # Stop
                        pass
                    elif key == "w":
                        self.twist.twist.linear.x = self.speed_linear
                    elif key == "s":
                        self.twist.twist.linear.x = -self.speed_linear
                    elif key == "a":
                        self.twist.twist.linear.y = self.speed_strafe
                    elif key == "d":
                        self.twist.twist.linear.y = -self.speed_strafe
                    elif key == "q":
                        self.twist.twist.angular.z = self.speed_angular
                    elif key == "e":
                        self.twist.twist.angular.z = -self.speed_angular
                    elif key == "\x1b":  # Arrow keys
                        next1 = sys.stdin.read(1)
                        next2 = sys.stdin.read(1)
                        if next1 == "[":
                            if next2 == "A":  # Up arrow
                                self.speed_linear = min(self.speed_linear + 0.1, 2.0)
                                self.speed_strafe = min(self.speed_strafe + 0.1, 2.0)
                                print(
                                    f"\n[Kecepatan] Linear/Strafe: {self.speed_linear:.1f}, Angular: {self.speed_angular:.1f}"
                                )
                            elif next2 == "B":  # Down arrow
                                self.speed_linear = max(self.speed_linear - 0.1, 0.0)
                                self.speed_strafe = max(self.speed_strafe - 0.1, 0.0)
                                print(
                                    f"\n[Kecepatan] Linear/Strafe: {self.speed_linear:.1f}, Angular: {self.speed_angular:.1f}"
                                )
                            elif next2 == "C":  # Right arrow
                                self.speed_angular = max(self.speed_angular - 0.1, 0.0)
                                print(
                                    f"\n[Kecepatan] Linear/Strafe: {self.speed_linear:.1f}, Angular: {self.speed_angular:.1f}"
                                )
                            elif next2 == "D":  # Left arrow
                                self.speed_angular = min(self.speed_angular + 0.1, 3.0)
                                print(
                                    f"\n[Kecepatan] Linear/Strafe: {self.speed_linear:.1f}, Angular: {self.speed_angular:.1f}"
                                )
                    else:
                        # Untuk kombinasi key, kita perlu baca key berikutnya
                        if key in ["w", "s", "a", "d"]:
                            # Cek apakah ada key kedua untuk diagonal
                            if self.is_data(timeout=0.05):
                                key2 = sys.stdin.read(1)
                                # Kombinasi diagonal
                                if key == "w":
                                    self.twist.twist.linear.x = self.speed_linear
                                elif key == "s":
                                    self.twist.twist.linear.x = -self.speed_linear
                                elif key == "a":
                                    self.twist.twist.linear.y = self.speed_strafe
                                elif key == "d":
                                    self.twist.twist.linear.y = -self.speed_strafe

                                if key2 == "a":
                                    self.twist.twist.linear.y = self.speed_strafe
                                elif key2 == "d":
                                    self.twist.twist.linear.y = -self.speed_strafe
                else:
                    # Auto-stop jika tidak ada input
                    self.twist.twist.linear.x = 0.0
                    self.twist.twist.linear.y = 0.0
                    self.twist.twist.angular.z = 0.0

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            # Stop robot
            self.twist.twist.linear.x = 0.0
            self.twist.twist.linear.y = 0.0
            self.twist.twist.angular.z = 0.0
            self.publish_twist()

    def is_data(self, timeout=0.01):
        """Cek apakah ada data di stdin"""
        import select

        return select.select([sys.stdin], [], [], timeout)[0] != []


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
