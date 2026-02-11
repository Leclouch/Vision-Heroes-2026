#!/usr/bin/env python3
# Copyright 2024 - TwistStamped Keyboard Teleop
#
# Mirip dengan teleop_twist_keyboard bawaan ROS2,
# tapi menggunakan TwistStamped pada topic /cmd_vel
#
# Kontrol:
#   u    i    o
#   j    k    l
#   m    ,    .
#
# q/z : naikkan/turunkan kecepatan linear dan angular
# w/x : naikkan/turunkan HANYA kecepatan linear
# e/c : naikkan/turunkan HANYA kecepatan angular
# k   : stop / semua kecepatan = 0
# Ctrl-C : keluar

import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Platform-safe terminal input
if sys.platform == 'win32':
    import msvcrt
else:
    import tty
    import termios

BANNER = """
TwistStamped Keyboard Teleop
-----------------------------
Kontrol gerakan:
   u    i    o
   j    k    l
   m    ,    .

q/z : +/- kecepatan linear dan angular (10%)
w/x : +/- HANYA kecepatan linear       (10%)
e/c : +/- HANYA kecepatan angular      (10%)

space / k : STOP

t/b : +/- kecepatan holonomic (sumbu y)
y/n : +/- sumbu z / up-down

CTRL-C untuk keluar
-----------------------------
"""

# Peta tombol -> (linear_x, linear_y, linear_z, angular_z)
MOVE_BINDINGS = {
    'i': ( 1,  0,  0,  0),
    'o': ( 1,  0,  0, -1),
    'j': ( 0,  0,  0,  1),
    'l': ( 0,  0,  0, -1),
    'u': ( 1,  0,  0,  1),
    ',': (-1,  0,  0,  0),
    '.': (-1,  0,  0,  1),
    'm': (-1,  0,  0, -1),
    'O': ( 1, -1,  0,  0),
    'I': ( 1,  0,  0,  0),
    'J': ( 0,  1,  0,  0),
    'L': ( 0, -1,  0,  0),
    'U': ( 1,  1,  0,  0),
    '<': (-1,  0,  0,  0),
    '>': (-1, -1,  0,  0),
    'M': (-1,  1,  0,  0),
    't': ( 0,  0,  1,  0),
    'b': ( 0,  0, -1,  0),
}

# Peta tombol kecepatan -> (speed_multiplier, turn_multiplier)
SPEED_BINDINGS = {
    'q': (1.1,  1.1),
    'z': (0.9,  0.9),
    'w': (1.1,  1.0),
    'x': (0.9,  1.0),
    'e': (1.0,  1.1),
    'c': (1.0,  0.9),
}


def get_key(settings):
    """Baca satu karakter dari keyboard tanpa menunggu Enter."""
    if sys.platform == 'win32':
        key = msvcrt.getwch()
        return key
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def save_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def print_speed(speed, turn):
    print(f"\rKecepatan Linear: {speed:.2f}  |  Kecepatan Angular: {turn:.2f}   ", end="")


class TeleopTwistStampedKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_stamped_keyboard')

        # Declare & get parameters
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('speed_limit', 1000.0)
        self.declare_parameter('turn_limit', 1000.0)
        self.declare_parameter('repeat_rate', 0.0)
        self.declare_parameter('key_timeout', 0.5)
        self.declare_parameter('stamped', True)
        self.declare_parameter('frame_id', 'base_link')

        self.speed      = self.get_parameter('speed').value
        self.turn       = self.get_parameter('turn').value
        self.speed_limit = self.get_parameter('speed_limit').value
        self.turn_limit  = self.get_parameter('turn_limit').value
        self.repeat_rate = self.get_parameter('repeat_rate').value
        self.key_timeout = self.get_parameter('key_timeout').value
        self.frame_id    = self.get_parameter('frame_id').value

        # Publisher TwistStamped
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.get_logger().info("TeleopTwistStampedKeyboard node started.")
        self.get_logger().info(f"Publishing TwistStamped ke: /cmd_vel")
        self.get_logger().info(f"frame_id: {self.frame_id}")

    def publish_twist(self, x, y, z, th, x_h=0.0):
        """Buat pesan TwistStamped dan publish."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.twist.linear.x  = x   * self.speed
        msg.twist.linear.y  = y   * self.speed
        msg.twist.linear.z  = z   * self.speed
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = th  * self.turn

        self.pub.publish(msg)

    def publish_stop(self):
        """Publish pesan berhenti."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # semua nol (default)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistStampedKeyboard()

    settings = save_terminal_settings()

    print(BANNER)
    print_speed(node.speed, node.turn)

    # Jalankan rclpy spin di thread terpisah
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    x   = 0.0
    y   = 0.0
    z   = 0.0
    th  = 0.0
    status = 0

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                x  = float(MOVE_BINDINGS[key][0])
                y  = float(MOVE_BINDINGS[key][1])
                z  = float(MOVE_BINDINGS[key][2])
                th = float(MOVE_BINDINGS[key][3])

            elif key in SPEED_BINDINGS:
                node.speed = min(node.speed * SPEED_BINDINGS[key][0], node.speed_limit)
                node.turn  = min(node.turn  * SPEED_BINDINGS[key][1], node.turn_limit)
                print_speed(node.speed, node.turn)

                # Tampilkan reminder setiap 15 perubahan kecepatan
                status += 1
                if status == 15:
                    print(BANNER)
                    status = 0

            elif key == ' ' or key == 'k':
                # STOP
                x  = 0.0
                y  = 0.0
                z  = 0.0
                th = 0.0

            else:
                # Jika tombol tidak dikenal atau Ctrl-C
                x  = 0.0
                y  = 0.0
                z  = 0.0
                th = 0.0
                if key == '\x03':  # Ctrl-C
                    break

            node.publish_twist(x, y, z, th)

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # Pastikan robot berhenti saat node mati
        node.publish_stop()
        restore_terminal_settings(settings)
        rclpy.shutdown()
        print("\nNode dihentikan. Robot di-stop.")


if __name__ == '__main__':
    main()