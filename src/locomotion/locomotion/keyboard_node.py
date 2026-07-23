#!/usr/bin/env python3

import sys
import tty
import termios
import select
import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        # Publisher
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # Publish at 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.publish_command
        )

        # Vehicle state
        self.target_speed = 1.0      # m/s
        self.current_speed = 0.0
        self.steering_angle =0.0

        self.speed_step = 0.5
        self.steering_step = math.radians(2.0)
        self.max_steering_angle = 0.36  # radians
        self.max_speed = 8.0
        self.min_speed = -8.0

        #cli control instructions
        self.get_logger().info("")
        self.get_logger().info("===== Keyboard Controls =====")
        self.get_logger().info("W : Forward")
        self.get_logger().info("S : Reverse")
        self.get_logger().info("A : Left")
        self.get_logger().info("D : Right")
        self.get_logger().info("C : Center Steering")
        self.get_logger().info("X : Stop")
        self.get_logger().info("+ : Increase Target Speed")
        self.get_logger().info("- : Decrease Target Speed")
        self.get_logger().info("Q : Quit")
        self.get_logger().info("=============================")

    def publish_command(self):

        msg = AckermannDriveStamped()

        msg.drive.speed = self.current_speed

        msg.drive.steering_angle = self.steering_angle

        self.publisher.publish(msg)

    def get_key(self, timeout=0.05):

        fd = sys.stdin.fileno()

        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)

            rlist, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                timeout
            )

            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''

        finally:
            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old_settings
            )

        return key

    def process_key(self, key):

        if key == '':
            return True

        key = key.lower()

        if key == 'w':
            self.current_speed = self.target_speed

        elif key == 's':
            self.current_speed = -self.target_speed

        elif key == 'x':
            self.current_speed = 0.0

        elif key == 'a':
            self.steering_angle -= self.steering_step
            self.steering_angle = max(
                -self.max_steering_angle,
                min(self.max_steering_angle, self.steering_angle)
            )

        elif key == 'd':
            self.steering_angle += self.steering_step
            self.steering_angle = max(
                -self.max_steering_angle,
                min(self.max_steering_angle, self.steering_angle)
            )

        elif key == 'c':
            self.steering_angle = 0.0

        elif key == '+':
            self.target_speed = min(
                self.max_speed,
                self.target_speed + self.speed_step
            )

        elif key == '-':
            self.target_speed = max(
                0.5,
                self.target_speed - self.speed_step
            )

        elif key == 'q':
            return False

        else:
            return True

        self.get_logger().info(
            f"Target Speed: {self.target_speed:.2f} m/s | "
            f"Current Speed: {self.current_speed:.2f} m/s | "
            f"Steering Angle: {self.steering_angle:.3f} rad"
        )

        return True
def main(args=None):

    rclpy.init(args=args)

    node = KeyboardNode()

    try:

        while rclpy.ok():

            key = node.get_key(0.05)

            if not node.process_key(key):
                break

            # Allow timer callbacks (20 Hz publisher) to execute
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
