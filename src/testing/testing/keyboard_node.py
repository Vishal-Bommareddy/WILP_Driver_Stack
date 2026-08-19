#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class KeyboardNode(Node):
    """Keyboard teleoperation for the standalone calibration stack."""

    TARGET_SPEED = 1.0
    SPEED_STEP = 0.5
    STEERING_STEP = math.radians(2.0)
    STEERING_TO_SERVO_GAIN = 1.2135
    STEERING_TO_SERVO_OFFSET = 0.583
    SERVO_MIN = 0.32
    SERVO_MAX = 0.85
    MIN_STEERING_ANGLE = (
        (SERVO_MIN - STEERING_TO_SERVO_OFFSET) / STEERING_TO_SERVO_GAIN)
    MAX_STEERING_ANGLE = (
        (SERVO_MAX - STEERING_TO_SERVO_OFFSET) / STEERING_TO_SERVO_GAIN)
    MAX_SPEED = 8.0
    MIN_TARGET_SPEED = 0.5

    def __init__(self):
        super().__init__('testing_keyboard_node')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.target_speed = self.TARGET_SPEED
        self.current_speed = 0.0
        self.steering_angle = 0.0

        self.get_logger().info(
            'Keyboard controls: W forward, S reverse, A left, D right, '
            'C center, X stop, +/- target speed, Q quit')

    def publish_command(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.current_speed
        msg.drive.steering_angle = self.steering_angle
        self.publisher.publish(msg)

    @staticmethod
    def get_key(timeout=0.05):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            readable, _, _ = select.select([sys.stdin], [], [], timeout)
            return sys.stdin.read(1) if readable else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def process_key(self, key):
        if not key:
            return True

        key = key.lower()
        if key == 'w':
            self.current_speed = self.target_speed
        elif key == 's':
            self.current_speed = -self.target_speed
        elif key == 'x':
            self.current_speed = 0.0
        elif key == 'a':
            self.steering_angle = max(
                self.MIN_STEERING_ANGLE,
                self.steering_angle - self.STEERING_STEP)
        elif key == 'd':
            self.steering_angle = min(
                self.MAX_STEERING_ANGLE,
                self.steering_angle + self.STEERING_STEP)
        elif key == 'c':
            self.steering_angle = 0.0
        elif key == '+':
            self.target_speed = min(self.MAX_SPEED, self.target_speed + self.SPEED_STEP)
        elif key == '-':
            self.target_speed = max(
                self.MIN_TARGET_SPEED, self.target_speed - self.SPEED_STEP)
        elif key == 'q':
            return False
        else:
            return True

        servo_position = (
            self.steering_angle * self.STEERING_TO_SERVO_GAIN
            + self.STEERING_TO_SERVO_OFFSET)
        self.get_logger().info(
            f'Target speed: {self.target_speed:.2f} m/s | '
            f'Current speed: {self.current_speed:.2f} m/s | '
            f'Steering: {self.steering_angle:.3f} rad | '
            f'Servo: {servo_position:.3f}')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        while rclpy.ok():
            if not node.process_key(node.get_key()):
                break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
