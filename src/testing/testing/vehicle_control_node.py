#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class VehicleControlNode(Node):
    """Convert /drive commands to VESC motor and servo commands for testing."""

    # Calibration constants intentionally live here rather than ROS parameters.
    SPEED_TO_ERPM_GAIN = 4142.11
    SPEED_TO_ERPM_OFFSET = 0.0
    STEERING_TO_SERVO_GAIN = 1.2135
    STEERING_TO_SERVO_OFFSET = 0.583
    SERVO_MIN = 0.32
    SERVO_MAX = 0.85
    MAX_SPEED = 8.0
    MIN_SPEED = -8.0

    def __init__(self):
        super().__init__('testing_vehicle_control_node')

        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, '/drive', self.drive_callback, 10)
        self.motor_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)

        self.get_logger().info('Testing vehicle control node started')

    def drive_callback(self, msg):
        desired_speed = msg.drive.speed
        desired_steering = msg.drive.steering_angle

        motor_speed = (
            desired_speed * self.SPEED_TO_ERPM_GAIN + self.SPEED_TO_ERPM_OFFSET)
        servo_position = (
            desired_steering * self.STEERING_TO_SERVO_GAIN
            + self.STEERING_TO_SERVO_OFFSET)
        servo_position = max(self.SERVO_MIN, min(self.SERVO_MAX, servo_position))

        motor_msg = Float64()
        motor_msg.data = motor_speed
        servo_msg = Float64()
        servo_msg.data = servo_position

        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)

        self.get_logger().info(
            f'Input -> speed: {desired_speed:.2f} m/s | '
            f'steering: {desired_steering:.2f} rad')
        self.get_logger().info(
            f'Output -> motor: {motor_speed:.1f} ERPM | '
            f'servo: {servo_position:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
