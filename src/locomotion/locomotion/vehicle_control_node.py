#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class VehicleControlNode(Node):

    def __init__(self):
        super().__init__("vehicle_control_node")

        # -----------------------------
        # Conversion Constants
        # -----------------------------
        self.SPEED_TO_ERPM_GAIN = 4123.4

        # -----------------------------
        # Subscriber
        # -----------------------------
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            "/drive",
            self.drive_callback,
            10,
        )

        # -----------------------------
        # Publishers
        # -----------------------------
        self.motor_pub = self.create_publisher(
            Float64,
            "/commands/motor/speed",
            10,
        )

        self.servo_pub = self.create_publisher(
            Float64,
            "/commands/servo/position",
            10,
        )

        self.get_logger().info("====================================")
        self.get_logger().info(" Vehicle Control Node Started")
        self.get_logger().info("====================================")

    def drive_callback(self, msg: AckermannDriveStamped):

        # ---------------------------------
        # Read desired command
        # ---------------------------------
        desired_speed = msg.drive.speed              # m/s
        desired_steering = msg.drive.steering_angle  # radians (temporary)

        # ---------------------------------
        # Convert speed to ERPM
        # ---------------------------------
        motor_speed = desired_speed * self.SPEED_TO_ERPM_GAIN

        # ---------------------------------
        # Steering conversion
        # TEMPORARY:
        # Directly pass through until
        # servo calibration is completed.
        # ---------------------------------
        servo_position = desired_steering

        # ---------------------------------
        # Create messages
        # ---------------------------------
        motor_msg = Float64()
        motor_msg.data = motor_speed

        servo_msg = Float64()
        servo_msg.data = servo_position

        # ---------------------------------
        # Publish
        # ---------------------------------
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)

        # ---------------------------------
        # Logging
        # ---------------------------------
        self.get_logger().info(
            f"Input  -> Speed: {desired_speed:.2f} m/s | "
            f"Steering: {desired_steering:.2f} rad"
        )

        self.get_logger().info(
            f"Output -> Motor: {motor_speed:.1f} ERPM | "
            f"Servo: {servo_position:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = VehicleControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
