#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class VehicleControlNode(Node):

    def __init__(self):
        super().__init__("vehicle_control_node")

        # Vehicle Parameters
        self.declare_parameter("speed_to_erpm_gain", 4123.4)
        self.declare_parameter("speed_to_erpm_offset", 0.0)

        self.declare_parameter("steering_to_servo_gain", 1.2135)
        self.declare_parameter("steering_to_servo_offset", 0.5405)

        self.declare_parameter("max_steering_angle", 0.36)

        self.declare_parameter("servo_min", 0.15)
        self.declare_parameter("servo_max", 0.85)

        self.declare_parameter("max_speed", 8.0)
        self.declare_parameter("min_speed", -8.0)

        self.SPEED_TO_ERPM_GAIN = self.get_parameter("speed_to_erpm_gain").value
        self.SPEED_TO_ERPM_OFFSET = self.get_parameter("speed_to_erpm_offset").value

        self.STEERING_TO_SERVO_GAIN = self.get_parameter("steering_to_servo_gain").value
        self.STEERING_TO_SERVO_OFFSET = self.get_parameter("steering_to_servo_offset").value

        self.MAX_STEERING_ANGLE = self.get_parameter("max_steering_angle").value

        self.SERVO_MIN = self.get_parameter("servo_min").value
        self.SERVO_MAX = self.get_parameter("servo_max").value

        self.MAX_SPEED = self.get_parameter("max_speed").value
        self.MIN_SPEED = self.get_parameter("min_speed").value

        # Subscriber
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            "/drive",
            self.drive_callback,
            10,
        )
        # Publishers
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

        # Read desired command
        desired_speed = msg.drive.speed              # m/s
        desired_steering = msg.drive.steering_angle  # radians
        desired_steering = max(-self.MAX_STEERING_ANGLE, min(self.MAX_STEERING_ANGLE, desired_steering)) #clamping angle

        # Convert speed to ERPM
        motor_speed = desired_speed * self.SPEED_TO_ERPM_GAIN + self.SPEED_TO_ERPM_OFFSET

        # Steering conversion
        servo_position = desired_steering * self.STEERING_TO_SERVO_GAIN + self.STEERING_TO_SERVO_OFFSET

        
        # Create messages
        motor_msg = Float64()
        motor_msg.data = motor_speed

        servo_msg = Float64()
        servo_msg.data = servo_position

        # Publish
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)

        # Logging
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
