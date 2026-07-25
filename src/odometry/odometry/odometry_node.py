#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Odometry


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        # Declare Parameters

        # Speed conversion
        self.declare_parameter("speed_to_erpm_gain", 4123.4)
        self.declare_parameter("speed_to_erpm_offset", 0.0)

        # Steering conversion
        self.declare_parameter("steering_to_servo_gain", 1.2135)
        self.declare_parameter("steering_to_servo_offset", 0.5405)

        # Vehicle parameters
        self.declare_parameter("wheelbase", 0.33)

        # Frames
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # TF
        self.declare_parameter("publish_tf", False)

        # Read Parameters

        self.speed_to_erpm_gain = self.get_parameter(
            "speed_to_erpm_gain"
        ).value

        self.speed_to_erpm_offset = self.get_parameter(
            "speed_to_erpm_offset"
        ).value

        self.steering_to_servo_gain = self.get_parameter(
            "steering_to_servo_gain"
        ).value

        self.steering_to_servo_offset = self.get_parameter(
            "steering_to_servo_offset"
        ).value

        self.wheelbase = self.get_parameter(
            "wheelbase"
        ).value

        self.odom_frame = self.get_parameter(
            "odom_frame"
        ).value

        self.base_frame = self.get_parameter(
            "base_frame"
        ).value

        self.publish_tf = self.get_parameter(
            "publish_tf"
        ).value

        # Vehicle State

        self.current_erpm = 0.0
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        self.angular_velocity = 0.0

        # Wait until a steering command is received
        self.received_servo = False

        # Vehicle Pose

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Time

        self.previous_time = self.get_clock().now()
        self.dt = 0.0

        # Subscribers

        self.vesc_sub = self.create_subscription(
            VescStateStamped,
            "/sensors/core",
            self.vesc_callback,
            10,
        )

        self.servo_sub = self.create_subscription(
            Float64,
            "/sensors/servo_position_command",
            self.servo_callback,
            10,
        )

        # Publisher

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

    # VESC Callback

    def vesc_callback(self, msg):

        # Wait until a steering command has been received
        if not self.received_servo:
            return

        # Read ERPM
        self.current_erpm = msg.state.speed

        # Convert ERPM -> Linear Speed (m/s)
        self.current_speed = (
            self.current_erpm
            - self.speed_to_erpm_offset
        ) / self.speed_to_erpm_gain

        # Compute dt
        current_time = self.get_clock().now()

        self.dt = (
            current_time - self.previous_time
        ).nanoseconds * 1e-9

        self.previous_time = current_time

        if self.dt <= 0.0:
            return

        # Bicycle Model

        self.angular_velocity = (
            self.current_speed
            / self.wheelbase
        ) * math.tan(self.current_steering_angle)

        # Forward Euler Integration

        self.x += (
            self.current_speed
            * math.cos(self.theta)
            * self.dt
        )

        self.y += (
            self.current_speed
            * math.sin(self.theta)
            * self.dt
        )

        self.theta += (
            self.angular_velocity
            * self.dt
        )

        # Odometry Message
        odom = Odometry()

        # Header
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (Yaw -> Quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(
            self.theta / 2.0
        )
        odom.pose.pose.orientation.w = math.cos(
            self.theta / 2.0
        )

        # Linear Velocity
        odom.twist.twist.linear.x = self.current_speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        # Angular Velocity
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_velocity

        # Publish
        self.odom_pub.publish(odom)

    # Servo Callback

    def servo_callback(self, msg):

        self.current_steering_angle = (
            msg.data
            - self.steering_to_servo_offset
        ) / self.steering_to_servo_gain

        self.received_servo = True


def main(args=None):

    rclpy.init(args=args)

    node = OdometryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()