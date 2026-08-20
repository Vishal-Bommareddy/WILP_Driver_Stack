#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class SafetyNode(Node):
    """
    Automatic Emergency Braking (AEB) node.

    Computes Time-To-Collision (TTC) from LiDAR data and publishes an
    emergency brake signal when the TTC falls below a threshold.
    """

    def __init__(self):
        super().__init__('safety_node')

        self.speed = 0.0
        self.ttc_threshold = 0.5
        self.declare_parameter('aeb_half_angle', np.deg2rad(30.0))
        self.aeb_half_angle = self.get_parameter('aeb_half_angle').value
        self.declare_parameter('aeb_range_hysteresis', 0.05)
        self.range_hysteresis = self.get_parameter('aeb_range_hysteresis').value
        self.aeb_active = False
        self.trigger_range = None
        self.latest_steering = 0.0

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )

        # Publisher
        self.ebrake_pub = self.create_publisher(
            Bool,
            '/emergency',
            10
        )
        self.aeb_pub = self.create_publisher(AckermannDriveStamped, '/aeb', 10)

    def drive_callback(self, drive_msg):
        self.latest_steering = drive_msg.drive.steering_angle

    def set_aeb_state(self, active, trigger_range=None):
        self.aeb_active = active
        self.trigger_range = trigger_range if active else None

    def publish_aeb(self):
        emergency_msg = Bool()
        emergency_msg.data = self.aeb_active
        self.ebrake_pub.publish(emergency_msg)

        if self.aeb_active:
            aeb_msg = AckermannDriveStamped()
            aeb_msg.drive.speed = 0.0
            aeb_msg.drive.steering_angle = self.latest_steering
            self.aeb_pub.publish(aeb_msg)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):

        ranges = np.array(scan_msg.ranges, dtype=float)
        angles = scan_msg.angle_min + \
            np.arange(len(ranges)) * scan_msg.angle_increment
        forward_mask = np.abs(angles) <= self.aeb_half_angle
        valid_mask = forward_mask & np.isfinite(ranges) & (ranges > 0.0)
        valid_ranges = ranges[valid_mask]
        min_range = np.min(valid_ranges) if valid_ranges.size else None

        # Closing speed for each beam
        range_rates = -self.speed * np.cos(angles)

        safe_ranges = np.where(valid_mask, ranges, np.inf)
        ttc = np.where(
            range_rates >= 0,
            np.inf,
            safe_ranges / (-range_rates)
        )

        min_ttc = np.min(ttc) if ttc.size else np.inf
        self.get_logger().info(
            f"Speed: {self.speed:.2f} m/s | Min TTC: {min_ttc:.2f} s"
        )

        # TTC arms AEB once; after that, range alone keeps it latched.
        if not self.aeb_active:
            if min_range is not None and min_ttc < self.ttc_threshold:
                self.set_aeb_state(True, min_range)
                self.get_logger().warn(
                    f"AEB active: TTC = {min_ttc:.3f} s | "
                    f"Trigger range = {min_range:.3f} m"
                )
        else:
            # Keep AEB latched until the closest valid return is safely clear.
            if (min_range is None or
                    min_range > self.trigger_range + self.range_hysteresis):
                self.set_aeb_state(False)
                self.get_logger().info(
                    "AEB cleared: minimum obstacle range is safe or invalid"
                )

        self.publish_aeb()


def main(args=None):
    rclpy.init(args=args)

    node = SafetyNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
