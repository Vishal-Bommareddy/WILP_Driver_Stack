#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

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

        # Publisher
        self.ebrake_pub = self.create_publisher(
            Bool,
            '/ebrake',
            10
        )

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):

        ranges = np.array(scan_msg.ranges)

        angles = scan_msg.angle_min + \
            np.arange(len(ranges)) * scan_msg.angle_increment

        # Closing speed for each beam
        range_rates = -self.speed * np.cos(angles)

        ttc = np.where(
            range_rates >= 0,
            np.inf,
            ranges / (-range_rates)
        )

        min_ttc = np.min(ttc)

        self.get_logger().info(
            f"Speed: {self.speed:.2f} m/s | Min TTC: {min_ttc:.2f} s"
        )

        brake_msg = Bool()

        if min_ttc < self.ttc_threshold:

            self.get_logger().warn(
                f"EMERGENCY BRAKE! TTC = {min_ttc:.3f} s | "
                f"Speed = {self.speed:.2f} m/s"
            )

            brake_msg.data = True

        else:
            brake_msg.data = False

        self.ebrake_pub.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)

    node = SafetyNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()