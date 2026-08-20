import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class WallFollow(Node):
    """Follow a wall on the vehicle's left using a two-ray LiDAR estimate."""

    def __init__(self):
        super().__init__('wall_follow_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('desired_distance', 0.7)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('kp', 0.88)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.55)
        self.declare_parameter('integral_limit', 2.0)
        self.declare_parameter('max_steering_angle', 0.36)
        self.declare_parameter('fast_speed', 1.5)
        self.declare_parameter('medium_speed', 1.0)
        self.declare_parameter('slow_speed', 0.75)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.desired_distance = self.get_parameter('desired_distance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.range_max = 0.0

        scan_topic = self.get_parameter('scan_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)

    def get_range(self, ranges, angle):
        """Return a valid range at an angle, replacing invalid readings."""
        index = round((angle - self.angle_min) / self.angle_increment)
        index = max(0, min(index, len(ranges) - 1))
        value = ranges[index]
        if not math.isfinite(value):
            return self.range_max
        return max(self.range_min, min(value, self.range_max))

    def get_error(self, ranges):
        """Estimate left-wall distance one metre ahead and return its error."""
        front_left = self.get_range(ranges, math.radians(45.0))
        left = self.get_range(ranges, math.radians(90.0))
        theta = math.radians(45.0)
        alpha = math.atan2(
            front_left * math.cos(theta) - left,
            front_left * math.sin(theta),
        )
        wall_distance = left * math.cos(alpha)
        predicted_distance = wall_distance + self.lookahead_distance * math.sin(alpha)
        return self.desired_distance - predicted_distance

    def publish_drive(self, error):
        now = self.get_clock().now()
        dt = (now - self.previous_time).nanoseconds * 1e-9
        dt = max(dt, 1e-3)
        self.previous_time = now
        self.integral = max(-self.integral_limit, min(
            self.integral_limit, self.integral + error * dt))
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        # The vehicle's steering convention is opposite to the reference PID
        # sign, so publish the positive PID output for a positive error.
        steering = self.kp * error + self.ki * self.integral + self.kd * derivative
        steering = max(-self.max_steering_angle, min(self.max_steering_angle, steering))

        if abs(error) < 0.1:
            speed = self.get_parameter('fast_speed').value
        elif abs(error) < 0.3:
            speed = self.get_parameter('medium_speed').value
        else:
            speed = self.get_parameter('slow_speed').value

        drive = AckermannDriveStamped()
        drive.drive.steering_angle = steering
        drive.drive.speed = speed
        self.drive_pub.publish(drive)
        self.get_logger().info(
            f'wall_error={error:.3f} m, steering={steering:.3f} rad, '
            f'speed={speed:.2f} m/s'
        )

    def scan_callback(self, msg):
        if not msg.ranges or msg.angle_increment == 0.0:
            self.get_logger().warning('Received an invalid LaserScan; ignoring it')
            return
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.publish_drive(self.get_error(msg.ranges))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
