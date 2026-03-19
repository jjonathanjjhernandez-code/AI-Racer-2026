#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking using iTTC.
    """

    def __init__(self):
        super().__init__("safety_node")

        # Declare parameters
        self.declare_parameter("ttc_threshold", 0.5)
        self.declare_parameter("speed_threshold", 0.1)

        # Get parameters
        self.ttc_threshold = self.get_parameter("ttc_threshold").value
        self.speed_threshold = self.get_parameter("speed_threshold").value

        # Initialize variables
        self.speed = 0.0

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 10
        )

        # Create publisher for brake commands
        self.brake_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        self.get_logger().info("=" * 50)
        self.get_logger().info("Safety Node initialized")
        self.get_logger().info(f"TTC Threshold: {self.ttc_threshold} seconds")
        self.get_logger().info(f"Speed Threshold: {self.speed_threshold} m/s")
        self.get_logger().info("=" * 50)

        self.brake_count = 0
        self.brake_threshold_count = 3  # Must trigger 3 times in a row

    def odom_callback(self, odom_msg):
        """
        Update current speed from odometry.
        The x component of linear velocity is the longitudinal speed.
        """
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        """
        Process laser scan and trigger emergency brake if needed.
        Calculates iTTC for all beams and brakes if minimum is below threshold.
        """
        # Skip processing if car is nearly stationary
        if abs(self.speed) < self.speed_threshold:
            return

        # Extract scan data
        ranges = np.array(scan_msg.ranges)

        # Calculate angle for each beam
        num_beams = len(ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment

        # Calculate range rate for each beam
        # ṙ = -v_x * cos(θ)
        range_rates = -self.speed * np.cos(angles)

        # Initialize iTTC array with infinity
        ittc = np.full(num_beams, np.inf)

        # Only calculate iTTC where range rate is negative (approaching)
        approaching = range_rates < 0

        # Calculate iTTC: iTTC = r / |ṙ| for approaching obstacles
        ittc[approaching] = ranges[approaching] / np.abs(range_rates[approaching])

        # Handle invalid measurements
        ittc = np.nan_to_num(ittc, nan=np.inf, posinf=np.inf, neginf=np.inf)

        # Filter out measurements outside valid range
        invalid_ranges = (ranges < scan_msg.range_min) | (ranges > scan_msg.range_max)
        ittc[invalid_ranges] = np.inf

        # Find minimum iTTC
        min_ittc = np.min(ittc)

        # Debug logging
        if min_ittc < self.ttc_threshold:
            min_idx = np.argmin(ittc)
            min_angle = angles[min_idx]
            min_range = ranges[min_idx]

            self.get_logger().warn(
                f"Collision Warning! iTTC: {min_ittc:.3f}s | "
                f"Range: {min_range:.2f}m | Angle: {np.degrees(min_angle):.1f}°",
                throttle_duration_sec=0.5,
            )

        # Trigger emergency brake if iTTC is below threshold
        if min_ittc < self.ttc_threshold:
            self.publish_brake()
        # define safety regions (in radians)
        front_region = (-0.5, 0.5)
        side_region_threshold = 0.3  # More permissive for sides

        # Apply region-specific thresholds
        front_mask = (angles >= front_region[0]) & (angles <= front_region[1])

        # Check front region with stricter threshold
        if np.any(ittc[front_mask] < self.ttc_threshold):
            self.publish_brake()
            return

        # Check sides with more permissive threshold
        if np.min(ittc) < side_region_threshold:
            self.publish_brake()

        if min_ittc < self.ttc_threshold:
            self.brake_count += 1
        else:
            self.brake_count = 0

        # Only brake if consistently detecting collision
        if self.brake_count >= self.brake_threshold_count:
            self.publish_brake()

    def publish_brake(self):
        """
        Publish emergency brake command (speed = 0).
        """
        brake_msg = AckermannDriveStamped()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.header.frame_id = "base_link"
        brake_msg.drive.speed = 0.0

        self.brake_pub.publish(brake_msg)
        self.get_logger().info(
            "🛑 EMERGENCY BRAKE ACTIVATED! 🛑", throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
