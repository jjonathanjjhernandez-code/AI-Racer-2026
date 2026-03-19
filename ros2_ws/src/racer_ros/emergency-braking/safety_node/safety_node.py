#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')

        # Declare parameters
        self.declare_parameter('ttc_threshold', 0.5)  # seconds
        self.declare_parameter('speed_threshold', 0.1)  # m/s minimum speed to check

        # Get parameters
        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value

        # Initialize variables
        self.speed = 0.0

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        # Create publisher for brake commands
        self.brake_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        self.get_logger().info('Safety Node initialized')
        self.get_logger().info(f'TTC Threshold: {self.ttc_threshold} seconds')
        self.get_logger().info(f'Speed Threshold: {self.speed_threshold} m/s')

    def odom_callback(self, odom_msg):
        """
        Update current speed from odometry
        """
        # Extract longitudinal velocity (x component in vehicle frame)
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        """
        Process laser scan and trigger emergency brake if needed
        """
        # TODO: Implement iTTC calculation and braking logic
            # Skip processing if car is nearly stationary
        if abs(self.speed) < self.speed_threshold:
            return

        # Extract scan data
        ranges = np.array(scan_msg.ranges)

        # Calculate angle for each beam
        # angle_i = angle_min + i * angle_increment
        num_beams = len(ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment

        # Calculate range rate for each beam
        # ṙ = -v_x * cos(θ)
        # Negative sign because approaching obstacle means decreasing range
        range_rates = -self.speed * np.cos(angles)

        # Calculate iTTC for each beam
        # iTTC = r / max(-ṙ, 0)
        # We need -ṙ because we want positive values for approaching obstacles
        # The max(..., 0) ensures we only consider negative range rates (approaching)

        # Initialize iTTC array with infinity
        ittc = np.full(num_beams, np.inf)

        # Only calculate iTTC where range rate is negative (approaching obstacle)
        approaching = range_rates < 0

        # Calculate iTTC for approaching obstacles
        # iTTC = range / abs(range_rate)
        ittc[approaching] = ranges[approaching] / np.abs(range_rates[approaching])

        # Handle invalid measurements (inf, nan, out of range)
        # Replace inf and nan with a large number to avoid triggering brake
        ittc = np.nan_to_num(ittc, nan=np.inf, posinf=np.inf, neginf=np.inf)

        # Also filter out measurements outside valid range
        invalid_ranges = (ranges < scan_msg.range_min) | (ranges > scan_msg.range_max)
        ittc[invalid_ranges] = np.inf

        # Find minimum iTTC
        min_ittc = np.min(ittc)

        # Log for debugging
        if min_ittc < self.ttc_threshold:
            self.get_logger().warn(
                f'Collision imminent! Min iTTC: {min_ittc:.3f}s (threshold: {self.ttc_threshold}s)',
                throttle_duration_sec=1.0)  # Throttle to avoid spam

        # Trigger emergency brake if iTTC is below threshold
        if min_ittc < self.ttc_threshold:
            self.publish_brake()

    def publish_brake(self):
        """
        Publish emergency brake command
        """
        brake_msg = AckermannDriveStamped()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.header.frame_id = 'base_link'

        # Set speed to 0 to brake
        brake_msg.drive.speed = 0.0

        self.brake_pub.publish(brake_msg)
        self.get_logger().info('EMERGENCY BRAKE ACTIVATED!', throttle_duration_sec=1.0)
        

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

if __name__ == '__main__':
    main()