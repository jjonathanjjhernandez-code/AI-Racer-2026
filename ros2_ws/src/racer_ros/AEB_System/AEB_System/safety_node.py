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
        super().__init__('AEB_System')

        # Declare parameters
        self.declare_parameter('ttc_threshold', 2.0)
        self.declare_parameter('speed_threshold', 0.5)

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

        self.get_logger().info('='*50)
        self.get_logger().info('Safety Node initialized')
        self.get_logger().info(f'TTC Threshold: {self.ttc_threshold} seconds')
        self.get_logger().info(f'Speed Threshold: {self.speed_threshold} m/s')
        self.get_logger().info('='*50)

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

        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value
        
        # Skip processing if car is nearly stationary
        if abs(self.speed) < self.speed_threshold:
            return

        # Extract scan data
        ranges = np.array(scan_msg.ranges)

        # Calculate angle for each beam
        num_beams = len(ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment
        
        
        # front================
        front_half_angle = np.radians(15.0)
        front_beams = np.abs(angles) < front_half_angle  
              
        # side================
        side_beams = (np.abs(angles) > np.radians(15.0)) & (np.abs(angles) < np.radians(60.0))        
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


        min_emergency_range = 1.0  # meters
        far_away = ranges > min_emergency_range
        ittc[far_away & front_beams] = np.inf

        # Find minimum iTTC
        min_forward_ittc = np.min(ittc[front_beams]) if front_beams.any() else np.inf
        min_side_ittc = np.min(ittc[side_beams]) if side_beams.any() else np.inf

        # Debug logging
        if min_forward_ittc < self.ttc_threshold:
            front_ittc = ittc.copy()
            front_ittc[~front_beams] = np.inf
            min_idx = np.argmin(front_ittc)
            min_angle = angles[min_idx]
            min_range = ranges[min_idx]

            self.get_logger().warn(
                f'FRONT COLLISION AHHHHHHHHHHHHHh\n'
                f'Collision Warning! iTTC: {min_forward_ittc:.3f}s | '
                f'Range: {min_range:.2f}m | Angle: {np.degrees(min_angle):.1f}°',
                throttle_duration_sec=0.5)
        
        # Debug logging
        if min_side_ittc < self.ttc_threshold * 0.5:
            side_ittc = ittc.copy()
            side_ittc[~side_beams] = np.inf
            min_idx = np.argmin(side_ittc)
            min_angle = angles[min_idx]
            min_range = ranges[min_idx]

            self.get_logger().warn(
                f'SIDE COLLISION (nothing much to worry about)\n'
                f'Collision Warning! iTTC: {min_side_ittc:.3f}s | '
                f'Range: {min_range:.2f}m | Angle: {np.degrees(min_angle):.1f}°',
                throttle_duration_sec=0.5)

        # Trigger emergency brake if iTTC is below threshold
        if min_forward_ittc < self.ttc_threshold:
            self.publish_brake(min_forward_ittc, self.ttc_threshold, emergency=True)
        elif min_side_ittc < self.ttc_threshold * 0.5:
            self.publish_brake(min_side_ittc, self.ttc_threshold, emergency=False)
            
    def publish_brake(self, min_ittc, threshold, emergency=True):
        """
        Publish emergency brake command (speed = 0).
        """
        brake_msg = AckermannDriveStamped()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.header.frame_id = 'base_link'
        if emergency:
            # True emergency — hard stop (speed = 0)
            # Don't do gradual braking that the wall follower can override
            brake_speed = 0.0
            self.get_logger().info(
                f'EMERGENCY STOP | speed was: {self.speed:.2f} m/s',
                throttle_duration_sec=1.0)
        else:
            # Side threat — reduce speed proportionally but don't stop
            closeness = 1.0 - np.clip(min_ittc / threshold, 0.0, 1.0)
            brake_speed = self.speed * (1.0 - closeness * 0.5)  # at most halve speed
            brake_speed = max(brake_speed, 0.0)
            self.get_logger().info(
                f'SIDE BRAKE | reducing speed to: {brake_speed:.2f} m/s',
                throttle_duration_sec=1.0)
 
def main(args=None):
    rclpy.init(args=args)
    AEB_System = SafetyNode()
    try:
        rclpy.spin(AEB_System)
    except KeyboardInterrupt:
        pass
    finally:
        AEB_System.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()

if __name__ == '__main__':
    main()