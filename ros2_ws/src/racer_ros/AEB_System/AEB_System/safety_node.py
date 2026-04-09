#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    Emergency braking node using iTTC (inverse Time-To-Collision).
    """
    def __init__(self):
        super().__init__('AEB_System')

        self.declare_parameter('ttc_threshold', 2.0)
        self.declare_parameter('speed_threshold', 0.5)

        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value

        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self.speed = 0.0
        self.latest_nav_cmd = AckermannDriveStamped()  # last command from movement_node

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Intercept movement_node commands and gate them through AEB
        # movement_node publishes to /drive_nav; we forward to /drive (or replace with brake)
        self.nav_sub = self.create_subscription(
            AckermannDriveStamped, '/drive_nav', self.nav_callback, 10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Safety Node initialized')
        self.get_logger().info(f'TTC Threshold: {self.ttc_threshold} seconds')
        self.get_logger().info(f'Speed Threshold: {self.speed_threshold} m/s')
        self.get_logger().info('=' * 50)

    def _on_parameters_changed(self, params):
        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value
        return SetParametersResult(successful=True)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def nav_callback(self, drive_msg):
        """Forward movement_node commands when AEB is not active."""
        self.latest_nav_cmd = drive_msg

    def scan_callback(self, scan_msg):
        if abs(self.speed) < self.speed_threshold:
            # Car is slow/stationary — AEB not needed, but still forward nav command
            self.drive_pub.publish(self.latest_nav_cmd)
            return

        ranges = np.array(scan_msg.ranges)
        num_beams = len(ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment

        front_beams = np.abs(angles) < np.radians(10.0)
        side_beams = (np.abs(angles) > np.radians(15.0)) & (np.abs(angles) < np.radians(60.0))

        range_rates = -self.speed * np.cos(angles)
        ittc = np.full(num_beams, np.inf)
        approaching = range_rates < 0
        ittc[approaching] = ranges[approaching] / np.abs(range_rates[approaching])
        ittc = np.nan_to_num(ittc, nan=np.inf, posinf=np.inf, neginf=np.inf)

        invalid_ranges = (ranges < scan_msg.range_min) | (ranges > scan_msg.range_max)
        ittc[invalid_ranges] = np.inf

        min_emergency_range = max(1.5, self.speed * 0.20)
        far_away = ranges > min_emergency_range
        ittc[far_away & front_beams] = np.inf

        min_forward_ittc = np.min(ittc[front_beams]) if front_beams.any() else np.inf
        min_side_ittc = np.min(ittc[side_beams]) if side_beams.any() else np.inf

        if min_forward_ittc < self.ttc_threshold:
            front_ittc = ittc.copy()
            front_ittc[~front_beams] = np.inf
            min_idx = np.argmin(front_ittc)
            self.get_logger().warn(
                f'FRONT COLLISION WARNING | iTTC: {min_forward_ittc:.3f}s | '
                f'Range: {ranges[min_idx]:.2f}m | Angle: {np.degrees(angles[min_idx]):.1f}°',
                throttle_duration_sec=0.5)

        if min_side_ittc < self.ttc_threshold * 0.5:
            side_ittc = ittc.copy()
            side_ittc[~side_beams] = np.inf
            min_idx = np.argmin(side_ittc)
            self.get_logger().warn(
                f'SIDE COLLISION WARNING | iTTC: {min_side_ittc:.3f}s | '
                f'Range: {ranges[min_idx]:.2f}m | Angle: {np.degrees(angles[min_idx]):.1f}°',
                throttle_duration_sec=0.5)

        if min_forward_ittc < self.ttc_threshold:
            self.publish_brake(min_forward_ittc, self.ttc_threshold, emergency=True)
        elif min_side_ittc < self.ttc_threshold * 0.5:
            self.publish_brake(min_side_ittc, self.ttc_threshold, emergency=False)
        else:
            # No collision threat — forward the latest navigation command unchanged
            self.drive_pub.publish(self.latest_nav_cmd)

    def publish_brake(self, min_ittc, threshold, emergency=True):
        brake_msg = AckermannDriveStamped()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.header.frame_id = 'base_link'

        if emergency:
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0
            self.get_logger().info(
                f'EMERGENCY STOP | speed was: {self.speed:.2f} m/s',
                throttle_duration_sec=1.0)
        else:
            closeness = 1.0 - np.clip(min_ittc / threshold, 0.0, 1.0)
            brake_speed = max(self.speed * (1.0 - closeness * 0.5), 0.0)
            brake_msg.drive.speed = brake_speed
            self.get_logger().info(
                f'SIDE BRAKE | reducing speed to: {brake_speed:.2f} m/s',
                throttle_duration_sec=1.0)

        # FIX: was never actually publishing — this was the original bug
        self.drive_pub.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()