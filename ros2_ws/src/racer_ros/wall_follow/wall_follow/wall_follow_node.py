#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class WallFollow(Node):
    """ 
    Implement Wall Following on the car using PID control
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        # Declare parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('desired_distance', 1.0)  # meters from wall
        self.declare_parameter('lookahead_distance', 1.0)  # meters
        self.declare_parameter('theta_deg', 50.0)  # angle for beam 'a' in degrees
        self.declare_parameter('max_speed', 20.0)

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.desired_distance = self.get_parameter('desired_distance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.theta_deg = self.get_parameter('theta_deg').value
        self.max_speed = self.get_parameter('max_speed').value


        # Convert theta to radians
        self.theta = np.deg2rad(self.theta_deg)

        # PID control variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Create publisher
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        self.error_pub = self.create_publisher(Float64, '/wall_follow/error', 10)
        self.dist_pub = self.create_publisher(Float64, '/wall_follow/wall_distance', 10)
        self.steering_pub = self.create_publisher(Float64, '/wall_follow/steering_angle', 10)

        self.get_logger().info('='*50)
        self.get_logger().info('Wall Follow Node Initialized')
        self.get_logger().info(f'PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
        self.get_logger().info(f'Desired Distance: {self.desired_distance}m')
        self.get_logger().info(f'Lookahead Distance: {self.lookahead_distance}m')
        self.get_logger().info(f'Theta: {self.theta_deg}°')
        self.get_logger().info('='*50)

    def get_range(self, range_data, angle, window=4):
        """
        Simple helper to return the corresponding range measurement at a given angle.
        Handles NaNs and Infs.

        Args:
            range_data: LaserScan message with range data
            angle: desired angle in radians (relative to car's x-axis)

        Returns:
            range: range measurement in meters at the given angle
        """
        # Calculate the index for the desired angle
        # angle_index = (angle - angle_min) / angle_increment
        angle_index = int((angle - range_data.angle_min) / range_data.angle_increment)

        # Ensure index is within bounds
        angle_index = np.clip(angle_index, 0, len(range_data.ranges) - 1)

        low = max(0, angle_index - window)
        high = min(len(range_data.ranges), angle_index + window + 1)
        
        
        # Get range value
        range_val = [r for r in np.array(range_data.ranges)[low:high]
                     if not np.isnan(r) and not np.isinf(r)]
            # Return a large value or max range

        return np.mean(range_val) if range_val else range_data.range_max

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follows the wall to the right.

        Args:
            range_data: LaserScan message
            dist: desired distance to the wall

        Returns:
            error: calculated error (desired - actual future distance)
        """
        # Get range at 90 degrees (perpendicular to car, pointing right)
        # In standard LiDAR frame: 0° is forward, positive angles go counter-clockwise
        # So 90° to the right is actually -90° or -π/2
        # But we want to follow the wall to the RIGHT
        # Typically: 0° forward, 90° left (π/2), -90° or 270° right (-π/2)

        # For right wall following:
        # b: perpendicular beam to the right (270° or -90° or 3π/2)
        # a: beam at angle theta forward from b

        angle_b = -np.pi/2  # 90 degrees to the right (-90° from forward)
        angle_a = angle_b + self.theta  # theta degrees forward from beam b

        # Get range measurements
        b = self.get_range(range_data, angle_b, window=4)
        a = self.get_range(range_data, angle_a, window=4)

        # Calculate alpha (angle between car's x-axis and the wall)
        # α = atan2((a*cos(θ) - b), (a*sin(θ)))
        numerator = a * np.cos(self.theta) - b
        denominator = a * np.sin(self.theta)

        # Handle edge case where denominator is zero
        if abs(denominator) < 1e-6:
            alpha = 0.0
        else:
            alpha = np.arctan2(numerator, denominator)

        # Calculate current distance to wall (D_t)
        D_t = b * np.cos(alpha)

        # Calculate projected future distance (D_{t+1})
        D_t_plus_1 = D_t + self.lookahead_distance * np.sin(alpha)

        # Calculate error
        error = dist - D_t_plus_1

        return error, D_t
    
    
    def pid_control(self, error, max_velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # Get current time
        current_time = self.get_clock().now()

        # Calculate dt (time since last update)
        if self.prev_time is None:
            dt = 0.0
            self.prev_time = current_time
        else:
            dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
            self.prev_time = current_time

        # Avoid division by zero or very small dt
        if dt < 1e-6:
            dt = 1e-6

        # Proportional term
        P = self.kp * error

        # Integral term (accumulate error over time)
        self.integral += error * dt
        self.integral = max(-10.0, self.integral)
        self.integral = min(10.0, self.integral)
        
        I = self.ki * self.integral

        # Derivative term (rate of change of error)
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative

        # Calculate steering angle
        angle = P + I + D

        # Clamp steering angle to reasonable limits (e.g., ±30 degrees)
        max_steering_angle = np.deg2rad(30.0 - 20.0 * (1.0))
        
        angle = np.clip(angle, -max_steering_angle, max_steering_angle)

        # smooth scaling around angles
        final_velocity = max_velocity * np.clip(np.cos(angle) ** 2, 0.1, 1.0)
        
        adaptive_max_steer = np.deg2rad(30.0 - 20.0 * (final_velocity / max_velocity))
        angle = np.clip(angle, -adaptive_max_steer, adaptive_max_steer)
        
        # Update previous error
        self.prev_error = error

        # Log for debugging (throttled)
        self.get_logger().debug(
            f'Error: {error:.3f}m | P: {P:.3f} | I: {I:.3f} | D: {D:.3f} | '
            f'Angle: {np.rad2deg(angle):.1f}°',
            throttle_duration_sec=0.5)
        
        # Publish debug topics for live plotting
        err_msg = Float64()
        err_msg.data = error
        self.error_pub.publish(err_msg)
 
        steer_msg = Float64()
        steer_msg.data = float(np.rad2deg(angle))
        self.steering_pub.publish(steer_msg)

        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(final_velocity)

        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and 
        publish the drive message.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_speed = self.get_parameter('max_speed').value
        
        self.desired_distance = self.get_parameter('desired_distance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
 
        # Calculate error
        error, wall_dist = self.get_error(msg, self.desired_distance)
 
        # Publish wall distance for live plotting
        dist_msg = Float64()
        dist_msg.data = wall_dist
        self.dist_pub.publish(dist_msg)
        # Apply PID control
        self.pid_control(error, self.max_speed)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    try:
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follow_node.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()


if __name__ == '__main__':
    main()