#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveFollowGap(Node):
    """ 
    Implement Follow the Gap reactive obstacle avoidance algorithm
    """
    def __init__(self):
        super().__init__('reactive_node')

        # Declare parameters
        self.declare_parameter('bubble_radius', 0.3)  # meters
        self.declare_parameter('preprocess_conv_size', 3)  # window size for averaging
        self.declare_parameter('max_lidar_range', 3.0)  # max range to consider (meters)
        self.declare_parameter('speed_min', 0.5)  # minimum speed (m/s)
        self.declare_parameter('speed_max', 2.0)  # maximum speed (m/s)
        self.declare_parameter('steering_gain', 1.0)  # steering proportional gain

        # Get parameters
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preprocess_conv_size = self.get_parameter('preprocess_conv_size').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_max = self.get_parameter('speed_max').value
        self.steering_gain = self.get_parameter('steering_gain').value

        # Topics
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscriber to LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10)

        # Create publisher to drive
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10)

        self.get_logger().info('='*50)
        self.get_logger().info('Follow the Gap Node Initialized')
        self.get_logger().info(f'Bubble Radius: {self.bubble_radius}m')
        self.get_logger().info(f'Max LiDAR Range: {self.max_lidar_range}m')
        self.get_logger().info(f'Speed Range: {self.speed_min}-{self.speed_max} m/s')
        self.get_logger().info('='*50)
        self.current_speed = 0.0 #speed tracker variable
        
    # extra helper functions needed for the algorithm!

    #====publisher function!====
    def publish_drive(self, steering_angle, speed):
        """
        Track current speed for dynamic bubble
        """
        self.current_speed = speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)

        self.drive_pub.publish(drive_msg)
    #====publisher function!====
    
    #=====clean up function for lidar sensor data!=====
    def preprocess_lidar(self, ranges):
        """ 
        Preprocess the LiDAR scan array.
        1. Apply mean filter over a window
        2. Clip values to max_lidar_range
        3. Handle inf and nan values

        Args:
            ranges: array of LiDAR ranges

        Returns:
            proc_ranges: processed array of ranges
        """
        # Convert to numpy array
        proc_ranges = np.array(ranges)

        # Replace inf and nan with max_lidar_range
        proc_ranges = np.nan_to_num(proc_ranges, 
                                    nan=self.max_lidar_range, 
                                    posinf=self.max_lidar_range, 
                                    neginf=0.0)

        # Clip to max range - anything beyond max_lidar_range is set to max_lidar_range
        proc_ranges = np.clip(proc_ranges, 0, self.max_lidar_range)

        # Apply mean filter (moving average) for smoothing
        # Use convolution for efficient averaging
        if self.preprocess_conv_size > 1:
            kernel = np.ones(self.preprocess_conv_size) / self.preprocess_conv_size
            proc_ranges = np.convolve(proc_ranges, kernel, mode='same')

        return proc_ranges
    #=====clean up function for lidar sensor data!=====

    #====returns gap_length based on cleaned up lidar sensor values====
    def find_max_gap(self, free_space_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges.
        A gap is a continuous sequence of non-zero values.

        Args:
            free_space_ranges: array with 0s for obstacles and non-zero for free space

        Returns:
            start_idx: start index of max gap
            end_idx: end index of max gap (inclusive)
        """
        # Find where free space begins and ends
        # non-zero values represent free space
        mask = free_space_ranges > 0

        # Find the boundaries of all gaps
        # np.diff finds changes (0->1 or 1->0)
        diff = np.diff(np.concatenate(([0], mask.astype(int), [0])))

        # starts: where diff changes from 0 to 1 (gap begins)
        # ends: where diff changes from 1 to 0 (gap ends)
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]

        # If no gaps found, return invalid indices
        if len(starts) == 0:
            self.get_logger().warn('No gaps found in LiDAR data!')
            return 0, 0

        # Calculate gap lengths
        gap_lengths = ends - starts

        # Find the maximum gap
        max_gap_idx = np.argmax(gap_lengths)

        start_idx = starts[max_gap_idx]
        end_idx = ends[max_gap_idx] - 1  # -1 because ends is exclusive

        return start_idx, end_idx
    #====returns gap_length based on cleaned up lidar sensor values====
    
    #====function in determining the closest obstacle to CAR and making a buble to it!====
    def get_bubble_indices(self, data, closest_idx, closest_distance):
        """
        Dynamic bubble radius based on current speed
        """
        # Scale bubble radius with speed (reaction time)
        # Higher speed = larger bubble needed
        dynamic_radius = self.bubble_radius * (1.0 + self.current_speed / self.speed_max)

        if closest_distance == 0:
            closest_distance = 0.1

        bubble_angle = dynamic_radius / closest_distance
        angle_increment = data.angle_increment
        bubble_idx_range = int(np.ceil(bubble_angle / angle_increment))

        start_idx = max(0, closest_idx - bubble_idx_range)
        end_idx = min(len(data.ranges) - 1, closest_idx + bubble_idx_range)

        return np.arange(start_idx, end_idx + 1)
    #====function in determining the closest obstacle to CAR and making a buble to it!====

    #====function used once the maximum gap(start and end indexes)is found for creation of path!====
    def find_best_point(self, start_i, end_i, ranges):
        """
        Enhanced best point selection considering gap depth and width
        """
        if start_i >= end_i or end_i >= len(ranges):
            return len(ranges) // 2

        gap_ranges = ranges[start_i:end_i+1]

        # Calculate gap quality metrics
        gap_width = end_i - start_i
        gap_max_depth = np.max(gap_ranges)
        gap_avg_depth = np.mean(gap_ranges)

        # Weighted selection
        # Favor deeper and wider gaps
        if gap_width > 100 and gap_avg_depth > 2.0:
            # Large, deep gap -> aim for furthest point
            best_idx = start_i + np.argmax(gap_ranges)
        elif gap_width < 50:
            # Narrow gap -> aim for center for safety
            best_idx = (start_i + end_i) // 2
        else:
            # Medium gap -> balanced approach
            furthest_idx = start_i + np.argmax(gap_ranges)
            centered_idx = (start_i + end_i) // 2
            best_idx = int(0.7 * furthest_idx + 0.3 * centered_idx)

        return best_idx
    #====function used once the maximum gap(start and end indexes)is found for creation of path!====

    #====dispartiy extender....whatever that means....====
    def extend_disparities(self, ranges, threshold=0.5):
        """
        Extend safety regions around sharp disparities in range data.
        Helps with corners and edges of obstacles.

        Args:
            ranges: array of LiDAR ranges
            threshold: minimum disparity to trigger extension (meters)

        Returns:
            extended_ranges: ranges with extended disparities
        """
        extended_ranges = ranges.copy()

        # Find disparities (large jumps in consecutive ranges)
        disparities = np.abs(np.diff(ranges))

        # Find where disparities exceed threshold
        disparity_indices = np.where(disparities > threshold)[0]

        # For each disparity, set nearby points to the closer value
        for idx in disparity_indices:
            # Determine which side is closer
            if ranges[idx] < ranges[idx + 1]:
                # Left side is closer, extend left value to the right
                closer_value = ranges[idx]
                extend_start = idx + 1
                extend_end = min(idx + 10, len(ranges))  # Extend 10 indices
                extended_ranges[extend_start:extend_end] = closer_value
            else:
                # Right side is closer, extend right value to the left
                closer_value = ranges[idx + 1]
                extend_start = max(0, idx - 10)
                extend_end = idx + 1
                extended_ranges[extend_start:extend_end] = closer_value

        return extended_ranges
    #====dispartiy extender....whatever that means....====

    #end of helper functions!
    def lidar_callback(self, data):
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm
        """
        ranges = np.array(data.ranges)

        # Step 1: Preprocess
        proc_ranges = self.preprocess_lidar(ranges)
        # Apply disparity extender
        proc_ranges = self.extend_disparities(proc_ranges)


        # Step 2: Find closest point
        proc_ranges_copy = proc_ranges.copy()
        nonzero_ranges = proc_ranges_copy[proc_ranges_copy > 0]

        if len(nonzero_ranges) == 0:
            self.get_logger().warn('No valid LiDAR ranges!')
            self.publish_drive(0.0, 0.0)
            return

        closest_distance = np.min(nonzero_ranges)
        closest_idx = np.argmin(proc_ranges_copy)

        # Step 3: Eliminate points inside bubble (safety bubble)
        bubble_indices = self.get_bubble_indices(data, closest_idx, closest_distance)
        proc_ranges_copy[bubble_indices] = 0.0

        # Step 4: Find max length gap
        start_i, end_i = self.find_max_gap(proc_ranges_copy)

        # Check if valid gap was found
        if start_i == 0 and end_i == 0:
            self.get_logger().warn('No valid gap found!')
            self.publish_drive(0.0, 0.5)  # Slow down
            return

        # Step 5: Find the best point in the gap
        best_idx = self.find_best_point(start_i, end_i, proc_ranges_copy)

        # Step 6: Calculate steering angle to best point
        # Convert index to angle
        angle_to_best = data.angle_min + best_idx * data.angle_increment

        # Apply steering gain
        steering_angle = self.steering_gain * angle_to_best

        # Clamp steering angle to reasonable limits
        max_steering = np.deg2rad(30.0)  # 30 degrees max
        steering_angle = np.clip(steering_angle, -max_steering, max_steering)

        # Step 7: Calculate speed based on steering angle and gap size
        # More steering = slower speed
        # Larger gap = faster speed
        gap_size = end_i - start_i
        steering_magnitude = abs(steering_angle)

        # Speed calculation
        if steering_magnitude < np.deg2rad(10):
            # Small steering -> high speed
            speed = self.speed_max
        elif steering_magnitude < np.deg2rad(20):
            # Medium steering -> medium speed
            speed = (self.speed_min + self.speed_max) / 2
        else:
            # Large steering -> low speed
            speed = self.speed_min

        # Also consider gap size
        # Smaller gap -> slower speed
        min_safe_gap = 50  # minimum indices for full speed
        if gap_size < min_safe_gap:
            speed = speed * (gap_size / min_safe_gap)
            speed = max(speed, self.speed_min)  # Don't go below minimum

        # Publish drive command
        self.publish_drive(steering_angle, speed)

        # Debug logging
        self.get_logger().info(
            f'Gap: [{start_i}:{end_i}] | Best: {best_idx} | '
            f'Angle: {np.rad2deg(steering_angle):.1f}° | Speed: {speed:.2f} m/s',
            throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    print("Follow the Gap Initialized")
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()


if __name__ == '__main__':
    main()