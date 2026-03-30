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
        self.declare_parameter('speed_max', 100.0)  # maximum speed (m/s)
        self.declare_parameter('steering_gain', 1.0)  # steering proportional gain
        
        # Get parameters
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preprocess_conv_size = self.get_parameter('preprocess_conv_size').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_max = 100# self.get_parameter('speed_max').value
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

    def preprocess_lidar(self, ranges):
        """ 
        Preprocess the LiDAR scan array.
        1. Setting each value to the mean over some window
        2. Rejecting high values (> max_lidar_range)
        
        Args:
            ranges: array of LiDAR ranges
            
        Returns:
            proc_ranges: processed array of ranges
        """
        # TODO: Implement preprocessing
        proc_ranges = np.array(ranges)
        return proc_ranges
    
    def find_max_gap(self, free_space_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges
        
        Args:
            free_space_ranges: array with 0s for obstacles and non-zero for free space
            
        Returns:
            start_idx: start index of max gap
            end_idx: end index of max gap
        """
        # TODO: Implement max gap finding
        return 0, 0
    
    def find_best_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indices of max-gap range
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        
        Args:
            start_i: start index of gap
            end_i: end index of gap
            ranges: array of LiDAR ranges
            
        Returns:
            best_idx: index of best point in gap
        """
        # TODO: Implement best point selection
        return 0
    
    def lidar_callback(self, data):
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm & 
        publish an AckermannDriveStamped Message
        
        Args:
            data: LaserScan message
        """
        ranges = np.array(data.ranges)
        
        # Step 1: Preprocess
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO: Find closest point
        
        # TODO: Eliminate points inside bubble
        
        # TODO: Find max length gap
        
        # TODO: Find the best point in the gap
        
        # TODO: Publish Drive message


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

