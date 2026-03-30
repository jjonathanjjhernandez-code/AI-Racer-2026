import rclpy
from rclpy.node import Node
from my_custom_msgs.msg import VehicleState
import random
import math

class CustomPublisher(Node):
    def __init__(self):
        super().__init__('custom_publisher')

        # Create publisher for custom message type
        self.publisher_ = self.create_publisher(
            VehicleState, 
            'vehicle_state', 
            10)

        # Timer to publish at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_vehicle_state)

        # Simulation variables
        self.simulation_time = 0.0

        self.get_logger().info('Custom Publisher started - Publishing VehicleState messages')

    def publish_vehicle_state(self):
        # Create custom message instance
        msg = VehicleState()

        # Simulate varying vehicle state
        self.simulation_time += 0.5

        # Speed varies between 10-30 m/s with some randomness
        msg.speed = 20.0 + 10.0 * math.sin(self.simulation_time * 0.5) + random.uniform(-1, 1)

        # Steering angle varies between -30 and +30 degrees
        msg.steering_angle = 15.0 * math.sin(self.simulation_time * 0.3) + random.uniform(-2, 2)

        # Throttle varies between 0.3 and 1.0
        msg.throttle = 0.65 + 0.35 * math.sin(self.simulation_time * 0.4)

        # Add timestamp
        msg.timestamp = self.get_clock().now().to_msg()

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published data
        self.get_logger().info(
            f'Publishing VehicleState: '
            f'speed={msg.speed:.2f} m/s, '
            f'steering={msg.steering_angle:.2f}°, '
            f'throttle={msg.throttle:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CustomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()