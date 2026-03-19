import rclpy
from rclpy.node import Node
from my_custom_msgs.msg import VehicleState

class CustomSubscriber(Node):
    def __init__(self):
        super().__init__('custom_subscriber')

        # Create subscription for custom message type
        self.subscription = self.create_subscription(
            VehicleState,
            'vehicle_state',
            self.vehicle_state_callback,
            10)

        self.subscription  # prevent unused variable warning

        self.get_logger().info('Custom Subscriber started - Listening for VehicleState messages')

    def vehicle_state_callback(self, msg):
        # Extract timestamp
        timestamp_sec = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9

        # Analyze the vehicle state
        speed_status = self.analyze_speed(msg.speed)
        steering_status = self.analyze_steering(msg.steering_angle)
        throttle_status = self.analyze_throttle(msg.throttle)

        # Log received data with analysis
        self.get_logger().info(
            f'Received VehicleState:\n'
            f'  Speed: {msg.speed:.2f} m/s ({speed_status})\n'
            f'  Steering: {msg.steering_angle:.2f}° ({steering_status})\n'
            f'  Throttle: {msg.throttle:.2f} ({throttle_status})\n'
            f'  Timestamp: {timestamp_sec:.2f}s'
        )

    def analyze_speed(self, speed):
        """Provide context about speed"""
        if speed < 15:
            return "SLOW"
        elif speed < 25:
            return "MODERATE"
        else:
            return "FAST"

    def analyze_steering(self, angle):
        """Provide context about steering"""
        if abs(angle) < 5:
            return "STRAIGHT"
        elif abs(angle) < 15:
            return "GENTLE TURN"
        else:
            return "SHARP TURN"

    def analyze_throttle(self, throttle):
        """Provide context about throttle"""
        if throttle < 0.4:
            return "LOW"
        elif throttle < 0.7:
            return "MEDIUM"
        else:
            return "HIGH"

def main(args=None):
    rclpy.init(args=args)
    node = CustomSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()