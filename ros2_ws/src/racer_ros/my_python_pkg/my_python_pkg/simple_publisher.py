import rclpy                          # Import ROS2 Python client library
from rclpy.node import Node           # Import Node class to create a ROS2 node
from std_msgs.msg import String       # Import standard String message

class SimplePublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'simple_publisher'
        super().__init__('simple_publisher')

        # Create a publisher of message type: String; topic name: 'topic'
        # And queue size: 10 (buffer size for outgoing messages)
        self.publisher_ = self.create_publisher(String, 'ros2intro', 10)

        # Create a timer that triggers 'timer_callback' every 1 second (1Hz)
        # The timer allows us to publish messages at a fixed interval.
        self.timer = self.create_timer(1.0, self.publish_message)  # 1 second interval

        # Initialize a counter to keep track of how many messages we've sent
        self.count = 0

    def publish_message(self):
        # Called every 1.0 second due to the timer.

        # Create a new String message instance
        msg = String()

        # Set its data field to a friendly message including the counter
        msg.data = f'Hello ROS 2: {self.count}'

        # Publish the message to the topic
        self.publisher_.publish(msg)

        # Log output to the console so we see what is being published
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter for the next message
        self.count += 1

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of our publisher node
    node = SimplePublisher()

    # Keep the node running until it is killed, allowing callbacks (like the timer) to function
    rclpy.spin(node)

    # Once we exit spin (e.g., by pressing Ctrl+C), destroy the node
    node.destroy_node()

    # Shutdown ROS2 gracefully
    rclpy.shutdown()

if __name__ == '__main__':
    main()