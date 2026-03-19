import rclpy                          # Import the ROS2 Python client library for initializing nodes and handling ROS-related operations
from rclpy.node import Node           # Import the Node class, which serves as the base class for all ROS2 node implementations
from std_msgs.msg import String       # Import the standard ROS2 String message type for sending and receiving text data

class SimpleSubscriber(Node):
    def __init__(self):
        # Initialize this class as a ROS2 node named 'simple_subscriber'
        super().__init__('simple_subscriber')

        # Create a subscription to a topic named 'topic' that publishes String messages.
        # Arguments:
        #   String: The message type the subscriber expects (std_msgs/String)
        #   'topic': The name of the topic to subscribe to
        #   self.listener_callback: The callback function that will be triggered when a new message arrives
        #   10: The queue size (message buffer) if messages arrive faster than they can be processed
        self.subscription = self.create_subscription(
            String,
            'ros2intro',
            self.listener_callback,
            10)

        # This line doesn't do anything functionally; it simply prevents a linting or IDE warning about the subscription
        # variable not being used. It's a common practice in ROS2 Python examples.
        self.subscription

    def listener_callback(self, msg):
        # This callback function is triggered each time a new message is received from the 'topic'.
        # 'msg' is a String message, so we access its data field to get the actual text content.
        # We then use the node's built-in logging system to print the received data.
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    # Initialize the ROS2 client library, necessary before using any ROS2-related code
    rclpy.init(args=args)

    # Create an instance of the SimpleSubscriber node
    node = SimpleSubscriber()

    try:
        # Spin the node, meaning this will block and process any incoming callbacks indefinitely
        # until an external event (like Ctrl+C) interrupts the process.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # If the user presses Ctrl+C in the terminal, a KeyboardInterrupt exception is raised.
        # Here, we simply pass to allow the program to proceed to the 'finally' block for cleanup.
        pass
    finally:
        # Once we are done spinning or have been interrupted, we destroy the node to clean up resources.
        node.destroy_node()
        if rclpy.ok():  # Only shutdown if not already shutting down; Prevent "shutdown already called" on Ctrl-C
           # Shut down the rclpy library, releasing all ROS2-related resources.
           rclpy.shutdown()

# The standard Python entry point for executable scripts.
# If this script is run directly (e.g., 'python3 subscriber.py'), execute the main() function.
if __name__ == '__main__':
    main()