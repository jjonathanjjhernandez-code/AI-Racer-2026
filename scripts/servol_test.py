#!/usr/bin/env python3
# =============================================================
# motor_servo_test.py
# Sends a simple speed + steering command to test BLDC and servo
# Run AFTER motor_test_launch.py is running
#
# Usage:
#   python3 ~/scripts/motor_servo_test.py
# =============================================================
 
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import time
 
class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            'ackermann_cmd_teleop',  # goes through mux → ackermann_drive
            10
        )
 
    def send(self, speed: float, steering: float, duration: float):
        """
        speed    : m/s  (positive = forward)
        steering : rad  (positive = left, negative = right)
        duration : seconds to hold the command
        """
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
 
        self.get_logger().info(
            f'Sending: speed={speed:.2f} m/s  steering={steering:.3f} rad  for {duration}s'
        )
 
        end = time.time() + duration
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            time.sleep(0.05)  # 20 Hz
 
    def stop(self):
        self.send(0.0, 0.0, 0.5)
        self.get_logger().info('Stopped.')
 
 
def main():
    rclpy.init()
    node = MotorTest()
 
    try:
        print("\n=== TEST 1: Servo center (no motion) ===")
        node.send(speed=0.0, steering=0.0, duration=2.0)
 
        print("\n=== TEST 2: Steer LEFT ===")
        node.send(speed=0.0, steering=0.2, duration=2.0)   # ~11.5 degrees
 
        print("\n=== TEST 3: Steer RIGHT ===")
        node.send(speed=0.0, steering=-0.2, duration=2.0)
 
        print("\n=== TEST 4: Center servo ===")
        node.send(speed=0.0, steering=0.0, duration=1.0)
 
        print("\n=== TEST 5: SLOW forward (wheels off ground first!) ===")
        input("  >>> Press ENTER to spin motor forward (make sure wheels are off ground) <<<")
        node.send(speed=0.5, steering=0.0, duration=2.0)   # 0.5 m/s
 
        print("\n=== TEST 6: Stop ===")
        node.stop()
 
    except KeyboardInterrupt:
        print("\nInterrupted — stopping motor.")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()