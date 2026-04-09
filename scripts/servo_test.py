#!/usr/bin/env python3
"""
Servo test script — interactively set steering angle from the terminal.

Usage:
    python3 servo_test.py

The script publishes AckermannDriveStamped to /ackermann_cmd (the topic
ackermann_to_vesc_node subscribes to), bypassing the mux entirely.
Speed is always 0 — only the servo moves.

Enter angles in degrees (e.g. "15", "-20", "0").
Ctrl+C to exit and center the servo.

Servo safe range derived from vesc.yaml:
    servo = -0.6 * angle_rad + 0.5304
    servo_min=0.15  →  angle_max ≈  0.634 rad ( 36.3°)
    servo_max=0.85  →  angle_min ≈ -0.532 rad (-30.5°)
    We cap at ±28° to stay inside with margin.
"""

import sys
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

STEER_MAX_DEG = 28.0        # hard cap — matches STEER_MAX in trajectory.py
PUBLISH_TOPIC = '/ackermann_cmd'


class ServoTester(Node):
    def __init__(self):
        super().__init__('servo_tester')
        self.pub = self.create_publisher(AckermannDriveStamped, PUBLISH_TOPIC, 10)
        # Publish at 20 Hz so the VESC doesn't time out on the topic
        self.current_angle_rad = 0.0
        self.timer = self.create_timer(0.05, self._publish)
        self.get_logger().info(f'Publishing to {PUBLISH_TOPIC} at 20 Hz (speed=0)')
        self.get_logger().info(f'Safe range: ±{STEER_MAX_DEG}°')

    def set_angle_deg(self, deg: float):
        import math
        clamped = max(-STEER_MAX_DEG, min(STEER_MAX_DEG, deg))
        if clamped != deg:
            self.get_logger().warn(
                f'Clamped {deg:.1f}° → {clamped:.1f}° (servo limit)')
        self.current_angle_rad = math.radians(clamped)
        self.get_logger().info(f'Steering set to {clamped:.1f}°')

    def center(self):
        self.current_angle_rad = 0.0
        self._publish()

    def _publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = self.current_angle_rad
        msg.drive.speed = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ServoTester()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f'\nServo Test — enter angle in degrees (±{STEER_MAX_DEG}°), or "q" to quit.')
    print('Examples: "15"  "-20"  "0"\n')

    try:
        while True:
            try:
                raw = input('angle (deg): ').strip()
            except EOFError:
                break

            if raw.lower() in ('q', 'quit', 'exit'):
                break

            if raw == '':
                continue

            try:
                deg = float(raw)
            except ValueError:
                print(f'  Invalid input "{raw}" — enter a number.')
                continue

            node.set_angle_deg(deg)

    except KeyboardInterrupt:
        pass
    finally:
        print('\nCentering servo and shutting down...')
        node.center()
        import time; time.sleep(0.2)   # let the center command publish
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
