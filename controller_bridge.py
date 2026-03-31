#!/usr/bin/env python3
"""
Controller bridge: reads PowerA Xbox controller via USB (PyUSB)
and publishes AckermannDriveStamped to /drive.

Left stick Y  -> speed (throttle)
Right stick X -> steering angle

Run on the Jetson HOST (not inside Docker), then the /drive topic
will be available to the Docker container via ROS2 network.

OR copy into Docker container and run there (USB device is passed through via /dev mount).

Usage:
    sudo python3 controller_bridge.py
"""

import usb.core
import usb.util
import struct
import math
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

# Controller USB IDs
VENDOR_ID  = 0x20d6
PRODUCT_ID = 0x2002

# Tuning — adjust to taste
MAX_SPEED         = 2.0   # m/s at full stick
MAX_STEERING      = 0.4   # radians at full stick
DEADZONE          = 0.05  # ignore stick noise below this fraction
INIT_PACKET       = bytes([0x05, 0x20, 0x00, 0x01, 0x00])

def normalize(raw: int, deadzone: float = DEADZONE) -> float:
    """Convert signed 16-bit raw value to [-1.0, 1.0] with deadzone."""
    val = raw / 32767.0
    val = max(-1.0, min(1.0, val))
    if abs(val) < deadzone:
        return 0.0
    # Rescale so deadzone edge = 0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - deadzone) / (1.0 - deadzone)


class ControllerBridge(Node):
    def __init__(self, dev):
        super().__init__('controller_bridge')
        self.dev = dev
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.02, self.read_and_publish)  # 50 Hz
        self.get_logger().info('Controller bridge started. Publishing to /drive')

    def read_and_publish(self):
        try:
            data = self.dev.read(0x81, 64, timeout=20)
        except usb.core.USBError as e:
            if 'timed out' not in str(e):
                self.get_logger().warn(f'USB error: {e}')
            return

        if len(data) < 18:
            return  # short packet, skip

        # Parse axes as signed 16-bit little-endian
        lx = struct.unpack_from('<h', bytes(data), 10)[0]  # left stick X
        ly = struct.unpack_from('<h', bytes(data), 12)[0]  # left stick Y
        rx = struct.unpack_from('<h', bytes(data), 14)[0]  # right stick X
        ry = struct.unpack_from('<h', bytes(data), 16)[0]  # right stick Y

        # Normalize
        throttle  = -normalize(ly)   # push up = positive speed (Y axis inverted)
        steering  = -normalize(rx)   # push right = turn right (negate if reversed)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed            = throttle * MAX_SPEED
        msg.drive.steering_angle   = steering * MAX_STEERING
        msg.drive.acceleration     = 0.0
        msg.drive.jerk             = 0.0
        msg.drive.steering_angle_velocity = 0.0

        self.pub.publish(msg)


def main():
    # Find controller
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if dev is None:
        print('ERROR: Controller not found! Is it plugged in?')
        return

    print(f'Found controller: {dev.product}')

    # Detach kernel driver if needed
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)

    dev.set_configuration()

    # Send Xbox init packet
    try:
        dev.write(0x01, INIT_PACKET)
        print('Init packet sent.')
    except Exception as e:
        print(f'Warning: could not send init packet: {e}')

    time.sleep(0.1)

    # Start ROS2
    rclpy.init()
    node = ControllerBridge(dev)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
