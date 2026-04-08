from launch import LaunchDescription
from launch_ros.actions import Node
 
# =============================================================
# motor_test_launch.py
# Launches VESC driver + Ackermann converter without depending
# on f1tenth_stack. Config is inlined here for easy tuning.
# =============================================================
 
# -------------------------------------------------------
# VESC Parameters — tune these for your hardware
# D3542 1450KV motor, 14-pole (7 pole pairs)
# Traxxas 2056 servo
# -------------------------------------------------------
VESC_PARAMS = {
    # Serial port — confirm with: ls /dev/ttyACM*
    'port': '/dev/ttyACM0',
 
    # --- BLDC / Speed ---
    # ERPM = KV x pole_pairs x (voltage factor)
    # D3542 1450KV, 7 pole pairs → 1450 * 7 = 10150
    'speed_to_erpm_gain': 10150.0,
    'speed_to_erpm_offset': 0.0,
 
    # Max ERPM limits (safety — start conservative)
    'max_erpm_speed_limit': 15000.0,  # ~1.5 m/s at start; increase after testing
 
    # --- Steering / Servo ---
    # Traxxas 2056: 0.0–1.0 maps to 1000–2000µs PWM
    # Neutral = 0.5, tune gain after measuring actual steering angle
    'steering_angle_to_servo_gain': -0.9549,   # flip sign if steering is inverted
    'steering_angle_to_servo_offset': 0.5,
    'servo_min': 0.15,   # physical hard stop protection
    'servo_max': 0.85,
 
    # --- Odometry ---
    'wheelbase': 0.33,          # meters — measure your car
    'tachometer_ticks_to_meters': 0.000085,  # tune after wheel calibration
}
 
MUX_PARAMS = {
    'ackermann_mux': {
        'ros__parameters': {
            'inputs': {
                'teleop': {
                    'topic': 'ackermann_cmd_teleop',
                    'timeout': 0.5,
                    'priority': 1,
                },
                'autonomous': {
                    'topic': 'ackermann_cmd_auto',
                    'timeout': 0.5,
                    'priority': 10,
                },
            }
        }
    }
}
 
 
def generate_launch_description():
    return LaunchDescription([
 
        # VESC serial driver — talks to hardware over /dev/ttyACM0
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[VESC_PARAMS],
            output='screen',
        ),
 
        # Converts AckermannDriveStamped → VESC motor + servo commands
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[VESC_PARAMS],
<<<<<<< HEAD
	    remappings=[('ackermann_cmd', 'ackermann_drive')],
=======
            remappings=[('ackermann_cmd', 'ackermann_cmd_teleop')],
>>>>>>> 4469169ccf90dbc261749f31165b742d46f4c582
            output='screen',
        ),
 
        # Publishes /odom from VESC encoder feedback
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[VESC_PARAMS],
            output='screen',
        ),
 
        # Mux — merges teleop + autonomous command topics
        # Output → /ackermann_drive
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[MUX_PARAMS],
            remappings=[('ackermann_cmd_out', 'ackermann_drive')],
            output='screen',
        ),
    ])
