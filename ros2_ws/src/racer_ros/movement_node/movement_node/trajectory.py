#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class WallFollowReactive(Node):
    """
    Combined Wall Following + Follow the Gap node.

    Primary behavior: PID wall following (tracks a desired distance from the right wall).
    Reactive layer: Follow the Gap safety features activate when obstacles are detected
    within a danger threshold, overriding wall-follow commands to avoid collisions.
    """

    def __init__(self):
        super().__init__('movement_node')

        # -==- Wall Follow Params -==-
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('desired_distance', 1.0)         # (meters) from wall
        self.declare_parameter('lookahead_distance', 1.0)       # (meters)
        self.declare_parameter('theta_deg', 50.0)               # angle for beam 'a'
        self.declare_parameter('max_speed', 20.0)

        # -==- Follow the Gap Params -==-
        self.declare_parameter('bubble_radius', 0.3)            # (meters)
        self.declare_parameter('preprocess_conv_size', 3)       # smoothing window
        self.declare_parameter('max_lidar_range', 3.0)          # clip range
        self.declare_parameter('speed_min', 0.5)                # min reactive speed
        self.declare_parameter('speed_max', 2.0)                # max reactive speed
        self.declare_parameter('steering_gain', 1.0)            # reactive steering gain
        self.declare_parameter('disparity_threshold', 0.5)      # disparity extender threshold

        # -==- Blending Params -==-
        self.declare_parameter('danger_threshold', 1.5)       # distance to trigger reactive mode
        self.declare_parameter('blend_range', 0.5)            # smooth transition zone width
        self.declare_parameter('wall_blend_alpha', 0.25)
        self.declare_parameter('min_wall_range', 0.15)
        self.declare_parameter('max_wall_range', 4.75)

        # -==- wall blending params -==-
        
        # -==- Load All Params -==-
        self._load_parameters()

        # -==- PID state -==-
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.current_speed = 0.0

        # -==- Pubs and Subs -==-
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # -==- Debugging pubs -==-
        self.error_pub = self.create_publisher(Float64, '/wall_follow/error', 10)
        self.dist_pub = self.create_publisher(Float64, '/wall_follow/wall_distance', 10)
        self.steering_pub = self.create_publisher(Float64, '/wall_follow/steering_angle', 10)
        self.mode_pub = self.create_publisher(Float64, '/wall_follow/reactive_weight', 10)
        self.left_dist_pub = self.create_publisher(Float64, '/wall_follow/left_wall_distance', 10)
        self.right_dist_pub = self.create_publisher(Float64, '/wall_follow/right_wall_distance', 10)
        self.corridor_pub = self.create_publisher(Float64, '/wall_follow/corridor_width', 10)

        self.get_logger().info('=' * 55)
        self.get_logger().info('Wall Follow + Reactive Gap Node Initialized')
        self.get_logger().info(f'PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
        self.get_logger().info(f'Desired Wall Distance: {self.desired_distance}m')
        self.get_logger().info(f'Danger Threshold: {self.danger_threshold}m')
        self.get_logger().info(f'Bubble Radius: {self.bubble_radius}m')
        self.get_logger().info('=' * 55)

    # -=====================- Saving the params as values -=====================-
    def _load_parameters(self):
        """Read all parameters from the parameter server."""
        # Wall follow
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.desired_distance = self.get_parameter('desired_distance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.theta_deg = self.get_parameter('theta_deg').value
        self.theta = np.deg2rad(self.theta_deg)
        self.max_speed = self.get_parameter('max_speed').value

        # Follow the gap
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preprocess_conv_size = self.get_parameter('preprocess_conv_size').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_max = self.get_parameter('speed_max').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.disparity_threshold = self.get_parameter('disparity_threshold').value

        # Blending
        self.danger_threshold = self.get_parameter('danger_threshold').value
        self.blend_range = self.get_parameter('blend_range').value
        self.wall_blend_alpha = self.get_parameter('wall_blend_alpha').value
        self.min_wall_range = self.get_parameter('min_wall_range').value
        self.max_wall_range = self.get_parameter('max_wall_range').value

    # -=====================- Wall Follow helpers -=====================-
    def get_range(self, range_data, angle, window=4):
        """
        LIDAR STUFF (finding the window of best info)
        """
        angle_index = int((angle - range_data.angle_min) / range_data.angle_increment)  # range of angle indexes from (current - min) / how much it can increment
        angle_index = np.clip(angle_index, 0, len(range_data.ranges) - 1)               # cap the angle

        low = max(0, angle_index - window)                                              # 
        high = min(len(range_data.ranges), angle_index + window + 1)

        valid = [r for r in np.array(range_data.ranges)[low:high]                       # clip based on ranges between 
                 if not np.isnan(r) and not np.isinf(r)]
        return np.mean(valid) if valid else range_data.range_max

    def get_wall_dist(self, range_data, side='right'):
        if side == 'right':
            angle_b = -np.pi / 2            # go right
            sign = 1.0
        else:
            angle_b = np.pi / 2             # go left
            sign = -1.0
        
        angle_a = angle_b + sign * self.theta
        
        b = self.get_range(range_data, angle_b, window=4)
        a = self.get_range(range_data, angle_a, window=4)
        
        if b > self.max_wall_range or b < self.min_wall_range:
            return None, None
        if a > self.max_wall_range or a < self.min_wall_range:
            return None, None
        
        numerator = a * np.cos(self.theta) - b
        denominator = a * np.sin(self.theta)
        alpha = np.arctan2(numerator, denominator) if abs(denominator) > 1e-6 else 0.0

        D_t = b * np.cos(alpha)                 # how far you are right now
        D_t1 = D_t + self.lookahead_distance * np.sin(alpha)    # lookahead distance
        
        return D_t1, D_t

    def get_wall_error(self, range_data, dist):
        """
        HOW it measures the wall
        """
        right_proj, right_Dt = self.get_wall_dist(range_data, side='right')
        left_proj, left_Dt = self.get_wall_dist(range_data, side='left')
        
        right_visible = right_proj is not None
        left_visible = left_proj is not None
        
        if right_visible and left_visible:
            # error from right wall
            right_error = self.desired_distance - right_proj
            
            # centering error: pos -> too close to right wall | neg -> too close to left wall
            centering_error = (right_Dt - left_Dt) / 2.0
            
            # blending: alpha=0 -> right only | alpha=1 -> pure centering
            blended_error = (1.0 - self.wall_blend_alpha) * right_error + self.wall_blend_alpha * centering_error
            
            wall_dist_for_debug = right_Dt
            self.get_logger().debug(
                f'Dual wall: R={right_Dt:.2f}m L={left_Dt:.2f}m '
                f'corridor={right_Dt+left_Dt:.2f}m '
                f'err={blended_error:.3f}',
                throttle_duration_sec=0.5
            )
        elif right_visible:
            blended_error = self.desired_distance - right_proj
            wall_dist_for_debug = right_Dt
            self.get_logger().debug('Right wall only', throttle_duration_sec=1.0)
        
        elif left_visible:
            # We don't have desired_distance from left directly, but we can still
            # use a centering heuristic: push toward the center of whatever we see.
            # Here we assume a typical corridor width and estimate right-wall distance.
            typical_corridor = 2.0 * self.desired_distance   # rough estimate
            estimated_right = typical_corridor - left_proj
            blended_error = self.desired_distance - estimated_right
            wall_dist_for_debug = estimated_right
            self.get_logger().debug('Left wall only (estimated)', throttle_duration_sec=1.0)
        
        else:
            # No walls detected -> hold steady
            blended_error = 0.0
            wall_dist_for_debug = self.desired_distance
            self.get_logger().warn('No wall detected!', throttle_duration_sec=1.0)
        
        return blended_error, wall_dist_for_debug, right_Dt, left_Dt

    def wall_follow_control(self, error):
        """
        PID controller for wall following.
        Returns (steering_angle, speed).
        """
        current_time = self.get_clock().now()

        if self.prev_time is None:
            dt = 1e-6
            self.prev_time = current_time
        else:
            dt = max((current_time - self.prev_time).nanoseconds / 1e9, 1e-6)
            self.prev_time = current_time

        P = self.kp * error                             # brings back to desired distance
        self.integral = np.clip(self.integral + error * dt, -10.0, 10.0)
        I = self.ki * self.integral                     # fix steady drift
        D = self.kd * (error - self.prev_error) / dt    # if error grows fast -> push back hard
        self.prev_error = error

        angle = np.clip(P + I + D, -np.deg2rad(30.0), np.deg2rad(30.0))
        speed = self.max_speed * np.clip(np.cos(angle) ** 3, 0.3, 1.0)  # the more you turn the slower you go (full speed -> striaght) | (30 deg turn -> 30% speed)

        # Adaptive steering limit at high speed
        adaptive_max = np.deg2rad(30.0 - 10.0 * (speed / self.max_speed))
        angle = np.clip(angle, -adaptive_max, adaptive_max)

        return angle, speed

    # -=====================- Follow the Gap helpers  -=====================-
    def preprocess_lidar(self, ranges):
        
        """
        Cleans the lidar
        """
        proc = np.nan_to_num(np.array(ranges),
                             nan=self.max_lidar_range,
                             posinf=self.max_lidar_range,
                             neginf=0.0)
        proc = np.clip(proc, 0, self.max_lidar_range)
        if self.preprocess_conv_size > 1:
            kernel = np.ones(self.preprocess_conv_size) / self.preprocess_conv_size
            proc = np.convolve(proc, kernel, mode='same')
        return proc

    def extend_disparities(self, ranges):
        """
        Extend closer values outwards to make smoothend wall
        
        [3.0, 3.0, 0.5, 3.0, 3.0] -> [3.0, 0.5, 0.5, 0.5, 3.0]
        """
        extended = ranges.copy()
        disparities = np.abs(np.diff(ranges))
        for idx in np.where(disparities > self.disparity_threshold)[0]:
            if ranges[idx] < ranges[idx + 1]:
                s, e = idx + 1, min(idx + 10, len(ranges))
                extended[s:e] = ranges[idx]
            else:
                s, e = max(0, idx - 10), idx + 1
                extended[s:e] = ranges[idx + 1]
        return extended

    def get_bubble_indices(self, data, closest_idx, closest_distance):
        """
        Zero all readings close to car so car can't messup and crash into it
        
        [3.0, 0.5, 0.5, 0.5, 3.0] -> [3.0, 0.0, 0.0, 0.0, 3.0]
        """
        dynamic_radius = self.bubble_radius * (1.0 + self.current_speed / self.speed_max)
        safe_dist = max(closest_distance, 0.1)
        bubble_angle = dynamic_radius / safe_dist
        bubble_idx_range = int(np.ceil(bubble_angle / data.angle_increment))
        start = max(0, closest_idx - bubble_idx_range)
        end = min(len(data.ranges) - 1, closest_idx + bubble_idx_range)
        return np.arange(start, end + 1)


    def find_max_gap(self, free_space_ranges):
        """Return (start, end) indices of the longest contiguous non-zero gap."""
        mask = free_space_ranges > 0
        diff = np.diff(np.concatenate(([0], mask.astype(int), [0])))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        if len(starts) == 0:
            return 0, 0
        max_i = np.argmax(ends - starts)
        return starts[max_i], ends[max_i] - 1

    def find_best_point(self, start_i, end_i, ranges):
        """Select the best steering target within a gap."""
        if start_i >= end_i or end_i >= len(ranges):
            return len(ranges) // 2
        gap = ranges[start_i:end_i + 1]
        width = end_i - start_i
        avg_depth = np.mean(gap)
        if width > 100 and avg_depth > 2.0:
            return start_i + np.argmax(gap)
        elif width < 50:
            return (start_i + end_i) // 2
        else:
            furthest = start_i + np.argmax(gap)
            centered = (start_i + end_i) // 2
            return int(0.7 * furthest + 0.3 * centered)

    def reactive_control(self, data):
        """
        Full Follow the Gap pipeline.
        Returns (steering_angle, speed) or None if no valid gap is found.
        """
        
        # process the lidar
        proc = self.preprocess_lidar(data.ranges)
        proc = self.extend_disparities(proc)

        # ADD THIS: mask out beams beyond ±90° to prevent steering into side walls
        num_beams = len(proc)
        angles = data.angle_min + np.arange(num_beams) * data.angle_increment
        side_mask = np.abs(angles) > np.pi / 2
        proc[side_mask] = 0.0   # treat extreme side beams as blocked


        # clean the values 
        nonzero = proc[proc > 0]
        if len(nonzero) == 0:
            return 0.0, 0.0


        closest_dist = np.min(nonzero)
        closest_idx = np.argmin(proc)
        
        # create a bubble 
        bubble_idx = self.get_bubble_indices(data, closest_idx, closest_dist)
        proc[bubble_idx] = 0.0

        # find the biggest gap
        start_i, end_i = self.find_max_gap(proc)
        if start_i == 0 and end_i == 0:
            return 0.0, self.speed_min

        # steer there
        best_idx = self.find_best_point(start_i, end_i, proc)
        angle = self.steering_gain * (data.angle_min + best_idx * data.angle_increment)
        
        max_steer = np.deg2rad(30.0)
        angle = np.clip(angle, -max_steer, max_steer)

        steer_mag = abs(angle)
        if steer_mag < np.deg2rad(5):           # straight -> go fast 
            speed = self.speed_max             
        elif steer_mag < np.deg2rad(15):        # genetler turn
            speed = self.speed_min + 0.7 * (self.speed_max - self.speed_min)
        elif steer_mag < np.deg2rad(25):        # tighter turn
            speed = self.speed_min + 0.3 * (self.speed_max - self.speed_min)
        else:                                   # sharpest turn
            speed = self.speed_min

        gap_size = end_i - start_i
        min_safe_gap = 50
        if gap_size < min_safe_gap:
            speed = max(speed * (gap_size / min_safe_gap), self.speed_min)

        return angle, speed

    # -=====================- The Blend -=====================-
    def scan_callback(self, msg):
        # Hot-reload tunable parameters every scan
        self._load_parameters()

        # Dual Wall follow
        wf_error, wall_dist, right_Dt, left_Dt = self.get_wall_error(msg, self.desired_distance)
        wf_angle, wf_speed = self.wall_follow_control(wf_error)

        # Closest obstacle distance
        ranges = np.array(msg.ranges)
        valid = ranges[np.isfinite(ranges) & (ranges > 0)]
        closest = np.min(valid) if len(valid) > 0 else self.max_lidar_range

        # Determine reactive weight
        lower = self.danger_threshold - self.blend_range # 0.5m
        if closest >= self.danger_threshold: # >= 0.8m
            reactive_weight = 0.0           # safe -> wall follow only
        elif closest <= lower:               # <= 0.5m
            reactive_weight = 1.0           # danger -> full reactive
        else:
            # Smooth linear blend in the transition zone
            reactive_weight = 1.0 - (closest - lower) / self.blend_range

        # follow the gap
        if reactive_weight > 0.0:
            rg_angle, rg_speed = self.reactive_control(msg)
        else:
            rg_angle, rg_speed = wf_angle, wf_speed   # ======= unused, just placeholders ========

        # Blend     
        # closest=1.0m  ->  [WF=100%  reactive=0%  ]  fast, tracking wall
        # closest=0.65m ->  [WF=50%   reactive=50% ]  blending
        # closest=0.4m  ->  [WF=0%    reactive=100%]  full gap-follow
        final_angle = (1.0 - reactive_weight) * wf_angle + reactive_weight * rg_angle
        final_speed = (1.0 - reactive_weight) * wf_speed + reactive_weight * rg_speed

        # Clamp final steering
        final_angle = np.clip(final_angle, -np.deg2rad(30.0), np.deg2rad(30.0))

        # Track speed for dynamic bubble sizing
        self.current_speed = final_speed

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(final_angle)
        drive_msg.drive.speed = float(final_speed)
        self.drive_pub.publish(drive_msg)

        # Publish debug topics
        err_msg = Float64()
        err_msg.data = wf_error
        self.error_pub.publish(err_msg)

        dist_msg = Float64()
        dist_msg.data = wall_dist
        self.dist_pub.publish(dist_msg)

        steer_msg = Float64()
        steer_msg.data = float(np.rad2deg(final_angle))
        self.steering_pub.publish(steer_msg)

        mode_msg = Float64()
        mode_msg.data = reactive_weight
        self.mode_pub.publish(mode_msg)
        
        
        if right_Dt is not None:
            r_msg = Float64()
            r_msg.data = float(right_Dt)
            self.right_dist_pub.publish(r_msg)
        if left_Dt is not None:
            l_msg = Float64()
            l_msg.data = float(left_Dt)
            self.left_dist_pub.publish(l_msg)
            
        if right_Dt is not None and left_Dt is not None:
            c_msg = Float64()
            c_msg.data = float(right_Dt + left_Dt)
            self.corridor_pub.publish(c_msg)
            

        mode_str = ('WALL_FOLLOW' if reactive_weight < 0.01
                    else 'REACTIVE' if reactive_weight > 0.99
                    else f'BLEND({reactive_weight:.0%})')
        
        alpha_str = f'alpha={self.wall_blend_alpha:.1f}'
        right_str = f'right={right_Dt:.2f}m' if right_Dt else 'right=-'
        left_str = f'left={left_Dt:.2f}m' if left_Dt else 'left=-'
        
        self.get_logger().info(
            f'[{mode_str}][{alpha_str}] {right_str} {left_str} | '
            f'Closest: {closest:.2f}m | '
            f'Angle: {np.rad2deg(final_angle):.1f}° | '
            f'Speed: {final_speed:.2f} m/s',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    print("Wall Follow + Reactive Gap Initialized")
    node = WallFollowReactive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()