#!/usr/bin/env python3
# Optimized exploration node for TurtleBot3 Waffle - Focus on Speed

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np # Using numpy for potential nan handling if needed

class FastExplorerNode(Node):

    def __init__(self):
        super().__init__("fast_explorer_node")

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan, "scan", self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data # More appropriate QoS for sensor data
        )

        # Timer for 90-second runtime
        self.start_time = self.get_clock().now()
        self.runtime_limit = 90.0  # seconds
        self.timer = self.create_timer(0.1, self.check_runtime)
        self.is_stopped = False

        # Velocity message
        self.twist = Twist()

        # --- Tunable Parameters ---
        # Distances (meters)
        self.critical_front_distance = 0.40   # Dangerously close in front -> Stop/Sharp Turn
        self.warning_front_distance = 0.70    # Getting close in front -> Slow down / Gentle Turn
        self.side_avoid_distance = 0.45       # Too close to side -> Gentle Nudge Away

        # Speeds
        self.max_linear_speed = 0.28          # m/s - Max forward speed
        self.cautious_linear_speed = 0.18     # m/s - Speed when obstacle is warned
        self.max_angular_speed = 1.9          # rad/s - Max turning speed (for avoidance)
        self.gentle_turn_speed = 0.8          # rad/s - Speed for gentle turns/nudges

        # LiDAR Sector Angles (degrees) - Adjust based on robot FOV and needs
        self.front_angle = 15      # +/- degrees from 0 for front sector
        self.front_side_angle = 45 # +/- degrees from 0 for wider front check
        self.side_angle_start = 50 # degrees - where side sector starts
        self.side_angle_end = 130  # degrees - where side sector ends

        self.get_logger().info(f"'{self.get_name()}' node initialized.")
        self.get_logger().info(f"Params: CritDist={self.critical_front_distance}, WarnDist={self.warning_front_distance}, SideDist={self.side_avoid_distance}")
        self.get_logger().info(f"Params: MaxLin={self.max_linear_speed}, MaxAng={self.max_angular_speed}")


    def check_runtime(self):
        """Check if runtime limit exceeded and stop the robot."""
        if self.is_stopped:
            return
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time >= self.runtime_limit:
            self.get_logger().info(f"{self.runtime_limit} seconds elapsed. Stopping exploration.")
            self.stop_robot()
            self.is_stopped = True
            self.timer.cancel()

    def stop_robot(self):
        """Sends a zero velocity command to stop the robot."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Robot stopped.")

    def get_min_dist_in_sector(self, ranges, angle_min_rad, angle_max_rad, angle_increment, range_min, range_max):
        """ Calculates the minimum valid distance within a given angular sector. """
        start_index = max(0, int(math.floor((angle_min_rad - (-math.pi)) / angle_increment)))
        end_index = min(len(ranges) - 1, int(math.ceil((angle_max_rad - (-math.pi)) / angle_increment)))

        if start_index > end_index: # Handle wrap-around cases if necessary, though angles should be within [-pi, pi]
             # This basic implementation assumes angles are handled correctly to avoid wrap-around issues here
             # For a simple +/- angle approach from 0, separate calls might be easier
             return range_max

        sector = ranges[start_index : end_index + 1]

        # Filter out invalid ranges (0, inf, nan) and those outside physical limits
        valid_ranges = [r for r in sector if range_min < r < range_max and np.isfinite(r)]

        if not valid_ranges:
            return range_max  # Return max range if no valid readings
        return min(valid_ranges)

    def get_sector_distances(self, msg: LaserScan):
        """ Get minimum distances in key sectors using angles """
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)
        range_min = msg.range_min + 0.02 # Add buffer
        range_max = msg.range_max

        # Define angles in radians
        front_rad = math.radians(self.front_angle)
        front_side_rad = math.radians(self.front_side_angle)
        side_start_rad = math.radians(self.side_angle_start)
        side_end_rad = math.radians(self.side_angle_end)

        # Calculate indices - careful with wrap-around 0 degrees / index 0
        # Center index is roughly num_ranges / 2 if angle_min is -pi
        # Assuming angle_min = -pi, angle_max = pi for standard lidar
        
        # Front (narrow): -front_rad to +front_rad
        idx_center = num_ranges // 2
        idx_front_delta = int(front_rad / angle_increment)
        front_indices = list(range(0, idx_front_delta + 1)) + \
                        list(range(num_ranges - idx_front_delta, num_ranges))
        front_ranges = [ranges[i] for i in front_indices]
        dist_f = min([r for r in front_ranges if range_min < r < range_max and np.isfinite(r)], default=range_max)


        # Front-Left (wider): +front_rad to +front_side_rad
        idx_fl_start = idx_front_delta + 1 # Start after narrow front
        idx_fl_end = int(front_side_rad / angle_increment)
        dist_fl = min([ranges[i] for i in range(idx_fl_start, idx_fl_end + 1) if range_min < ranges[i] < range_max and np.isfinite(ranges[i])], default=range_max)

        # Front-Right (wider): -front_side_rad to -front_rad
        idx_fr_start = num_ranges - int(front_side_rad / angle_increment)
        idx_fr_end = num_ranges - idx_front_delta - 1 # End before narrow front
        dist_fr = min([ranges[i] for i in range(idx_fr_start, idx_fr_end + 1) if range_min < ranges[i] < range_max and np.isfinite(ranges[i])], default=range_max)


        # Left Side: +side_start_rad to +side_end_rad
        idx_l_start = int(side_start_rad / angle_increment)
        idx_l_end = int(side_end_rad / angle_increment)
        dist_l = min([ranges[i] for i in range(idx_l_start, idx_l_end + 1) if range_min < ranges[i] < range_max and np.isfinite(ranges[i])], default=range_max)

        # Right Side: -side_end_rad to -side_start_rad
        idx_r_start = num_ranges - int(side_end_rad / angle_increment)
        idx_r_end = num_ranges - int(side_start_rad / angle_increment)
        dist_r = min([ranges[i] for i in range(idx_r_start, idx_r_end + 1) if range_min < ranges[i] < range_max and np.isfinite(ranges[i])], default=range_max)


        # self.get_logger().debug(f"Distances: F={dist_f:.2f}, FL={dist_fl:.2f}, FR={dist_fr:.2f}, L={dist_l:.2f}, R={dist_r:.2f}")
        return dist_f, dist_fl, dist_fr, dist_l, dist_r


    def lidar_callback(self, msg: LaserScan):
        """ Main logic loop """
        if self.is_stopped:
            return

        dist_f, dist_fl, dist_fr, dist_l, dist_r = self.get_sector_distances(msg)

        target_linear_x = self.max_linear_speed
        target_angular_z = 0.0

        # --- Decision Logic ---
        if dist_f < self.critical_front_distance:
            # 1. Critical Obstacle Directly Ahead: Stop linear, sharp turn
            self.get_logger().warn(f"CRITICAL front obstacle: {dist_f:.2f}m. Turning.")
            target_linear_x = 0.0
            # Turn towards side with more space in the wider front sectors
            if dist_fl > dist_fr:
                target_angular_z = self.max_angular_speed # Turn left
            else:
                target_angular_z = -self.max_angular_speed # Turn right

        elif dist_f < self.warning_front_distance:
            # 2. Warning Obstacle Ahead: Slow down, gentle turn
            self.get_logger().info(f"Warning front obstacle: {dist_f:.2f}m. Slowing/Turning.")
            target_linear_x = self.cautious_linear_speed
            # Turn gently towards side with more space
            if dist_fl > dist_fr:
                 target_angular_z = self.gentle_turn_speed # Gentle left
            else:
                 target_angular_z = -self.gentle_turn_speed # Gentle right

        else:
            # 3. Path Ahead Clear: Check sides for nudging
            target_linear_x = self.max_linear_speed # Maintain full speed
            side_nudge = 0.0
            if dist_l < self.side_avoid_distance:
                # Too close to left wall, nudge right
                # Nudge strength proportional to how close it is
                error = self.side_avoid_distance - dist_l
                side_nudge = -self.gentle_turn_speed * (error / self.side_avoid_distance) * 1.5 # Stronger nudge factor
                self.get_logger().debug(f"Nudging right from left wall: {dist_l:.2f}m")
            elif dist_r < self.side_avoid_distance:
                # Too close to right wall, nudge left
                error = self.side_avoid_distance - dist_r
                side_nudge = self.gentle_turn_speed * (error / self.side_avoid_distance) * 1.5
                self.get_logger().debug(f"Nudging left from right wall: {dist_r:.2f}m")
            
            # Apply nudge only if not turning for front obstacle already
            # (Avoid conflicting commands - front obstacle takes priority)
            # This condition is implicitly handled by the if/elif/else structure
            target_angular_z = side_nudge

        # --- Apply Velocities ---
        self.twist.linear.x = target_linear_x
        self.twist.angular.z = np.clip(target_angular_z, -self.max_angular_speed, self.max_angular_speed) # Ensure angular speed is within limits

        # Publish Command
        self.cmd_vel_pub.publish(self.twist)


    def on_shutdown(self):
        """Ensure robot stops when node is shut down."""
        self.get_logger().info("Node shutting down. Stopping robot...")
        self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = FastExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure cleanup happens
        if rclpy.ok():
            node.on_shutdown() # Call the explicit shutdown handler
            if node.timer is not None and not node.timer.canceled:
                 node.timer.cancel()
            node.destroy_node()
        rclpy.shutdown()
        print("ROS Cleanup Complete.")


if __name__ == '__main__':
    main()