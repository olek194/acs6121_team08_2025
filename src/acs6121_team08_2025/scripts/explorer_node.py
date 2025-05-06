#!/usr/bin/env python3
# Optimized exploration node for TurtleBot3 Waffle - Focus on Speed and Zone Coverage

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Added for position tracking
from sensor_msgs.msg import LaserScan
import math

class FastExplorerNode(Node):

    def __init__(self):
        super().__init__("fast_explorer_node")

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, "scan", self.lidar_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # Timer for 90-second runtime
        self.start_time = self.get_clock().now()  # Start timer immediately
        self.runtime_limit = 90.0  # seconds
        self.timer = self.create_timer(0.1, self.check_runtime)
        self.is_stopped = False
        self.shutdown_flag = False

        # Position tracking (similar to move_square.py)
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        
        # Arena zones tracking
        self.arena_size_x = 4.0  # meters (assumed)
        self.arena_size_y = 4.0  # meters (assumed)
        self.zones_visited = set()  # Track visited zones
        self.current_zone = None
        
        # Velocity message
        self.twist = Twist()

        # --- Tunable Parameters ---
        # Distances (meters)
        self.critical_front_distance = 0.40
        self.warning_front_distance = 0.70
        self.side_avoid_distance = 0.45

        # Speeds
        self.max_linear_speed = 0.28
        self.cautious_linear_speed = 0.18
        self.max_angular_speed = 1.9
        self.gentle_turn_speed = 0.8

        # LiDAR Sector Angles (degrees)
        self.front_angle = 15
        self.front_side_angle = 45
        self.side_angle_start = 50
        self.side_angle_end = 130

        self.get_logger().info(f"'{self.get_name()}' node initialized.")
        
        # Start moving immediately
        self.start_moving()

    def start_moving(self):
        """Start the robot moving forward."""
        self.twist.linear.x = self.max_linear_speed
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Starting exploration!")

    def check_runtime(self):
        """Check if runtime limit exceeded and stop the robot."""
        if self.is_stopped or self.shutdown_flag:
            return
            
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time >= self.runtime_limit:
            self.get_logger().info(f"{self.runtime_limit} seconds elapsed. Stopping exploration.")
            self.get_logger().info(f"Visited {len(self.zones_visited)} zones: {sorted(list(self.zones_visited))}")
            self.stop_robot()
            self.is_stopped = True
            if self.timer is not None and not self.timer.canceled:
                self.timer.cancel()

    def stop_robot(self):
        """Sends a zero velocity command to stop the robot."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Robot stopped.")

    def get_current_zone(self):
        """Calculate which zone the robot is in (1-16, numbered left-to-right, top-to-bottom)."""
        # Assuming arena is 4x4m and divided into 16 1x1m squares
        # Normalize coordinates to 0-4 range and calculate zone
        x_norm = (self.x + self.arena_size_x/2) / self.arena_size_x * 4
        y_norm = (self.arena_size_y/2 - self.y) / self.arena_size_y * 4
        
        # Ensure coordinates are within bounds
        x_norm = max(0, min(3.99, x_norm))
        y_norm = max(0, min(3.99, y_norm))
        
        # Calculate zone number (1-16)
        col = int(x_norm)
        row = int(y_norm)
        zone = row * 4 + col + 1
        
        return zone

    def odom_callback(self, msg: Odometry):
        """Track robot position and update zone information."""
        # Update position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Update zone tracking
        new_zone = self.get_current_zone()
        if new_zone != self.current_zone:
            self.current_zone = new_zone
            self.zones_visited.add(new_zone)
            self.get_logger().info(f"Entered zone {new_zone}. Total zones visited: {len(self.zones_visited)}")

    def get_sector_distances(self, msg: LaserScan):
        """ Get minimum distances in key sectors using angles """
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)
        range_min_thresh = msg.range_min + 0.02
        range_max_thresh = msg.range_max

        # Define angles in radians
        front_rad = math.radians(self.front_angle)
        front_side_rad = math.radians(self.front_side_angle)
        side_start_rad = math.radians(self.side_angle_start)
        side_end_rad = math.radians(self.side_angle_end)
        
        # Calculate indices for each sector
        idx_front_delta = int(front_rad / angle_increment)
        
        # Front sector (combining positive and negative angles)
        front_indices_positive = list(range(0, idx_front_delta + 1))
        front_indices_negative = list(range(num_ranges - idx_front_delta, num_ranges))
        front_combined_indices = front_indices_positive + front_indices_negative

        front_ranges = [ranges[i] for i in front_combined_indices if 0 <= i < num_ranges]
        dist_f = min([r for r in front_ranges if range_min_thresh < r < range_max_thresh and math.isfinite(r)], default=range_max_thresh)

        # Front-Left sector
        idx_fl_start = idx_front_delta + 1
        idx_fl_end = int(front_side_rad / angle_increment)
        dist_fl = min([ranges[i] for i in range(idx_fl_start, idx_fl_end + 1) 
                      if 0 <= i < num_ranges and range_min_thresh < ranges[i] < range_max_thresh 
                      and math.isfinite(ranges[i])], default=range_max_thresh)

        # Front-Right sector
        idx_fr_start = num_ranges - int(front_side_rad / angle_increment)
        idx_fr_end = num_ranges - (idx_front_delta + 1)
        dist_fr = min([ranges[i] for i in range(idx_fr_start, idx_fr_end + 1)
                      if 0 <= i < num_ranges and range_min_thresh < ranges[i] < range_max_thresh
                      and math.isfinite(ranges[i])], default=range_max_thresh)
        
        # Left Side sector
        idx_l_start = int(side_start_rad / angle_increment)
        idx_l_end = int(side_end_rad / angle_increment)
        dist_l = min([ranges[i] for i in range(idx_l_start, idx_l_end + 1)
                     if 0 <= i < num_ranges and range_min_thresh < ranges[i] < range_max_thresh
                     and math.isfinite(ranges[i])], default=range_max_thresh)

        # Right Side sector
        idx_r_start = num_ranges - int(side_end_rad / angle_increment)
        idx_r_end = num_ranges - int(side_start_rad / angle_increment)
        dist_r = min([ranges[i] for i in range(idx_r_start, idx_r_end + 1)
                     if 0 <= i < num_ranges and range_min_thresh < ranges[i] < range_max_thresh
                     and math.isfinite(ranges[i])], default=range_max_thresh)

        return dist_f, dist_fl, dist_fr, dist_l, dist_r

    def lidar_callback(self, msg: LaserScan):
        """Main control loop for obstacle avoidance and exploration."""
        if self.is_stopped or self.shutdown_flag:
            return

        dist_f, dist_fl, dist_fr, dist_l, dist_r = self.get_sector_distances(msg)

        # Default to maximum speed for exploration
        target_linear_x = self.max_linear_speed
        target_angular_z = 0.0

        # Obstacle avoidance logic
        if dist_f < self.critical_front_distance:
            self.get_logger().warn(f"CRITICAL front obstacle: {dist_f:.2f}m. Turning.")
            target_linear_x = 0.0
            if dist_fl > dist_fr:
                target_angular_z = self.max_angular_speed
            else:
                target_angular_z = -self.max_angular_speed

        elif dist_f < self.warning_front_distance:
            self.get_logger().info(f"Warning front obstacle: {dist_f:.2f}m. Slowing/Turning.")
            target_linear_x = self.cautious_linear_speed
            if dist_fl > dist_fr:
                target_angular_z = self.gentle_turn_speed
            else:
                target_angular_z = -self.gentle_turn_speed

        else:
            # No front obstacles - check sides
            target_linear_x = self.max_linear_speed
            side_nudge = 0.0
            
            if dist_l < self.side_avoid_distance:
                error = self.side_avoid_distance - dist_l
                side_nudge = -self.gentle_turn_speed * (error / self.side_avoid_distance) * 1.5
                self.get_logger().debug(f"Nudging right from left wall: {dist_l:.2f}m")
            elif dist_r < self.side_avoid_distance:
                error = self.side_avoid_distance - dist_r
                side_nudge = self.gentle_turn_speed * (error / self.side_avoid_distance) * 1.5
                self.get_logger().debug(f"Nudging left from right wall: {dist_r:.2f}m")
            
            target_angular_z = side_nudge

        # Apply velocities with limits
        self.twist.linear.x = target_linear_x
        self.twist.angular.z = max(-self.max_angular_speed, min(target_angular_z, self.max_angular_speed))

        self.cmd_vel_pub.publish(self.twist)

    def on_shutdown(self):
        """Ensure robot stops and timers are cancelled when node is shut down."""
        if not self.shutdown_flag:
            self.get_logger().info("Node shutting down. Stopping robot...")
            self.stop_robot()
            if self.timer is not None and not self.timer.canceled:
                self.get_logger().info("Cancelling runtime timer.")
                self.timer.cancel()
            self.is_stopped = True
            self.shutdown_flag = True

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = FastExplorerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        if node is not None:
            node.get_logger().info("Finalizing shutdown...")
            node.on_shutdown()
            
            if rclpy.ok():
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if node.shutdown_flag and not rclpy.ok():
                        break
            
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        print("ROS Cleanup Complete.")

if __name__ == '__main__':
    main()