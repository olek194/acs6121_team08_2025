#!/usr/bin/env python3
# Perimeter-Focused exploration node for TurtleBot3 Waffle

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import enum
from typing import List, Set, Dict, Tuple, Optional

class ExplorationState(enum.Enum):
    """State machine for the explorer robot"""
    INITIAL = 0        # Initial state, moving from center to perimeter
    PERIMETER = 1      # Following the perimeter
    TRANSIT = 2        # Moving between non-adjacent perimeter zones
    RECOVERY = 3       # Recovering from obstacles or interior zones

class PerimeterExplorerNode(Node):

    def __init__(self):
        super().__init__("perimeter_explorer_node")

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, "scan", self.lidar_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # Timer for 90-second runtime (restored)
        self.start_time = self.get_clock().now()
        self.runtime_limit = 90.0  # seconds
        self.timer = self.create_timer(0.1, self.check_runtime)
        self.is_stopped = False
        self.shutdown_flag = False

        # Position tracking
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        
        # Arena zones tracking
        self.arena_size_x = 4.0  # meters
        self.arena_size_y = 4.0  # meters
        self.zones_visited = set()  # Track visited zones
        self.current_zone = None
        
        # Define perimeter zones explicitly
        self.perimeter_zones = {1, 2, 3, 4, 5, 8, 9, 12, 13, 14, 15, 16}
        self.interior_zones = {6, 7, 10, 11}
        
        # Zone target and path planning
        self.target_zone = None
        self.next_zones_queue = []
        self.exploration_state = ExplorationState.INITIAL
        self.perimeter_found = False
        
        # Velocity message
        self.twist = Twist()

        # --- Tunable Parameters ---
        # Distances (meters)
        self.critical_front_distance = 0.55
        self.warning_front_distance = 0.85
        self.side_avoid_distance = 0.60
        self.perimeter_follow_distance = 0.65

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
        
        # Initialize the zone priority queue
        self.initialize_zone_queue()

        self.get_logger().info(f"'{self.get_name()}' node initialized.")
        
        # Start moving immediately
        self.start_moving()

    def initialize_zone_queue(self):
        """Initialize the priority queue for zone exploration"""
        # Determine closest perimeter zone from center (starting point)
        # In this case, we'll start by moving to zone 1 (top-left corner)
        self.next_zones_queue = [1, 2, 3, 4, 8, 12, 16, 15, 14, 13, 9, 5]
        self.target_zone = self.next_zones_queue[0]
        self.get_logger().info(f"Initial target zone: {self.target_zone}")

    def check_runtime(self):
        """Check if runtime limit exceeded and stop the robot."""
        if self.is_stopped or self.shutdown_flag:
            return
            
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time >= self.runtime_limit:
            self.get_logger().info(f"{self.runtime_limit} seconds elapsed. Stopping exploration.")
            self.get_logger().info(f"Visited {len(self.zones_visited)} zones: {sorted(list(self.zones_visited))}")
            # Count perimeter zones visited
            perimeter_zones_visited = self.perimeter_zones.intersection(self.zones_visited)
            self.get_logger().info(f"Visited {len(perimeter_zones_visited)}/{len(self.perimeter_zones)} perimeter zones")
            self.stop_robot()
            self.is_stopped = True
            if not self.timer.canceled:
                self.timer.cancel()

    def start_moving(self):
        """Start the robot moving toward the perimeter."""
        self.exploration_state = ExplorationState.INITIAL
        self.twist.linear.x = self.max_linear_speed
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Starting perimeter exploration!")

    def stop_robot(self):
        """Sends a zero velocity command to stop the robot."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Robot stopped.")

    def update_target_zone(self):
        """Update the target zone based on current position and exploration state."""
        if self.current_zone in self.perimeter_zones:
            # We're on the perimeter, update state if needed
            if self.exploration_state != ExplorationState.PERIMETER:
                self.exploration_state = ExplorationState.PERIMETER
                self.get_logger().info(f"Reached perimeter in zone {self.current_zone}. Entering PERIMETER state.")
            
            # If we've reached the current target zone, update to next
            if self.current_zone == self.target_zone and len(self.next_zones_queue) > 0:
                # Remove current zone from queue if it's at the front
                if self.next_zones_queue and self.next_zones_queue[0] == self.current_zone:
                    self.next_zones_queue.pop(0)
                
                # Get next zone if available
                if self.next_zones_queue:
                    self.target_zone = self.next_zones_queue[0]
                    self.get_logger().info(f"New target zone: {self.target_zone}")
                    
                    # Check if we need to transit (non-adjacent zones)
                    if not self.are_zones_adjacent(self.current_zone, self.target_zone):
                        self.exploration_state = ExplorationState.TRANSIT
                        self.get_logger().info(f"Non-adjacent zone target. Entering TRANSIT state.")
        
        elif self.current_zone in self.interior_zones:
            # We're in an interior zone, need to get back to perimeter
            self.exploration_state = ExplorationState.RECOVERY
            self.get_logger().info(f"In interior zone {self.current_zone}. Entering RECOVERY state.")

    def are_zones_adjacent(self, zone1: int, zone2: int) -> bool:
        """Check if two zones are adjacent."""
        # Calculate row and column for each zone
        row1, col1 = divmod(zone1 - 1, 4)
        row2, col2 = divmod(zone2 - 1, 4)
        
        # Check if zones are adjacent (horizontally or vertically)
        return (abs(row1 - row2) + abs(col1 - col2)) == 1

    def get_direction_to_zone(self, target_zone: int) -> Tuple[float, float]:
        """Calculate direction vector to target zone center from current position."""
        # Calculate target zone center
        zone_row, zone_col = divmod(target_zone - 1, 4)
        # Zone centers are at 0.5, 1.5, 2.5, 3.5 in each dimension
        target_x = (zone_col + 0.5) - 2.0  # Adjust to robot coordinate system
        target_y = 2.0 - (zone_row + 0.5)  # Adjust to robot coordinate system
        
        # Calculate direction vector
        dir_x = target_x - self.x
        dir_y = target_y - self.y
        
        return dir_x, dir_y

    def get_current_zone(self):
        """Calculate which zone the robot is in (1-16, numbered left-to-right, top-to-bottom)."""
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
        
        # Extract orientation (yaw/theta_z)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        # This is a simplified calculation for yaw only
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.theta_z = math.atan2(siny_cosp, cosy_cosp)
        
        # Update zone tracking
        new_zone = self.get_current_zone()
        if new_zone != self.current_zone:
            self.current_zone = new_zone
            self.zones_visited.add(new_zone)
            self.get_logger().info(f"Entered zone {new_zone}. Total zones visited: {len(self.zones_visited)}")
            
            # Update target based on new zone
            self.update_target_zone()

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
        valid_ranges = [r for r in front_ranges if range_min_thresh < r < range_max_thresh and math.isfinite(r)]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn("No valid front readings!")
        
        dist_f = min(valid_ranges, default=range_max_thresh)

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

        # State-based behavior
        if self.exploration_state == ExplorationState.INITIAL:
            # Initial state: Move from center to perimeter
            target_linear_x, target_angular_z = self.handle_initial_state(dist_f, dist_fl, dist_fr, dist_l, dist_r)
        
        elif self.exploration_state == ExplorationState.PERIMETER:
            # Perimeter following state
            target_linear_x, target_angular_z = self.handle_perimeter_state(dist_f, dist_fl, dist_fr, dist_l, dist_r)
        
        elif self.exploration_state == ExplorationState.TRANSIT:
            # Moving between non-adjacent perimeter zones
            target_linear_x, target_angular_z = self.handle_transit_state(dist_f, dist_fl, dist_fr, dist_l, dist_r)
        
        elif self.exploration_state == ExplorationState.RECOVERY:
            # Recovering from obstacles or interior zones
            target_linear_x, target_angular_z = self.handle_recovery_state(dist_f, dist_fl, dist_fr, dist_l, dist_r)

        # Apply critical obstacle avoidance (safety override)
        if dist_f < self.critical_front_distance:
            self.get_logger().warn(f"CRITICAL front obstacle: {dist_f:.2f}m - emergency avoidance")
            target_linear_x = 0.0
            if dist_fl > dist_fr:
                target_angular_z = self.max_angular_speed
            else:
                target_angular_z = -self.max_angular_speed

        # Apply velocities with limits
        self.twist.linear.x = target_linear_x
        self.twist.angular.z = max(-self.max_angular_speed, min(target_angular_z, self.max_angular_speed))
        self.cmd_vel_pub.publish(self.twist)

    def handle_initial_state(self, dist_f, dist_fl, dist_fr, dist_l, dist_r) -> Tuple[float, float]:
        """Handle the initial movement from center to perimeter."""
        # Get direction to target zone (initially zone 1)
        dir_x, dir_y = self.get_direction_to_zone(self.target_zone)
        
        # Calculate desired heading angle
        desired_theta = math.atan2(dir_y, dir_x)
        
        # Calculate angular error
        angular_error = self.normalize_angle(desired_theta - self.theta_z)
        
        # P-controller for angular velocity
        target_angular_z = angular_error * 1.5
        
        # If we're facing approximately the right direction, go forward
        if abs(angular_error) < 0.3:  # ~17 degrees tolerance
            target_linear_x = self.max_linear_speed
        else:
            # Slow down while turning
            target_linear_x = self.cautious_linear_speed
        
        # Check if we've reached the perimeter
        if self.current_zone in self.perimeter_zones:
            self.exploration_state = ExplorationState.PERIMETER
            self.get_logger().info(f"Reached perimeter in zone {self.current_zone}")
        
        # Apply obstacle avoidance if needed
        if dist_f < self.warning_front_distance:
            target_linear_x = self.cautious_linear_speed
            if dist_fl > dist_fr:
                target_angular_z = self.gentle_turn_speed
            else:
                target_angular_z = -self.gentle_turn_speed
        
        return target_linear_x, target_angular_z

    def handle_perimeter_state(self, dist_f, dist_fl, dist_fr, dist_l, dist_r) -> Tuple[float, float]:
        """Handle perimeter following behavior."""
        target_linear_x = self.max_linear_speed
        target_angular_z = 0.0
        
        # Decide which side to follow based on current zone and target
        if self.should_follow_left_wall():
            # Follow left wall
            if dist_l > 1.5:  # No wall on left, turn to find it
                self.get_logger().info("No left wall detected, turning to find it")
                target_linear_x = self.cautious_linear_speed
                target_angular_z = self.gentle_turn_speed
            else:
                # Control distance to left wall
                error = dist_l - self.perimeter_follow_distance
                # P-controller for wall following
                target_angular_z = -error * 2.0
                
                # Adjust speed based on wall curvature
                if abs(error) > 0.2:
                    target_linear_x = self.cautious_linear_speed
        else:
            # Follow right wall
            if dist_r > 1.5:  # No wall on right, turn to find it
                self.get_logger().info("No right wall detected, turning to find it")
                target_linear_x = self.cautious_linear_speed
                target_angular_z = -self.gentle_turn_speed
            else:
                # Control distance to right wall
                error = dist_r - self.perimeter_follow_distance
                # P-controller for wall following
                target_angular_z = error * 2.0
                
                # Adjust speed based on wall curvature
                if abs(error) > 0.2:
                    target_linear_x = self.cautious_linear_speed
        
        # Obstacle avoidance takes precedence
        if dist_f < self.warning_front_distance:
            target_linear_x = self.cautious_linear_speed
            if dist_fl > dist_fr:
                target_angular_z = self.gentle_turn_speed
            else:
                target_angular_z = -self.gentle_turn_speed
        
        return target_linear_x, target_angular_z

    def handle_transit_state(self, dist_f, dist_fl, dist_fr, dist_l, dist_r) -> Tuple[float, float]:
        """Handle movement between non-adjacent perimeter zones."""
        # Get direction to target zone
        dir_x, dir_y = self.get_direction_to_zone(self.target_zone)
        
        # Calculate desired heading angle
        desired_theta = math.atan2(dir_y, dir_x)
        
        # Calculate angular error
        angular_error = self.normalize_angle(desired_theta - self.theta_z)
        
        # P-controller for angular velocity
        target_angular_z = angular_error * 1.5
        
        # If we're facing approximately the right direction, go forward
        if abs(angular_error) < 0.3:  # ~17 degrees tolerance
            target_linear_x = self.max_linear_speed
        else:
            # Slow down while turning
            target_linear_x = self.cautious_linear_speed
        
        # Check if we've reached the perimeter target zone
        if self.current_zone == self.target_zone:
            self.exploration_state = ExplorationState.PERIMETER
            self.get_logger().info(f"Reached target zone {self.current_zone}. Returning to PERIMETER state.")
        
        # Apply obstacle avoidance if needed
        if dist_f < self.warning_front_distance:
            target_linear_x = self.cautious_linear_speed
            if dist_fl > dist_fr:
                target_angular_z = self.gentle_turn_speed
            else:
                target_angular_z = -self.gentle_turn_speed
        
        return target_linear_x, target_angular_z

    def handle_recovery_state(self, dist_f, dist_fl, dist_fr, dist_l, dist_r) -> Tuple[float, float]:
        """Handle recovery from interior zones or obstacles."""
        # Find nearest perimeter zone and head toward it
        nearest_perim_zone = self.find_nearest_perimeter_zone()
        
        # Get direction to nearest perimeter zone
        dir_x, dir_y = self.get_direction_to_zone(nearest_perim_zone)
        
        # Calculate desired heading angle
        desired_theta = math.atan2(dir_y, dir_x)
        
        # Calculate angular error
        angular_error = self.normalize_angle(desired_theta - self.theta_z)
        
        # P-controller for angular velocity
        target_angular_z = angular_error * 1.5
        
        # If we're facing approximately the right direction, go forward
        if abs(angular_error) < 0.3:  # ~17 degrees tolerance
            target_linear_x = self.max_linear_speed
        else:
            # Slow down while turning
            target_linear_x = self.cautious_linear_speed
        
        # Check if we've reached the perimeter
        if self.current_zone in self.perimeter_zones:
            # If we've reached a perimeter zone, update state
            self.exploration_state = ExplorationState.PERIMETER
            self.get_logger().info(f"Recovered to perimeter in zone {self.current_zone}")
        
        # Apply obstacle avoidance if needed
        if dist_f < self.warning_front_distance:
            target_linear_x = self.cautious_linear_speed
            if dist_fl > dist_fr:
                target_angular_z = self.gentle_turn_speed
            else:
                target_angular_z = -self.gentle_turn_speed
        
        return target_linear_x, target_angular_z

    def should_follow_left_wall(self) -> bool:
        """Determine whether to follow the left or right wall based on current position."""
        # Simple strategy: for zones on the left half of the arena (1,5,9,13), follow right wall
        # For zones on the right half (4,8,12,16), follow left wall
        # For others, decide based on target zone direction
        left_edge_zones = {1, 5, 9, 13}
        right_edge_zones = {4, 8, 12, 16}
        
        if self.current_zone in left_edge_zones:
            return False  # Follow right wall
        elif self.current_zone in right_edge_zones:
            return True  # Follow left wall
        else:
            # For other zones, calculate based on target direction
            zone_row, zone_col = divmod(self.current_zone - 1, 4)
            target_row, target_col = divmod(self.target_zone - 1, 4)
            
            # If target is to the left, follow left wall
            if target_col < zone_col:
                return True
            # If target is to the right, follow right wall
            else:
                return False

    def find_nearest_perimeter_zone(self) -> int:
        """Find the nearest perimeter zone from current position."""
        nearest_zone = None
        min_distance = float('inf')
        
        for zone in self.perimeter_zones:
            zone_row, zone_col = divmod(zone - 1, 4)
            # Zone centers are at 0.5, 1.5, 2.5, 3.5 in each dimension
            zone_x = (zone_col + 0.5) - 2.0  # Adjust to robot coordinate system
            zone_y = 2.0 - (zone_row + 0.5)  # Adjust to robot coordinate system
            
            # Calculate Euclidean distance
            distance = math.sqrt((zone_x - self.x)**2 + (zone_y - self.y)**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_zone = zone
        
        return nearest_zone

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def on_shutdown(self):
        """Ensure robot stops when node is shut down."""
        if not self.shutdown_flag:
            self.get_logger().info("Node shutting down. Stopping robot...")
            self.stop_robot()
            if self.timer is not None and not self.timer.canceled:
                self.get_logger().info("Cancelling runtime timer.")
                self.timer.cancel()
            self.shutdown_flag = True

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = PerimeterExplorerNode()
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