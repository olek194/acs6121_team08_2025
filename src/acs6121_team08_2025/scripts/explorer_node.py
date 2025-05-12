#!/usr/bin/env python3
# Optimized exploration node for TurtleBot3 Waffle - Focus on outer box exploration

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from enum import Enum, auto

class ExplorationState(Enum):
    INIT = auto()           # Initial state, moving to first position
    NAVIGATING = auto()     # Moving to next target
    ROTATING = auto()       # Rotating to align with next target
    AVOIDING = auto()       # Avoiding obstacles

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

        self.shutdown_flag = False

        # Position tracking
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        
        # Arena configuration
        self.box_size = 1.0  # 1x1 meter boxes
        self.arena_size_x = 4.0
        self.arena_size_y = 4.0
        
        # Path planning
        self.current_state = ExplorationState.INIT
        self.target_box = 0
        self.boxes_to_explore = [1,2,3,4,5,8,9,12,13,14,15,16]  # Outer boxes only
        self.visited_boxes = set()
        
        # Box center positions (x,y) relative to arena center
        self.box_positions = {
            1: (-1.5, 1.5),   2: (-0.5, 1.5),   3: (0.5, 1.5),   4: (1.5, 1.5),
            5: (-1.5, 0.5),   6: (-0.5, 0.5),   7: (0.5, 0.5),   8: (1.5, 0.5),
            9: (-1.5, -0.5), 10: (-0.5, -0.5), 11: (0.5, -0.5), 12: (1.5, -0.5),
            13: (-1.5, -1.5), 14: (-0.5, -1.5), 15: (0.5, -1.5), 16: (1.5, -1.5)
        }
        
        # Current target position
        self.target_x = 0.0
        self.target_y = 0.0
        
        # Navigation parameters - adjusted for more efficient movement
        self.position_tolerance = 0.15  # Increased tolerance for faster transitions
        self.angle_tolerance = 0.15     # Increased angle tolerance
        self.target_heading = 0.0

        # Tunable Parameters - adjusted for smoother movement through inner boxes
        self.critical_front_distance = 0.50  # Reduced since we don't need to be as cautious
        self.warning_front_distance = 0.70   # Reduced for more direct paths
        self.side_avoid_distance = 0.45      # Reduced side clearance
        self.min_clearance = 0.40           # Reduced minimum clearance

        # Speeds - adjusted for faster movement
        self.max_linear_speed = 0.30        # Slightly increased
        self.cautious_linear_speed = 0.18   # Increased for faster obstacle passing
        self.max_angular_speed = 1.9
        self.gentle_turn_speed = 1.2

        # LiDAR Sector Angles (degrees)
        self.front_angle = 20
        self.front_side_angle = 50
        self.side_angle_start = 50
        self.side_angle_end = 130

        # Velocity message
        self.twist = Twist()
        
        self.get_logger().info("Starting exploration from center position!")
        self.set_next_target()

    def set_next_target(self):
        """Set the next target box to explore."""
        # If we've visited all boxes, we're done
        if len(self.visited_boxes) >= len(self.boxes_to_explore):
            self.get_logger().info("Exploration complete! All outer boxes visited.")
            self.stop_robot()
            return False

        # Get next unvisited box - now using a more efficient path
        current_box = self.get_current_box()
        min_distance = float('inf')
        next_box = None

        # Find the closest unvisited box
        for box in self.boxes_to_explore:
            if box not in self.visited_boxes:
                box_x, box_y = self.box_positions[box]
                dx = box_x - self.x
                dy = box_y - self.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < min_distance:
                    min_distance = distance
                    next_box = box

        if next_box:
            self.target_box = next_box
            self.target_x, self.target_y = self.box_positions[next_box]
            self.get_logger().info(f"Setting new target: Box {next_box} at ({self.target_x:.2f}, {self.target_y:.2f})")
            return True
        
        return False

    def get_current_box(self):
        """Calculate which box the robot is in."""
        # Normalize coordinates to box grid
        x_norm = (self.x + self.arena_size_x/2) / self.box_size
        y_norm = (self.arena_size_y/2 - self.y) / self.box_size
        
        # Calculate box number (1-16)
        col = int(x_norm)
        row = int(y_norm)
        box = row * 4 + col + 1
        
        return box

    def update_navigation(self):
        """Update navigation state and set appropriate velocities."""
        current_box = self.get_current_box()
        
        # Mark current box as visited if it's in our target list
        if current_box in self.boxes_to_explore and current_box not in self.visited_boxes:
            self.visited_boxes.add(current_box)
            self.get_logger().info(f"Visited box {current_box}. Total boxes visited: {len(self.visited_boxes)}")
            self.set_next_target()

        # Calculate distance and angle to target
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Normalize angle difference to [-pi, pi]
        angle_diff = target_angle - self.theta_z
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi

        # State machine for navigation
        if self.current_state == ExplorationState.INIT:
            if abs(angle_diff) > self.angle_tolerance:
                self.rotate_to_target(angle_diff)
            else:
                self.current_state = ExplorationState.NAVIGATING
                
        elif self.current_state == ExplorationState.NAVIGATING:
            if distance < self.position_tolerance:
                self.set_next_target()
            elif abs(angle_diff) > self.angle_tolerance * 2:
                self.current_state = ExplorationState.ROTATING
            else:
                self.move_to_target(distance, angle_diff)
                
        elif self.current_state == ExplorationState.ROTATING:
            if abs(angle_diff) < self.angle_tolerance:
                self.current_state = ExplorationState.NAVIGATING
            else:
                self.rotate_to_target(angle_diff)

        return distance, angle_diff

    def rotate_to_target(self, angle_diff):
        """Rotate towards target angle."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = max(-self.max_angular_speed, 
                                 min(self.max_angular_speed, angle_diff))

    def move_to_target(self, distance, angle_diff):
        """Move towards target position."""
        # Scale linear speed based on distance and angle
        speed_factor = min(1.0, distance / 0.5)  # Slow down when close
        angle_factor = max(0.0, 1.0 - abs(angle_diff))  # Slow down when not aligned
        
        # More aggressive movement when far from target
        if distance > 1.0:
            speed_factor = 1.0
            angle_factor = max(0.3, angle_factor)  # Maintain some forward motion while turning
        
        self.twist.linear.x = self.max_linear_speed * speed_factor * angle_factor
        self.twist.angular.z = max(-self.gentle_turn_speed, 
                                min(self.gentle_turn_speed, angle_diff))

    def odom_callback(self, msg: Odometry):
        """Update robot's position and orientation."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.theta_z = math.atan2(siny_cosp, cosy_cosp)
        
        # Update navigation if not in obstacle avoidance
        if self.current_state != ExplorationState.AVOIDING:
            self.update_navigation()

    def lidar_callback(self, msg: LaserScan):
        """Handle obstacle avoidance during navigation."""
        if self.shutdown_flag:
            return

        dist_f, dist_fl, dist_fr, dist_l, dist_r = self.get_sector_distances(msg)

        # Check if we need to avoid obstacles
        if dist_f < self.critical_front_distance or dist_f < self.warning_front_distance:
            self.current_state = ExplorationState.AVOIDING
            self.handle_obstacle_avoidance(dist_f, dist_fl, dist_fr, dist_l, dist_r)
        elif self.current_state == ExplorationState.AVOIDING:
            # Return to normal navigation
            self.current_state = ExplorationState.NAVIGATING
            self.update_navigation()

        self.cmd_vel_pub.publish(self.twist)

    def handle_obstacle_avoidance(self, dist_f, dist_fl, dist_fr, dist_l, dist_r):
        """Handle obstacle avoidance logic."""
        if dist_f < self.critical_front_distance:
            # Critical front obstacle - stop and turn
            self.twist.linear.x = 0.0
            
            # Simplified turning decision - just turn in the direction with more space
            if dist_fl > dist_fr:
                self.twist.angular.z = self.max_angular_speed
            else:
                self.twist.angular.z = -self.max_angular_speed
                
        elif dist_f < self.warning_front_distance:
            # Warning distance - slow down and start turning
            self.twist.linear.x = self.cautious_linear_speed
            distance_factor = (self.warning_front_distance - dist_f) / (self.warning_front_distance - self.critical_front_distance)
            turn_speed = self.gentle_turn_speed + (self.max_angular_speed - self.gentle_turn_speed) * distance_factor
            
            # Simplified turning decision
            if dist_fl > dist_fr:
                self.twist.angular.z = turn_speed
            else:
                self.twist.angular.z = -turn_speed

    def get_sector_distances(self, msg: LaserScan):
        """Get minimum distances in key sectors."""
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)
        range_min_thresh = msg.range_min + 0.02
        range_max_thresh = msg.range_max

        # Add debug info about LiDAR configuration
        self.get_logger().debug(
            f"LiDAR config: ranges={len(ranges)}, "
            f"angle_min={msg.angle_min:.2f}, "
            f"angle_max={msg.angle_max:.2f}, "
            f"increment={angle_increment:.4f}"
        )

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

        # Add debug info about sector indices
        self.get_logger().debug(
            f"Front sector indices: positive={front_indices_positive[0]}-{front_indices_positive[-1]}, "
            f"negative={front_indices_negative[0]}-{front_indices_negative[-1]}"
        )

        front_ranges = [ranges[i] for i in front_combined_indices if 0 <= i < num_ranges]
        valid_ranges = [r for r in front_ranges if range_min_thresh < r < range_max_thresh and math.isfinite(r)]
        
        # Add debug info about valid readings
        if len(valid_ranges) == 0:
            self.get_logger().warn("No valid front readings!")
            self.get_logger().debug(f"Front ranges: {front_ranges}")
        
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

    def stop_robot(self):
        """Stop the robot."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Robot stopped.")

    def on_shutdown(self):
        """Handle shutdown."""
        if not self.shutdown_flag:
            self.get_logger().info("Node shutting down. Stopping robot...")
            self.stop_robot()
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