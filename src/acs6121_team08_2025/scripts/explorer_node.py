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
    FINDING_SPACE = auto()    # Initial state - looking for open space
    NAVIGATING = auto()       # Moving to target
    AVOIDING = auto()         # Avoiding obstacles
    TURNING = auto()          # Executing 180-degree turn
    RECOVERING = auto()       # Returning to exploration path

class FastExplorerNode(Node):

    def __init__(self):
        super().__init__("fast_explorer_node")

        # Publisher and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
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
        
        # Arena parameters
        self.arena_size_x = 4.0  # 4m x 4m arena
        self.arena_size_y = 4.0
        self.box_size = 1.0      # 1m x 1m boxes
        
        # Robot physical parameters
        self.robot_radius = 0.25  # 25cm radius (50cm diameter)
        
        # Navigation parameters
        self.position_tolerance = 0.2
        self.open_space_threshold = 1.0  # Minimum distance to consider space "open"
        
        # Speeds
        self.max_linear_speed = 0.26
        self.cautious_linear_speed = 0.15
        self.max_angular_speed = 1.82
        self.search_turn_speed = 0.8     # Slower turn while searching for space
        
        # LiDAR Sector Angles (degrees)
        self.front_angle = 30            # Wider front detection for finding space
        self.front_side_angle = 45
        
        # Start in finding space state
        self.current_state = ExplorationState.FINDING_SPACE
        
        # Path planning
        self.target_box = 0
        self.boxes_to_explore = [1,2,3,4,5,8,9,12,13,14,15,16]
        self.visited_boxes = set()
        
        # Box positions
        self.box_positions = {
            1: (-1.5, 1.5),   2: (-0.5, 1.5),   3: (0.5, 1.5),   4: (1.5, 1.5),
            5: (-1.5, 0.5),   6: (-0.5, 0.5),   7: (0.5, 0.5),   8: (1.5, 0.5),
            9: (-1.5, -0.5), 10: (-0.5, -0.5), 11: (0.5, -0.5), 12: (1.5, -0.5),
            13: (-1.5, -1.5), 14: (-0.5, -1.5), 15: (0.5, -1.5), 16: (1.5, -1.5)
        }
        
        self.target_x = 0.0
        self.target_y = 0.0
        
        # Velocity message
        self.twist = Twist()
        
        # Timer for 90-second exploration
        self.start_time = self.get_clock().now()
        self.exploration_duration = 90.0  # seconds
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz timer
        
        self.get_logger().info("Starting exploration - searching for open space!")
        self.set_next_target()

        # Corner boxes that need 180-degree turn
        self.corner_boxes = {1, 4, 13, 16}
        
        # Recovery tracking
        self.recovery_target_x = 0.0
        self.recovery_target_y = 0.0
        self.pre_avoid_state = None
        self.turn_start_angle = 0.0
        self.turn_target_angle = 0.0

    def set_next_target(self):
        """Set the next target box to explore."""
        if len(self.visited_boxes) >= len(self.boxes_to_explore):
            self.get_logger().info("Exploration complete! All outer boxes visited.")
            self.stop_robot()
            return False

        # Find the next unvisited box in numerical order
        next_box = None
        for box in self.boxes_to_explore:  # boxes_to_explore is already in numerical order
            if box not in self.visited_boxes:
                next_box = box
                break

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
        
        # Calculate distance to current target
        if self.current_state == ExplorationState.RECOVERING:
            dx = self.recovery_target_x - self.x
            dy = self.recovery_target_y - self.y
        else:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
        
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Mark current box as visited if we're close enough
        if current_box in self.boxes_to_explore and current_box not in self.visited_boxes:
            if distance < self.position_tolerance:
                self.visited_boxes.add(current_box)
                self.get_logger().info(f"Visited box {current_box}. Total boxes visited: {len(self.visited_boxes)}")
                
                # If we're in a corner box, initiate 180-degree turn
                if current_box in self.corner_boxes:
                    self.get_logger().info("Corner box reached - initiating 180-degree turn")
                    self.current_state = ExplorationState.TURNING
                    self.turn_start_angle = self.theta_z
                    self.turn_target_angle = self.normalize_angle(self.theta_z + math.pi)
                    return
                
                # Set next target and continue
                if self.set_next_target():
                    self.current_state = ExplorationState.NAVIGATING
                return
        
        # Handle recovery completion
        if self.current_state == ExplorationState.RECOVERING and distance < self.position_tolerance:
            self.get_logger().info("Recovery complete - resuming normal navigation")
            self.current_state = ExplorationState.NAVIGATING
            if not self.set_next_target():
                self.stop_robot()
            return
        
        # Continue moving to target
        self.move_to_target(dx, dy, distance)

    def move_to_target(self, dx, dy, distance):
        """Move directly towards target."""
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(target_angle - self.theta_z)
            
        # If we're facing roughly the right direction, move forward
        if abs(angle_diff) < 0.3:  # ~17 degrees tolerance
            self.twist.linear.x = self.max_linear_speed
            self.twist.angular.z = max(-0.3, min(0.3, angle_diff))  # Small corrections
        else:
            # Turn to face target first
            self.twist.linear.x = 0.0
            self.twist.angular.z = max(-0.8, min(0.8, angle_diff))

    def odom_callback(self, msg: Odometry):
        """Update robot's position and orientation."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.theta_z = math.atan2(siny_cosp, cosy_cosp)
        
        # Update navigation if not avoiding obstacles
        if self.current_state != ExplorationState.AVOIDING:
            self.update_navigation()

    def lidar_callback(self, msg: LaserScan):
        """Handle movement based on current state."""
        if self.shutdown_flag:
            return

        dist_f, dist_fl, dist_fr = self.get_sector_distances(msg)

        if self.current_state == ExplorationState.FINDING_SPACE:
            self.handle_space_finding(dist_f, dist_fl, dist_fr)
        elif self.current_state == ExplorationState.NAVIGATING:
            if dist_f < (0.45 + self.robot_radius):  # Only avoid very close obstacles
                self.current_state = ExplorationState.AVOIDING
                self.handle_obstacle_avoidance(dist_f, dist_fl, dist_fr)
            else:
                self.update_navigation()
        elif self.current_state == ExplorationState.AVOIDING:
            self.handle_obstacle_avoidance(dist_f, dist_fl, dist_fr)
        elif self.current_state == ExplorationState.TURNING:
            self.handle_turning()
        elif self.current_state == ExplorationState.RECOVERING:
            if dist_f < (0.45 + self.robot_radius):  # Obstacle during recovery
                self.current_state = ExplorationState.AVOIDING
                self.handle_obstacle_avoidance(dist_f, dist_fl, dist_fr)
            else:
                self.update_navigation()

        self.cmd_vel_pub.publish(self.twist)

    def handle_space_finding(self, dist_f, dist_fl, dist_fr):
        """Initial behavior: move forward slowly while turning until open space found."""
        self.twist.linear.x = self.cautious_linear_speed
        
        # Keep turning in one direction (left) until we find open space
        if dist_f > self.open_space_threshold and dist_fl > self.open_space_threshold:
            self.get_logger().info("Found open space! Moving to first target.")
            self.current_state = ExplorationState.NAVIGATING
            # Set full speed toward target
            self.twist.linear.x = self.max_linear_speed
            self.twist.angular.z = 0.0
        else:
            # Keep turning left while moving slowly
            self.twist.angular.z = self.search_turn_speed

    def handle_obstacle_avoidance(self, dist_f, dist_fl, dist_fr):
        """Enhanced obstacle avoidance with path recovery."""
        # Check if we were previously avoiding
        was_avoiding = (self.current_state == ExplorationState.AVOIDING)
        
        # If we find open space while avoiding, enter recovery
        # Require front and at least one side to be clear enough to proceed
        if dist_f > self.open_space_threshold and max(dist_fl, dist_fr) > (0.5 + self.robot_radius): # Check if sides have enough clearance
            if was_avoiding:
                self.get_logger().info("Found open space - entering recovery mode")
                self.current_state = ExplorationState.RECOVERING
                # Set recovery velocity - move forward cautiously
                self.twist.linear.x = self.cautious_linear_speed
                self.twist.angular.z = 0.0
            else:
                 # Store current target for recovery if just entering avoid state
                 self.recovery_target_x = self.target_x
                 self.recovery_target_y = self.target_y
                 self.pre_avoid_state = self.current_state
                 self.current_state = ExplorationState.AVOIDING # Ensure we are in AVOIDING state
                 # Start turning immediately
                 self.twist.linear.x = 0.0
                 turn_direction = 1 if dist_fl > dist_fr else -1
                 self.twist.angular.z = turn_direction * self.search_turn_speed # Use search turn speed
            return
            
        # If still avoiding (no clear path found yet)
        if not was_avoiding: # Store recovery target if just entered AVOIDING
            self.recovery_target_x = self.target_x
            self.recovery_target_y = self.target_y
            self.pre_avoid_state = self.current_state
            self.current_state = ExplorationState.AVOIDING
            
        self.get_logger().info(f"Avoiding obstacle: F:{dist_f:.2f} FL:{dist_fl:.2f} FR:{dist_fr:.2f}")
        self.twist.linear.x = 0.0 # Stop forward motion while turning
        
        # Turn in direction with more space using a consistent speed
        turn_direction = 1 if dist_fl > dist_fr else -1
        # If distances are very close, prefer turning left slightly (arbitrary tie-break)
        if abs(dist_fl - dist_fr) < 0.1: 
            turn_direction = 1
            
        self.twist.angular.z = turn_direction * self.search_turn_speed # Use a moderate, consistent turn speed

    def get_sector_distances(self, msg: LaserScan):
        """Get minimum distances in front sectors using simple minimum."""
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)
        range_min_thresh = msg.range_min + 0.02
        range_max_thresh = msg.range_max - 0.02 # Add buffer for max range

        # Calculate indices for front sectors
        front_rad = math.radians(self.front_angle)
        front_side_rad = math.radians(self.front_side_angle)
        
        idx_front = int(front_rad / angle_increment)
        idx_side = int(front_side_rad / angle_increment)
        
        # Helper to get minimum valid distance in a slice
        def get_min_dist(readings):
            valid = [r for r in readings if range_min_thresh < r < range_max_thresh and math.isfinite(r)]
            return min(valid) if valid else range_max_thresh
        
        # Get front ranges (both positive and negative angles)
        front_ranges = ranges[:idx_front] + ranges[-idx_front:]
        front_left_ranges = ranges[idx_front:idx_side]
        front_right_ranges = ranges[-idx_side:-idx_front]
        
        dist_f = get_min_dist(front_ranges)
        dist_fl = get_min_dist(front_left_ranges)
        dist_fr = get_min_dist(front_right_ranges)

        return dist_f, dist_fl, dist_fr

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

    def timer_callback(self):
        """Check if exploration time is up."""
        try:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            if elapsed_time >= self.exploration_duration:
                self.get_logger().info(f"Exploration time ({self.exploration_duration}s) complete!")
                self.get_logger().info(f"Visited {len(self.visited_boxes)} boxes: {sorted(list(self.visited_boxes))}")
                self.stop_robot()
                self.timer.cancel()
                rclpy.shutdown()
                return
                
            # Log progress every 10 seconds
            if int(elapsed_time) % 10 == 0:
                self.get_logger().info(f"Time remaining: {self.exploration_duration - elapsed_time:.1f}s")
                
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")
            self.stop_robot()
            self.timer.cancel()
            rclpy.shutdown()

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def handle_turning(self):
        """Handle 180-degree turn."""
        # Calculate remaining angle to turn
        angle_diff = self.normalize_angle(self.turn_target_angle - self.theta_z)
        
        if abs(angle_diff) < 0.1:  # Turn complete
            self.get_logger().info("180-degree turn complete")
            self.current_state = ExplorationState.NAVIGATING
            if not self.set_next_target():
                self.stop_robot()
            self.twist.angular.z = 0.0  # Stop turning
            return
        
        # Execute turn
        self.twist.linear.x = 0.0
        turn_speed = min(self.max_angular_speed, max(0.5, abs(angle_diff)))  # Proportional turn speed
        self.twist.angular.z = turn_speed if angle_diff > 0 else -turn_speed

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = FastExplorerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        if node is not None:
            node.get_logger().info("Finalizing shutdown...")
            node.on_shutdown()
            
            try:
                if rclpy.ok():
                    for _ in range(10):
                        rclpy.spin_once(node, timeout_sec=0.05)
                        if node.shutdown_flag and not rclpy.ok():
                            break
            except Exception as e:
                print(f"Error during shutdown: {str(e)}")
            
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        print("ROS Cleanup Complete.")

if __name__ == '__main__':
    main()