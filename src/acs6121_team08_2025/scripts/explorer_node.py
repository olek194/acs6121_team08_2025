#!/usr/bin/env python3
# Exploration node with perimeter following logic

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from enum import Enum, auto

class ExplorationState(Enum):
    FINDING_SPACE = auto()       # Initial spin to find clear path
    GOING_TO_FIRST_BOX = auto() # Move to the initial target box
    ALIGNING_PERIMETER = auto() # Turn 90deg left after reaching first box
    FOLLOWING_PERIMETER = auto() # Go straight along the perimeter
    AVOIDING = auto()            # Stop and turn away from obstacle

class PatternExplorerNode(Node):

    def __init__(self):
        super().__init__("pattern_explorer_node")

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
        self.arena_size_x = 4.0
        self.arena_size_y = 4.0
        self.box_size = 1.0
        
        # Robot physical parameters
        self.robot_radius = 0.25
        
        # Navigation parameters
        self.position_tolerance = 0.2 # Tolerance for reaching first box
        self.obstacle_threshold = 0.5 # Distance to trigger avoidance
        self.clear_threshold = 0.6    # Distance to consider obstacle cleared
        self.open_space_threshold = 1.0 # For initial space finding
        
        # Speeds
        self.max_linear_speed = 0.26
        self.cautious_linear_speed = 0.15 # Used when exiting avoidance
        self.max_angular_speed = 1.82
        self.search_turn_speed = 0.8     # Speed for FINDING_SPACE and AVOIDING turns
        self.align_turn_speed = 0.5      # Speed for 90-degree alignment turn
        
        # LiDAR Sector Angles (degrees) - Simplified
        self.front_angle = 15            # Narrower front angle for obstacle detection
        self.side_angle = 45             # Side angles for turning direction
        
        # State Machine
        self.current_state = ExplorationState.FINDING_SPACE
        self.target_angle = 0.0          # Used for alignment turn
        self.initial_box_reached = False
        
        # Path planning (only for first box and tracking)
        self.target_box = 1 # Start with Box 1
        self.boxes_to_explore = [1,2,3,4,5,8,9,12,13,14,15,16]
        self.visited_boxes = set()
        
        # Box positions (only needed for the first box target)
        self.box_positions = {
            1: (-1.5, 1.5),   2: (-0.5, 1.5),   3: (0.5, 1.5),   4: (1.5, 1.5),
            5: (-1.5, 0.5),   6: (-0.5, 0.5),   7: (0.5, 0.5),   8: (1.5, 0.5),
            9: (-1.5, -0.5), 10: (-0.5, -0.5), 11: (0.5, -0.5), 12: (1.5, -0.5),
            13: (-1.5, -1.5), 14: (-0.5, -1.5), 15: (0.5, -1.5), 16: (1.5, -1.5)
        }
        self.target_x, self.target_y = self.box_positions[self.target_box]
        
        # Velocity message
        self.twist = Twist()
        
        # Timer for 90-second exploration
        self.start_time = self.get_clock().now()
        self.exploration_duration = 90.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Starting exploration pattern. Initial target: Box {self.target_box}")

    # --- State Handling Methods ---

    def handle_finding_space(self, dist_f):
        """Spin until front is clear."""
        if dist_f > self.open_space_threshold:
            self.get_logger().info("Found open space. Proceeding to first box.")
            self.current_state = ExplorationState.GOING_TO_FIRST_BOX
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.search_turn_speed

    def handle_going_to_first_box(self):
        """Move towards the first target box."""
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < self.position_tolerance:
            self.get_logger().info(f"Reached vicinity of Box {self.target_box}. Aligning for perimeter follow.")
            self.initial_box_reached = True
            self.current_state = ExplorationState.ALIGNING_PERIMETER
            # Set target for 90-degree left turn
            self.target_angle = self.normalize_angle(self.theta_z + math.pi / 2.0)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0 # Stop briefly before turning
        else:
            # Use simplified move_to_target logic
            target_heading = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_heading - self.theta_z)
            
            if abs(angle_diff) < 0.3:
                self.twist.linear.x = self.max_linear_speed
                self.twist.angular.z = max(-0.3, min(0.3, angle_diff))
            else:
                self.twist.linear.x = 0.0 # Turn first
                self.twist.angular.z = max(-0.8, min(0.8, angle_diff))

    def handle_aligning_perimeter(self):
        """Execute 90-degree left turn."""
        angle_diff = self.normalize_angle(self.target_angle - self.theta_z)
        
        if abs(angle_diff) < 0.1: # Turn complete
            self.get_logger().info("Alignment turn complete. Following perimeter.")
            self.current_state = ExplorationState.FOLLOWING_PERIMETER
            self.twist.angular.z = 0.0
            self.twist.linear.x = self.max_linear_speed # Start moving forward
        else:
            # Execute turn
            self.twist.linear.x = 0.0
            turn_speed = self.align_turn_speed
            self.twist.angular.z = turn_speed if angle_diff > 0 else -turn_speed

    def handle_following_perimeter(self, dist_f):
        """Move straight, check for obstacles."""
        if dist_f < self.obstacle_threshold:
            self.get_logger().info(f"Obstacle detected (dist: {dist_f:.2f}m). Switching to AVOIDING.")
            self.current_state = ExplorationState.AVOIDING
            self.twist.linear.x = 0.0 # Stop immediately
            # Angular velocity will be set in the next call to AVOIDING handler
        else:
            self.twist.linear.x = self.max_linear_speed
            self.twist.angular.z = 0.0 # Go straight

    def handle_avoiding(self, dist_f, dist_fl, dist_fr):
        """Stop and turn away from obstacle."""
        if dist_f > self.clear_threshold:
            self.get_logger().info("Obstacle cleared. Resuming perimeter following.")
            self.current_state = ExplorationState.FOLLOWING_PERIMETER
            self.twist.linear.x = self.cautious_linear_speed # Start moving cautiously
            self.twist.angular.z = 0.0
        else:
            # Keep turning away
            self.twist.linear.x = 0.0
            turn_direction = 1 if dist_fl > dist_fr else -1
            if abs(dist_fl - dist_fr) < 0.1:
                turn_direction = 1 # Tie-breaker: turn left
            self.twist.angular.z = turn_direction * self.search_turn_speed
            self.get_logger().debug(f"Avoiding. Turning {'left' if turn_direction > 0 else 'right'}. F:{dist_f:.2f} FL:{dist_fl:.2f} FR:{dist_fr:.2f}")

    # --- Callbacks and Helpers ---

    def lidar_callback(self, msg: LaserScan):
        """Handle LiDAR data and state transitions."""
        if self.shutdown_flag:
            return

        dist_f, dist_fl, dist_fr = self.get_sector_distances(msg)

        # State-specific logic triggered by LiDAR
        if self.current_state == ExplorationState.FINDING_SPACE:
            self.handle_finding_space(dist_f)
        elif self.current_state == ExplorationState.FOLLOWING_PERIMETER:
            self.handle_following_perimeter(dist_f)
        elif self.current_state == ExplorationState.AVOIDING:
            self.handle_avoiding(dist_f, dist_fl, dist_fr)
        elif self.current_state == ExplorationState.GOING_TO_FIRST_BOX:
             # Check for obstacles even when going to the first box
            if dist_f < self.obstacle_threshold:
                 self.get_logger().warning("Obstacle detected while going to first box! Switching to AVOIDING.")
                 self.current_state = ExplorationState.AVOIDING
                 # We will lose the target box, but prioritize safety
                 self.twist.linear.x = 0.0
            # Actual movement logic is in odom_callback based update

        # Note: GOING_TO_FIRST_BOX and ALIGNING_PERIMETER primarily update based on odometry/internal logic
        # They don't directly react to lidar unless an obstacle forces AVOIDING state.

        self.cmd_vel_pub.publish(self.twist)

    def odom_callback(self, msg: Odometry):
        """Update robot pose and handle state logic based on position/orientation."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.theta_z = math.atan2(siny_cosp, cosy_cosp)

        # Update visited boxes based on current position
        current_box = self.get_current_box()
        if current_box is not None and current_box in self.boxes_to_explore and current_box not in self.visited_boxes:
             # Check distance to box center - only mark if reasonably close
             box_x, box_y = self.box_positions[current_box]
             dist_to_box_center = math.sqrt((self.x - box_x)**2 + (self.y - box_y)**2)
             if dist_to_box_center < (self.box_size / 2.0): # Mark if within half the box size from center
                 self.visited_boxes.add(current_box)
                 self.get_logger().info(f"Entered and visited Box {current_box}. Total visited: {len(self.visited_boxes)}/{len(self.boxes_to_explore)} - {sorted(list(self.visited_boxes))}")
                 # Check completion condition
                 if len(self.visited_boxes) >= len(self.boxes_to_explore):
                     self.get_logger().info("All target boxes visited!")
                     self.stop_robot()
                     # Potentially shutdown ROS here if needed
                     # rclpy.shutdown()

        # State logic dependent on odometry
        if self.current_state == ExplorationState.GOING_TO_FIRST_BOX:
            self.handle_going_to_first_box()
        elif self.current_state == ExplorationState.ALIGNING_PERIMETER:
            self.handle_aligning_perimeter()

    def timer_callback(self):
        """Check if exploration time is up."""
        if self.shutdown_flag: # Check if shutdown already initiated
            return
            
        try:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            # Log progress every 10 seconds
            # Check elapsed_time > 1 to avoid spamming at start
            if elapsed_time > 1 and int(elapsed_time) % 10 == 0 and abs(elapsed_time - int(elapsed_time)) < 0.15:
                 self.get_logger().info(f"Time: {elapsed_time:.1f}/{self.exploration_duration:.1f}s. State: {self.current_state.name}. Visited: {len(self.visited_boxes)}/{len(self.boxes_to_explore)}.")

            if elapsed_time >= self.exploration_duration:
                self.get_logger().info(f"Exploration time ({self.exploration_duration}s) complete!")
                self.get_logger().info(f"Visited {len(self.visited_boxes)} boxes: {sorted(list(self.visited_boxes))}")
                self.stop_robot()
                self.shutdown_flag = True # Signal shutdown
                self.timer.cancel()
                # Initiate shutdown sequence cleanly
                # Allow some time for stop command to publish
                shutdown_timer = self.create_timer(0.5, self.initiate_shutdown)
                return
                
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")
            self.stop_robot()
            self.shutdown_flag = True
            self.timer.cancel()
            self.initiate_shutdown() # Attempt shutdown
            
    def initiate_shutdown(self):
        """Callback to actually call rclpy.shutdown after a short delay."""
        if rclpy.ok():
            self.get_logger().info("Timer initiating ROS shutdown.")
            rclpy.shutdown()
        # Cancel this timer itself if it hasn't been destroyed yet
        if hasattr(self, 'shutdown_timer') and self.shutdown_timer:
             try:
                 self.shutdown_timer.cancel()
             except Exception as e:
                  self.get_logger().warning(f"Could not cancel shutdown timer: {e}")

    def get_current_box(self):
        """Calculate which box the robot is in. Returns box number or None."""
        # Check if within arena bounds approximately
        half_x = self.arena_size_x / 2.0
        half_y = self.arena_size_y / 2.0
        if not (-half_x < self.x < half_x and -half_y < self.y < half_y):
             # self.get_logger().warning(f"Position ({self.x:.2f}, {self.y:.2f}) outside arena bounds.")
             return None # Outside the known arena
             
        # Normalize coordinates to box grid (origin at top-left)
        col = int((self.x + half_x) / self.box_size)
        row = int((half_y - self.y) / self.box_size) # Y is inverted
        
        # Ensure row/col are within 0-3 range
        col = max(0, min(3, col))
        row = max(0, min(3, row))
        
        # Calculate box number (1-16)
        box = row * 4 + col + 1
        return box

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_sector_distances(self, msg: LaserScan):
        """Get minimum distances in front and side sectors."""
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)
        range_min_thresh = msg.range_min + 0.02
        range_max_thresh = msg.range_max - 0.02

        # Calculate indices
        front_rad = math.radians(self.front_angle)
        side_rad = math.radians(self.side_angle)
        
        idx_front = int(front_rad / angle_increment)
        idx_side = int(side_rad / angle_increment)
        
        def get_min_dist(readings):
            valid = [r for r in readings if range_min_thresh < r < range_max_thresh and math.isfinite(r)]
            return min(valid) if valid else range_max_thresh
        
        # Front sector (narrower)
        front_ranges = ranges[:idx_front] + ranges[-idx_front:]
        # Front-Left sector (e.g., 15 to 45 degrees)
        front_left_ranges = ranges[idx_front:idx_side]
        # Front-Right sector (e.g., -15 to -45 degrees)
        front_right_ranges = ranges[-idx_side:-idx_front]
        
        dist_f = get_min_dist(front_ranges)
        dist_fl = get_min_dist(front_left_ranges)
        dist_fr = get_min_dist(front_right_ranges)

        return dist_f, dist_fl, dist_fr

    def stop_robot(self):
        """Stop the robot."""
        if self.shutdown_flag: # Don't spam stop if already shutting down
            return
        self.get_logger().info("Stopping robot...")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def on_shutdown(self):
        """Handle shutdown."""
        if not self.shutdown_flag:
            self.get_logger().info("Node shutting down externally. Stopping robot...")
            self.stop_robot()
            self.shutdown_flag = True
            if self.timer: self.timer.cancel()
            # If external shutdown (e.g. Ctrl+C), rclpy.shutdown() is likely handled elsewhere

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = PatternExplorerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Unexpected error: {str(e)}", exc_info=True)
    finally:
        if node is not None:
            node.get_logger().info("Finalizing shutdown sequence...")
            node.on_shutdown() # Ensure robot stops
            node.destroy_node()

        # Check if rclpy is still ok before shutting down (might have been shut down by timer)
        if rclpy.ok():
             rclpy.shutdown()
        print("ROS Cleanup Potentially Complete.") # Might print before shutdown finishes

if __name__ == '__main__':
    main()