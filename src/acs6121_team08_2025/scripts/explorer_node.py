#!/usr/bin/env python3
# Exploration node with perimeter following logic

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time # Import time for MOVING_MIDDLE duration
from enum import Enum, auto

class ExplorationState(Enum):
    FINDING_SPACE = auto()       # Initial spin to find clear path
    MOVING_TO_BOX = auto()       # Moving to target box
    AVOIDING = auto()            # Avoiding obstacles
    ROTATING_TO_BOX = auto()     # Rotating to face next box
    STOPPED = auto()             # Exploration complete

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
        self.position_tolerance = 0.2 # Tolerance for reaching target box
        self.angle_tolerance = 0.1    # Tolerance for rotation alignment
        self.obstacle_threshold = 0.75 # Distance to trigger avoidance
        self.clear_threshold = 0.8    # Distance to consider path clear
        self.open_space_threshold = 1.0 # For initial space finding
        
        # Speeds
        self.max_linear_speed = 0.26  # Max straight speed
        self.cautious_linear_speed = 0.15 # Used when near target or after avoiding
        self.max_angular_speed = 1.82
        self.search_turn_speed = 1.82 # Speed for FINDING_SPACE and AVOIDING turns
        self.rotate_turn_speed = 0.5  # Speed for rotating to face next box
        
        # LiDAR Sector Angles (degrees)
        self.front_angle = 15         # Narrower front angle for obstacle detection
        self.side_angle = 45          # Side angles for turning direction
        
        # State Machine
        self.current_state = ExplorationState.FINDING_SPACE
        self.target_angle = 0.0
        
        # Path planning
        self.target_box = 1 # Start with Box 1
        self.boxes_to_explore = [1, 2, 3, 4, 8, 12, 16, 15, 14, 13, 5]  # Updated path sequence
        self.visited_boxes = set()
        self.current_path_index = 0  # Track current position in path
        
        # Box positions
        self.box_positions = {
            1: (-1.5, 1.5),   2: (-0.5, 1.5),   3: (0.5, 1.5),   4: (1.5, 1.5),
            5: (-1.5, 0.5),   6: (-0.5, 0.5),   7: (0.5, 0.5),   8: (1.5, 0.5),
            9: (-1.5, -0.5), 10: (-0.5, -0.5), 11: (0.5, -0.5), 12: (1.5, -0.5),
            13: (-1.5, -1.5), 14: (-0.5, -1.5), 15: (0.5, -1.5), 16: (1.5, -1.5)
        }
        
        # Box navigation
        self.box_neighbors = {
            1: [2, 5],     2: [1, 3, 6],     3: [2, 4, 7],     4: [3, 8],
            5: [1, 6, 9],  6: [2, 5, 7, 10], 7: [3, 6, 8, 11], 8: [4, 7, 12],
            9: [5, 10, 13],10: [6, 9, 11, 14],11: [7, 10, 12, 15],12: [8, 11, 16],
            13: [9, 14],   14: [10, 13, 15], 15: [11, 14, 16], 16: [12, 15]
        }
        self.target_x, self.target_y = self.box_positions[self.target_box]
        self.rerouting = False  # Flag to indicate if we're following an alternate path
        self.alternate_path = []  # Store alternate path when obstacle detected
        
        # Velocity message
        self.twist = Twist()
        
        # Timer for 90-second exploration
        self.start_time = self.get_clock().now()
        self.exploration_duration = 90.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Starting exploration with path: {self.boxes_to_explore}")

    # --- State Transition Helper ---
    def change_state(self, new_state):
        """Helper to change state, log, handle oscillation, potentially reset timers."""
        if self.current_state != new_state:
            self.get_logger().info(f"Changing state from {self.current_state.name} to {new_state.name}")
            previous_state = self.current_state # Store before overwriting
            self.current_state = new_state

            # Reset state timer if transitioning to MOVING_MIDDLE
            if new_state == ExplorationState.MOVING_TO_BOX:
                self.state_start_time = time.time()
                
            # Reset twist command when changing to *most* new states (except maybe AVOIDING where immediate turn is set)
            if new_state != ExplorationState.AVOIDING: # Avoid resetting twist if avoiding handler sets it immediately
                 self.twist.linear.x = 0.0
                 self.twist.angular.z = 0.0

    # --- State Handling Methods ---

    def handle_finding_space(self, dist_f):
        """Spin until front is clear."""
        if dist_f > self.open_space_threshold:
            # Calculate initial heading to first box
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            self.target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(self.target_angle - self.theta_z)
            
            if abs(angle_diff) < self.angle_tolerance:
                self.change_state(ExplorationState.MOVING_TO_BOX)
                self.twist.linear.x = self.max_linear_speed
                self.twist.angular.z = 0.0
            else:
                self.change_state(ExplorationState.ROTATING_TO_BOX)
                self.twist.linear.x = 0.0
                self.twist.angular.z = max(-self.rotate_turn_speed, 
                                         min(self.rotate_turn_speed, angle_diff))
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.search_turn_speed

    def handle_moving_to_box(self, dist_f):
        """Move towards the current target box."""
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_heading = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_heading - self.theta_z)

        if distance < self.position_tolerance:
            # Reached target box, rotate to face next box
            self.get_logger().info(f"Reached box {self.target_box}")
            next_box = self.get_next_target_box()
            if next_box:
                next_x, next_y = self.box_positions[next_box]
                self.target_angle = math.atan2(next_y - self.y, next_x - self.x)
                self.change_state(ExplorationState.ROTATING_TO_BOX)
            else:
                self.change_state(ExplorationState.STOPPED)
        elif dist_f < self.obstacle_threshold:
            self.get_logger().info(f"Obstacle detected while moving to box {self.target_box}")
            self.change_state(ExplorationState.AVOIDING)
        else:
            # Move towards target box
            if abs(angle_diff) > self.angle_tolerance * 2:  # Wider tolerance during movement
                self.twist.linear.x = self.cautious_linear_speed
                self.twist.angular.z = max(-0.8, min(0.8, angle_diff))
                self.get_logger().debug(f"Correcting angle: diff={angle_diff:.2f}, x={self.twist.linear.x:.2f}, z={self.twist.angular.z:.2f}")
            else:
                self.twist.linear.x = self.max_linear_speed
                self.twist.angular.z = max(-0.3, min(0.3, angle_diff))

    def handle_avoiding(self, dist_f, dist_fl, dist_fr):
        """Stop and turn away from obstacle."""
        if dist_f > self.clear_threshold:
            self.get_logger().info("Obstacle cleared. Finding alternate path.")
            current_box = self.get_current_box()
            if current_box and not self.rerouting:
                # Try to find alternate path to target
                self.alternate_path = self.find_alternate_path(current_box, self.target_box)
                if self.alternate_path:
                    self.rerouting = True
                    self.alternate_path.pop(0)  # Remove current box
                    self.update_target_box()
                    self.change_state(ExplorationState.MOVING_TO_BOX)
                else:
                    self.get_logger().warning(f"No alternate path found to box {self.target_box}")
            self.twist.linear.x = self.cautious_linear_speed
            self.change_state(ExplorationState.MOVING_TO_BOX)
        else:
            self.twist.linear.x = 0.0
            turn_direction = 1 if dist_fl > dist_fr else -1
            if abs(dist_fl - dist_fr) < 0.1: turn_direction = 1
            self.twist.angular.z = turn_direction * self.search_turn_speed

    def handle_rotating_to_box(self):
        """Rotate to face the next target box."""
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        current_target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(current_target_angle - self.theta_z)
        
        self.get_logger().debug(f"Rotating: target={math.degrees(current_target_angle):.1f}°, current={math.degrees(self.theta_z):.1f}°, diff={math.degrees(angle_diff):.1f}°")
        
        if abs(angle_diff) < self.angle_tolerance:
            self.change_state(ExplorationState.MOVING_TO_BOX)
            self.twist.linear.x = self.max_linear_speed
            self.twist.angular.z = 0.0
            self.get_logger().info("Rotation complete, moving to box")
        else:
            self.twist.linear.x = 0.0
            # Use proportional control for smoother rotation
            rotation_speed = max(-self.rotate_turn_speed, 
                               min(self.rotate_turn_speed, angle_diff))
            self.twist.angular.z = rotation_speed
            self.get_logger().debug(f"Rotating with speed: {rotation_speed:.2f}")

    # --- Callbacks and Helpers ---

    def lidar_callback(self, msg: LaserScan):
        """Handle LiDAR data and state transitions."""
        if self.shutdown_flag:
            return

        dist_f, dist_fl, dist_fr = self.get_sector_distances(msg)
        current_state_copy = self.current_state
        
        # Log current state and distances periodically
        if hasattr(self, 'last_log_time') and time.time() - self.last_log_time < 1.0:
            pass
        else:
            self.last_log_time = time.time()
            self.get_logger().debug(f"State: {current_state_copy.name}, Distances - F: {dist_f:.2f}, FL: {dist_fl:.2f}, FR: {dist_fr:.2f}")
        
        if current_state_copy == ExplorationState.FINDING_SPACE:
            self.handle_finding_space(dist_f)
        elif current_state_copy == ExplorationState.MOVING_TO_BOX:
            self.handle_moving_to_box(dist_f)
        elif current_state_copy == ExplorationState.AVOIDING:
            self.handle_avoiding(dist_f, dist_fl, dist_fr)
        elif current_state_copy == ExplorationState.ROTATING_TO_BOX:
            self.handle_rotating_to_box()
        elif current_state_copy == ExplorationState.STOPPED:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            
        if self.current_state == current_state_copy:
            self.cmd_vel_pub.publish(self.twist)
            
        # Log movement commands periodically
        if hasattr(self, 'last_cmd_log_time') and time.time() - self.last_cmd_log_time < 1.0:
            pass
        else:
            self.last_cmd_log_time = time.time()
            self.get_logger().debug(f"Movement - Linear: {self.twist.linear.x:.2f}, Angular: {self.twist.angular.z:.2f}")

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
                 
                 # If we're following an alternate path and reached a box, update target
                 if self.rerouting:
                     if not self.alternate_path:  # Reached end of alternate path
                         self.rerouting = False
                         self.update_target_box()  # Resume normal sequence
                     else:
                         self.update_target_box()  # Continue on alternate path
                 else:
                     # Check if we should move to next box in sequence
                     self.update_target_box()

        # State logic dependent on odometry
        current_state_copy = self.current_state
        
        if current_state_copy == ExplorationState.MOVING_TO_BOX:
            self.handle_moving_to_box(dist_f)
        elif current_state_copy == ExplorationState.ROTATING_TO_BOX:
            self.handle_rotating_to_box()
            
        # Publish twist if state didn't change during odom processing
        if self.current_state == current_state_copy:
             self.cmd_vel_pub.publish(self.twist)

    def timer_callback(self):
        """Check if exploration time is up."""
        if self.shutdown_flag:
            return
            
        try:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            log_interval = 10 # seconds
            # Log state less frequently to avoid spam
            if elapsed_time > 1 and int(elapsed_time) % log_interval == 0 and abs(elapsed_time - int(elapsed_time)) < 0.15:
                 self.get_logger().info(f"Time: {elapsed_time:.1f}/{self.exploration_duration:.1f}s. State: {self.current_state.name}. Visited: {len(self.visited_boxes)}/{len(self.boxes_to_explore)}.")

            if elapsed_time >= self.exploration_duration:
                self.get_logger().info(f"Exploration time ({self.exploration_duration}s) complete!")
                self.get_logger().info(f"Visited {len(self.visited_boxes)} boxes: {sorted(list(self.visited_boxes))}")
                self.stop_robot()
                self.shutdown_flag = True 
                if self.timer: self.timer.cancel()
                # Use a short delay before shutting down ROS to allow stop command to publish
                self.create_timer(0.5, self.initiate_shutdown)
                return 
                
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}", exc_info=True)
            self.stop_robot()
            self.shutdown_flag = True
            if self.timer: self.timer.cancel()
            self.initiate_shutdown() 
            
    def initiate_shutdown(self):
        """Callback to actually call rclpy.shutdown after a short delay."""
        # Cancel this timer itself
        caller_timer = self._timers[-1] # Assumes this is the last timer created
        if caller_timer is not None and not caller_timer.is_canceled():
             caller_timer.cancel()
             
        if rclpy.ok() and not self.shutdown_flag:
             # Check shutdown_flag again as it might be set by other means between timer creation and execution
            self.get_logger().info("Timer initiating ROS shutdown.")
            self.shutdown_flag = True # Ensure flag is set
            self.stop_robot() # Send stop again just in case
            rclpy.shutdown()
        elif not rclpy.ok():
             self.get_logger().warning("Shutdown initiator called but RCLPY not OK.")
        elif self.shutdown_flag:
             self.get_logger().info("Shutdown initiator called but shutdown already in progress.")

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

    def get_next_target_box(self):
        """Get the next box in the exploration sequence."""
        if self.current_path_index < len(self.boxes_to_explore) - 1:
            self.current_path_index += 1
            return self.boxes_to_explore[self.current_path_index]
        return None

    def find_alternate_path(self, current_box, target_box):
        """Find alternate path to target box using neighbors when obstacle detected."""
        visited = set()
        queue = [(current_box, [current_box])]
        
        while queue:
            (box, path) = queue.pop(0)
            for neighbor in self.box_neighbors[box]:
                if neighbor not in visited:
                    if neighbor == target_box:
                        return path + [neighbor]
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None  # No path found

    def update_target_box(self):
        """Update target box and position."""
        if self.rerouting and self.alternate_path:
            # Follow alternate path
            self.target_box = self.alternate_path.pop(0)
        else:
            # Follow normal sequence
            next_box = self.get_next_target_box()
            if next_box:
                self.target_box = next_box
            else:
                self.get_logger().info("Completed exploration sequence!")
                self.stop_robot()
                return False
                
        self.target_x, self.target_y = self.box_positions[self.target_box]
        return True

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