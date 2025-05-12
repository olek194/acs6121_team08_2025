#!/usr/bin/env python3
# Optimized exploration node for TurtleBot3 Waffle - Focus on Speed and Zone Coverage

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Added for position tracking
from sensor_msgs.msg import LaserScan
import math
import enum

class RobotState(enum.Enum):
    SEARCHING_FOR_WALL = 0
    ALIGNING_WITH_WALL = 1
    FOLLOWING_WALL = 2

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
        self.critical_front_distance = 0.40 # Keep for safety
        self.wall_detection_distance = 0.60 # Distance to trigger alignment
        self.target_wall_distance = 0.35    # Desired distance from wall
        self.wall_follow_tolerance = 0.10   # Allowed deviation from target distance
        self.side_safety_distance = 0.25    # Minimum allowed side distance

        # Speeds
        self.search_linear_speed = 0.25
        self.wall_following_speed = 0.20
        self.alignment_angular_speed = 0.8
        self.max_angular_speed = 1.5       # Reduced slightly for smoother following
        self.wall_follow_kp = 2.5          # Proportional gain for wall distance control

        # LiDAR Sector Angles (degrees)
        self.front_angle = 15
        self.front_side_angle = 45
        self.side_angle_start = 50 # Used for left wall following
        self.side_angle_end = 130 # Used for left wall following

        # State Machine
        self.state = RobotState.SEARCHING_FOR_WALL
        self.get_logger().info(f"Starting in state: {self.state.name}")

        self.get_logger().info(f"'{self.get_name()}' node initialized for wall following.")
        
        # No longer starting immediately, wait for first lidar scan
        # self.start_moving() # Removed

    def start_moving(self):
        """DEPRECATED for wall following - logic is in lidar_callback now."""
        # This function is no longer the primary way to start movement.
        # The state machine in lidar_callback handles initial movement.
        self.get_logger().warn("start_moving() called, but movement is state-driven.")
        pass # Do nothing, handled by state machine

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

    def lidar_callback(self, msg: LaserScan):
        """Main control loop implementing the wall following state machine."""
        if self.is_stopped or self.shutdown_flag:
            return

        dist_f, dist_fl, dist_fr, dist_l, dist_r = self.get_sector_distances(msg)

        # Log distances and current state
        self.get_logger().debug(
            f"State: {self.state.name} | Distances (m) - F:{dist_f:.2f}, FL:{dist_fl:.2f}, FR:{dist_fr:.2f}, L:{dist_l:.2f}, R:{dist_r:.2f}"
        )

        target_linear_x = 0.0
        target_angular_z = 0.0

        # --- State Machine Logic ---

        if self.state == RobotState.SEARCHING_FOR_WALL:
            # Move forward until a wall is detected in front
            if dist_f < self.wall_detection_distance:
                # Wall detected, stop and switch to alignment
                self.get_logger().info(f"Wall detected at {dist_f:.2f}m. Switching to ALIGNING_WITH_WALL.")
                self.state = RobotState.ALIGNING_WITH_WALL
                target_linear_x = 0.0
                target_angular_z = self.alignment_angular_speed # Start turning left
            else:
                # Keep searching
                target_linear_x = self.search_linear_speed
                target_angular_z = 0.0 # Go straight

        elif self.state == RobotState.ALIGNING_WITH_WALL:
            # Turn left until roughly parallel to the wall (using left sensor)
            # We aim to have the left sensor see the wall at the target distance
            # and the front sensor clear enough to start moving along the wall.
            if dist_l < (self.target_wall_distance + self.wall_follow_tolerance) and dist_f > self.critical_front_distance * 1.5:
                 # Good enough alignment and front is clear, start following
                self.get_logger().info(f"Alignment achieved (Left dist: {dist_l:.2f}m). Switching to FOLLOWING_WALL.")
                self.state = RobotState.FOLLOWING_WALL
                target_linear_x = self.wall_following_speed # Start moving forward slowly
                target_angular_z = 0.0 # Correct angle later in FOLLOWING state
            elif dist_f < self.critical_front_distance:
                 # Too close to wall while turning, maybe turn sharper right temporarily? Or just stop turning?
                 self.get_logger().warn("Too close to front wall during alignment. Stopping turn.")
                 target_linear_x = 0.0
                 target_angular_z = 0.0 # Stop turning to avoid collision
                 # Consider a small backup or right turn here if it gets stuck
            else:
                # Continue turning left
                target_linear_x = 0.0
                target_angular_z = self.alignment_angular_speed

        elif self.state == RobotState.FOLLOWING_WALL:
            # Follow the left wall, maintaining target distance
            if dist_f < self.critical_front_distance:
                # Obstacle directly ahead (e.g., corner), turn right sharply
                self.get_logger().warn(f"Obstacle ahead during wall following (Dist: {dist_f:.2f}m). Turning right.")
                target_linear_x = 0.0 # Stop forward motion
                target_angular_z = -self.max_angular_speed
            elif dist_fl < self.target_wall_distance:
                 # Inner corner detected by front-left sensor, turn right more gradually
                 self.get_logger().debug(f"Inner corner detected (FL: {dist_fl:.2f}m). Nudging right.")
                 target_linear_x = self.wall_following_speed * 0.5 # Slow down slightly
                 # Reduce turn based on how close FL is
                 error_fl = self.target_wall_distance - dist_fl
                 target_angular_z = -self.max_angular_speed * (error_fl / self.target_wall_distance) * 0.8 # Turn right
            elif dist_l > self.target_wall_distance * 2.5: # Lost the wall completely (e.g. large opening / outer corner)
                 self.get_logger().info(f"Lost left wall (Dist: {dist_l:.2f}m). Re-searching.")
                 self.state = RobotState.SEARCHING_FOR_WALL # Go back to searching
                 target_linear_x = self.search_linear_speed * 0.5 # Move forward slowly while searching
                 target_angular_z = 0.0
                 # Alternative: could try a specific turn-left maneuver here to find wall again

            else:
                # Wall detected on left, apply proportional control for distance
                error = self.target_wall_distance - dist_l
                target_angular_z = self.wall_follow_kp * error

                # Ensure safety distance is maintained
                if dist_l < self.side_safety_distance:
                    self.get_logger().warn(f"Too close to left wall ({dist_l:.2f}m < {self.side_safety_distance}m). Forcing right turn.")
                    target_angular_z = -self.max_angular_speed * 0.6 # Force a moderate turn right

                # Clamp angular velocity
                target_angular_z = max(-self.max_angular_speed, min(target_angular_z, self.max_angular_speed))

                # Maintain forward speed
                target_linear_x = self.wall_following_speed


        # Log final command
        self.get_logger().debug(
            f"Command: linear={target_linear_x:.2f} m/s, "
            f"angular={target_angular_z:.2f} rad/s"
        )

        # Apply velocities
        self.twist.linear.x = target_linear_x
        self.twist.angular.z = target_angular_z
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