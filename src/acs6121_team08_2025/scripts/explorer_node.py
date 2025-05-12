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
    INIT = auto()           # Initial state
    NAVIGATING = auto()     # Moving to target
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
        self.current_state = ExplorationState.NAVIGATING  # Start in navigation state
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
        
        # Navigation parameters
        self.position_tolerance = 0.2  # Increased for faster box transitions
        
        # Obstacle avoidance parameters
        self.critical_front_distance = 0.45  # Reduced for more direct paths
        self.warning_front_distance = 0.65
        
        # Speeds
        self.max_linear_speed = 0.26        # Maximum allowed linear velocity
        self.cautious_linear_speed = 0.15   # Reduced speed for obstacle avoidance
        self.max_angular_speed = 1.82       # Maximum allowed angular velocity
        self.turn_speed = 1.5               # Single turn speed for simplicity

        # LiDAR Sector Angles (degrees)
        self.front_angle = 25               # Wider front detection
        self.front_side_angle = 45          # Reduced side detection

        # Velocity message
        self.twist = Twist()
        
        self.get_logger().info("Starting exploration from center position!")
        self.set_next_target()

    def set_next_target(self):
        """Set the next target box to explore."""
        if len(self.visited_boxes) >= len(self.boxes_to_explore):
            self.get_logger().info("Exploration complete! All outer boxes visited.")
            self.stop_robot()
            return False

        # Find the closest unvisited box
        current_box = self.get_current_box()
        min_distance = float('inf')
        next_box = None

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

        # Calculate distance and direction to target
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.position_tolerance:
            self.set_next_target()
        else:
            self.move_to_target(dx, dy, distance)

    def move_to_target(self, dx, dy, distance):
        """Move directly towards target."""
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.theta_z
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Always move forward, adjust turning based on angle difference
        self.twist.linear.x = self.max_linear_speed
        
        # Proportional control for turning, max at 90 degrees
        turn_factor = min(abs(angle_diff) / (math.pi/2), 1.0)
        turn_direction = 1.0 if angle_diff > 0 else -1.0
        self.twist.angular.z = turn_direction * self.max_angular_speed * turn_factor

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
        """Handle obstacle avoidance during navigation."""
        if self.shutdown_flag:
            return

        dist_f, dist_fl, dist_fr = self.get_sector_distances(msg)

        # Check if we need to avoid obstacles
        if dist_f < self.critical_front_distance or dist_f < self.warning_front_distance:
            self.current_state = ExplorationState.AVOIDING
            self.handle_obstacle_avoidance(dist_f, dist_fl, dist_fr)
        elif self.current_state == ExplorationState.AVOIDING:
            self.current_state = ExplorationState.NAVIGATING
            self.update_navigation()

        self.cmd_vel_pub.publish(self.twist)

    def handle_obstacle_avoidance(self, dist_f, dist_fl, dist_fr):
        """Simple obstacle avoidance."""
        if dist_f < self.critical_front_distance:
            # Stop and turn away from obstacle
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.turn_speed if dist_fl > dist_fr else -self.turn_speed
        else:
            # Slow down and turn while moving
            self.twist.linear.x = self.cautious_linear_speed
            self.twist.angular.z = self.turn_speed if dist_fl > dist_fr else -self.turn_speed

    def get_sector_distances(self, msg: LaserScan):
        """Get minimum distances in front sectors."""
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)
        range_min_thresh = msg.range_min + 0.02
        range_max_thresh = msg.range_max

        # Calculate indices for front sectors
        front_rad = math.radians(self.front_angle)
        front_side_rad = math.radians(self.front_side_angle)
        
        idx_front = int(front_rad / angle_increment)
        idx_side = int(front_side_rad / angle_increment)
        
        # Get minimum distances in each sector
        front_ranges = ranges[:idx_front] + ranges[-idx_front:]
        front_left_ranges = ranges[idx_front:idx_side]
        front_right_ranges = ranges[-idx_side:-idx_front]
        
        # Filter valid readings
        valid_front = [r for r in front_ranges if range_min_thresh < r < range_max_thresh and math.isfinite(r)]
        valid_fl = [r for r in front_left_ranges if range_min_thresh < r < range_max_thresh and math.isfinite(r)]
        valid_fr = [r for r in front_right_ranges if range_min_thresh < r < range_max_thresh and math.isfinite(r)]
        
        dist_f = min(valid_front, default=range_max_thresh)
        dist_fl = min(valid_fl, default=range_max_thresh)
        dist_fr = min(valid_fr, default=range_max_thresh)

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