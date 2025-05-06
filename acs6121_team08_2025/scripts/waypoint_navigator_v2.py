#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.waypoints = [
            (-1.5, 1.5), (-0.5, 1.5), (0.5, 1.5), (1.5, 1.5), (1.5, 0.5),
            (1.5, -0.5), (1.5, -1.5), (0.5, -1.5), (-0.5, -1.5), (-1.5, -1.5),
            (-1.5, -0.5), (-1.5, 0.5),(0,0)
        ]
        self.current_waypoint_index = 0
        self.robot_position = (0.0, 0.0)
        self.timer = self.create_timer(8.0, self.navigate)
        self.get_logger().info("Waypoint Navigator Started")

    def odom_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def navigate(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = self.waypoints[self.current_waypoint_index]
        goal.pose.orientation.w = 1.0

        self.publisher_.publish(goal)
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}: {self.waypoints[self.current_waypoint_index]}")
        
        if self.is_waypoint_reached():
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached!")
            self.current_waypoint_index += 1

    def is_waypoint_reached(self, threshold=0.4):
        wp_x, wp_y = self.waypoints[self.current_waypoint_index]
        robot_x, robot_y = self.robot_position
        distance = ((wp_x - robot_x) ** 2 + (wp_y - robot_y) ** 2) ** 0.5
        return distance < threshold

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()