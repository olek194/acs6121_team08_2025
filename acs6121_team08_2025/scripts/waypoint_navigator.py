#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
#from rclpy.action.client import GoalCancelledError
from nav_msgs.msg import Odometry
import time
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.initial_pose_captured = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0

        self.relative_waypoints = [
            (-1.5, 1.5), (-0.5, 1.5), (0.5, 1.5), (1.5, 1.5),
            (1.5, 0.5), (1.5, -0.5), (1.5, -1.5), (0.5, -1.5),
            (-0.5, -1.5), (-1.5, -1.5), (-1.5, -0.5), (-1.5, 0.5),
            (0.0, 0.0)
        ]

        self.absolute_waypoints = []
        self.current_waypoint_index = 0
        self.navigation_active = False

        self.last_feedback_time = None
        self.last_distance_remaining = None
        self.stuck_detected = False
        self.recovery_mode = False

        self.goal_handle = None
        self.stuck_goal = None  # Save the goal to retry later

        self.get_logger().info("Waiting for initial odometry...")

    def odom_callback(self, msg):
        # Always update robot position
        self.current_robot_x = msg.pose.pose.position.x
        self.current_robot_y = msg.pose.pose.position.y

        if not self.initial_pose_captured:
            self.initial_x = self.current_robot_x
            self.initial_y = self.current_robot_y
            self.initial_pose_captured = True
            self.get_logger().info(f"Captured initial position: ({self.initial_x:.2f}, {self.initial_y:.2f})")

            self.absolute_waypoints = [
                (self.initial_x + dx, self.initial_y + dy) for (dx, dy) in self.relative_waypoints
            ]

            self.send_next_goal()

    def send_next_goal(self, target=None):
        if not self.initial_pose_captured:
            self.get_logger().warn('Initial pose not captured yet.')
            return

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        if target is None:
            if self.current_waypoint_index >= len(self.absolute_waypoints):
                self.get_logger().info('All waypoints completed!')
                return
            waypoint = self.absolute_waypoints[self.current_waypoint_index]
        else:
            waypoint = target

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint[0]
        goal.pose.pose.position.y = waypoint[1]
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal: ({waypoint[0]:.2f}, {waypoint[1]:.2f})')
        self.navigation_active = True

        self.send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining

        current_time = time.time()

        if self.last_feedback_time is None:
            self.last_feedback_time = current_time
            self.last_distance_remaining = distance_remaining
            return

        # Check stuck
        if abs(distance_remaining - self.last_distance_remaining) < 0.05:
            if current_time - self.last_feedback_time > 5.0 and not self.recovery_mode:
                self.get_logger().warn('Robot seems stuck! Initiating recovery...')
                self.recover()
                return
        else:
            self.last_feedback_time = current_time
            self.last_distance_remaining = distance_remaining

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.navigation_active = False
            return

        self.get_logger().info('Goal accepted.')
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        if self.stuck_detected:
            return  # Already in recovery

        result = future.result().result
        self.get_logger().info('Goal completed according to server.')

        if not self.recovery_mode:
            if self.is_close_to_goal(self.absolute_waypoints[self.current_waypoint_index]):
                self.get_logger().info('Goal reached within margin (+/-0.15m)!')
                self.current_waypoint_index += 1
                self.navigation_active = False
                self.last_feedback_time = None
                self.last_distance_remaining = None
                self.send_next_goal()
            else:
                self.get_logger().warn('Goal reported as complete but outside margin. Re-attempting...')
                self.send_next_goal()  # retry current goal

        else:
            # If we were recovering (going back home), now retry the stuck goal
            self.get_logger().info('Back to (0,0). Retrying stuck goal.')
            self.recovery_mode = False
            self.stuck_detected = False
            self.send_next_goal(target=self.stuck_goal)

    def is_close_to_goal(self, goal, margin=0.15):
        goal_x, goal_y = goal
        dist = math.sqrt((self.current_robot_x - goal_x)**2 + (self.current_robot_y - goal_y)**2)
        return dist <= margin

    def recover(self):
        if self.stuck_detected:
            return
        self.stuck_detected = True
        self.recovery_mode = True

        # Save the stuck goal to retry later
        self.stuck_goal = self.absolute_waypoints[self.current_waypoint_index]

        # Cancel current goal
        if self.goal_handle:
            cancel_future = self.goal_handle.cancel_goal_async()
            self.get_logger().info('Cancelling current goal...')
        
            def after_cancel(fut):
                try:
                    result = fut.result()
                    self.get_logger().info('Goal cancelled successfully.')
                except Exception as e:
                    self.get_logger().error(f'Cancel failed: {e}')
                
                # Now send goal to home
                self.get_logger().info('Sending goal to (0,0) for recovery...')
                home_goal = (self.initial_x, self.initial_y)
                self.send_next_goal(target=home_goal)

            cancel_future.add_done_callback(after_cancel)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
