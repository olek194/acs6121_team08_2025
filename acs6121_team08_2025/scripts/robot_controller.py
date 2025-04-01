#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
#from tf_transformations import euler_from_quaternion
from part2_navigation_modules.tb3_tools import quaternion_to_euler

import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'object_detection',
            self.obstacle_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control & state
        self.obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        self.yaw = 0.0
        self.initial_yaw = None

        self.linear_speed = 0.2
        self.turn_speed = 0.4
        self.safe_distance = 0.5
        self.Kp = 1.0  # PID gain for yaw correction

        self.state = 'FORWARD'
        self.sides_completed = 0
        self.start_time = self.get_clock().now()
        self.side_duration = 4.0 / self.linear_speed
        self.timer = self.create_timer(0.1, self.control_loop)

        # Re-routing
        self.blocked_since = None
        self.reroute_stage = 0  # 0: waiting, 1: sidestep, 2: turn

    def obstacle_callback(self, msg: Float32MultiArray):
        self.obstacle_distance = msg.data[0]
        self.obstacle_angle = msg.data[1]

    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = quaternion_to_euler(orientation_q)
        self.yaw = yaw

    def control_loop(self):
        twist = Twist()

        if self.state == 'FORWARD':
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            if self.obstacle_distance < self.safe_distance:
                if self.blocked_since is None:
                    self.blocked_since = self.get_clock().now()
                else:
                    blocked_time = (self.get_clock().now() - self.blocked_since).nanoseconds / 1e9
                    if blocked_time > 2.0:
                        self.get_logger().info("Blocked. Initiating re-route.")
                        self.state = 'REROUTE'
                        self.reroute_stage = 0
                        self.reroute_start = self.get_clock().now()
                        return
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                self.blocked_since = None
                # PID heading control
                if self.initial_yaw is None:
                    self.initial_yaw = self.yaw
                yaw_error = self.normalize_angle(self.initial_yaw - self.yaw)
                twist.linear.x = self.linear_speed
                twist.angular.z = self.Kp * yaw_error

            if elapsed > self.side_duration:
                self.get_logger().info("Completed side. Turning.")
                self.state = 'TURN'
                self.turn_start = self.get_clock().now()
                self.initial_yaw = None
                return

        elif self.state == 'TURN':
            elapsed = (self.get_clock().now() - self.turn_start).nanoseconds / 1e9
            if elapsed >= (math.pi / 2) / self.turn_speed:  # ~90 deg
                self.sides_completed += 1
                if self.sides_completed >= 4:
                    self.get_logger().info("Navigation complete.")
                    self.timer.cancel()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    return
                self.state = 'FORWARD'
                self.start_time = self.get_clock().now()
                self.initial_yaw = None
                return
            twist.angular.z = self.turn_speed

        elif self.state == 'REROUTE':
            reroute_time = (self.get_clock().now() - self.reroute_start).nanoseconds / 1e9
            if self.reroute_stage == 0:
                self.get_logger().info("Sidestepping.")
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                if reroute_time > 1.0:
                    self.reroute_stage = 1
                    self.reroute_start = self.get_clock().now()
            elif self.reroute_stage == 1:
                self.get_logger().info("Retrying forward motion.")
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                if self.obstacle_distance > self.safe_distance:
                    self.get_logger().info("Path cleared. Resuming forward.")
                    self.state = 'FORWARD'
                    self.blocked_since = None
                    self.start_time = self.get_clock().now()
                    return
                elif reroute_time > 2.0:
                    self.reroute_stage = 2
                    self.reroute_start = self.get_clock().now()
            elif self.reroute_stage == 2:
                self.get_logger().info("Turning to reroute.")
                twist.linear.x = 0.0
                twist.angular.z = -0.5
                if reroute_time > 1.5:
                    self.get_logger().info("Back to forward mode.")
                    self.state = 'FORWARD'
                    self.start_time = self.get_clock().now()
                    self.blocked_since = None
                    self.initial_yaw = None
                    return

        self.publisher.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
