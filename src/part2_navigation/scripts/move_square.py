#!/usr/bin/env python3
# A simple ROS2 Publisher for Square Motion

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 
from part2_navigation_modules.tb3_tools import quaternion_to_euler 
from math import sqrt, pow, pi 

class Square(Node):
    def __init__(self):
        super().__init__("move_square")
        self.first_message = False
        self.turn = False 
        self.vel_msg = Twist() 
        
        # Position tracking
        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0
        self.side_length = 1.0  # meters
        self.target_angle = pi/2  # 90 degrees in radians
        
        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10)
        
        # Control timer
        ctrl_rate = 10  # Hz
        self.timer = self.create_timer(1/ctrl_rate, self.timer_callback)
        self.shutdown = False

        self.get_logger().info(f"'{self.get_name()}' node initialized")

    def on_shutdown(self):
        self.get_logger().info("Stopping robot...")
        self.vel_pub.publish(Twist())  # Stop command
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 
        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation) 
        self.x = pose.position.x 
        self.y = pose.position.y
        self.theta_z = yaw  # Keep signed value for turning

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z

    def timer_callback(self):
        if not self.first_message:
            return

        # Calculate distance from starting point
        distance = sqrt(pow(self.x - self.xref, 2) + pow(self.y - self.yref, 2))
        angle_diff = abs(self.theta_z - self.theta_zref)

        if self.turn:
            # Rotate 90 degrees
            if angle_diff < self.target_angle:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.2  # Slow turn
            else:
                self.turn = False
                self.vel_msg.angular.z = 0.0
                self.xref = self.x
                self.yref = self.y
                self.theta_zref = self.theta_z
        else:
            # Move forward 1m
            if distance < self.side_length:
                self.vel_msg.linear.x = 0.1  # Slow forward
                self.vel_msg.angular.z = 0.0
            else:
                self.turn = True
                self.vel_msg.linear.x = 0.0
                self.xref = self.x
                self.yref = self.y

        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    move_square = Square()
    try:
        rclpy.spin(move_square)
    except KeyboardInterrupt:
        move_square.get_logger().info("Shutdown requested")
    finally:
        move_square.on_shutdown()
        while not move_square.shutdown:
            rclpy.spin_once(move_square, timeout_sec=0.1)
        move_square.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()