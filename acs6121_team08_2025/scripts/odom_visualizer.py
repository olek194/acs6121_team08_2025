#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import threading

class OdomVisualizer(Node):
    def __init__(self):
        super().__init__("odom_visualizer")

        # Subscribe to raw and EKF odometry topics
        self.odom_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.ekf_subscriber = self.create_subscription(Odometry, "odom_ekf", self.ekf_callback, 10)

        # Storage for trajectories
        self.odom_trajectory = []
        self.ekf_trajectory = []

        # Start visualization in a separate thread
        self.plot_thread = threading.Thread(target=self.plot_trajectory)
        self.plot_thread.start()

        self.get_logger().info("OdomVisualizer Node Initialized")

    def odom_callback(self, msg):
        """Callback function for raw odometry"""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.odom_trajectory.append((x, y))

    def ekf_callback(self, msg):
        """Callback function for EKF-filtered odometry"""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.ekf_trajectory.append((x, y))

    def plot_trajectory(self):
        """Live plotting function"""
        plt.figure(figsize=(8, 6))
        while rclpy.ok():
            if len(self.odom_trajectory) > 1 and len(self.ekf_trajectory) > 1:
                plt.clf()
                odom_x, odom_y = zip(*self.odom_trajectory)
                ekf_x, ekf_y = zip(*self.ekf_trajectory)

                plt.plot(odom_x, odom_y, 'r.', label="Raw Odometry", alpha=0.5)
                plt.plot(ekf_x, ekf_y, 'b-', label="EKF Odometry", linewidth=2)

                plt.xlabel("X Position (m)")
                plt.ylabel("Y Position (m)")
                plt.legend()
                plt.title("Raw vs EKF Odometry Trajectory")
                plt.grid(True)
                plt.pause(0.1)  # Update plot every 0.1s

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OdomVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
