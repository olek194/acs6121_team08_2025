#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import threading
from filterpy.kalman import KalmanFilter

class OdomVisualizer(Node):
    def __init__(self):
        super().__init__("odom_visualizer")

        # Subscribe to odometry topics
        self.odom_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.ekf_subscriber = self.create_subscription(Odometry, "odom_ekf", self.ekf_callback, 10)

        # Store trajectories
        self.raw_trajectory = []
        self.noisy_trajectory = []
        self.kf_trajectory = []
        self.ekf_trajectory = []

        # Create a 2D Kalman Filter (x, y positions)
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array([[1, 0, 1, 0],  # State transition model
                              [0, 1, 0, 1],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0],  # Measurement function
                              [0, 1, 0, 0]])
        self.kf.P *= 1000  # Initial uncertainty
        self.kf.R = np.array([[0.5, 0],  # Measurement noise
                              [0, 0.5]])
        self.kf.Q = np.eye(4) * 0.1  # Process noise
        self.kf.x = np.zeros((4, 1))  # Initial state

        # Start visualization in a separate thread
        self.plot_thread = threading.Thread(target=self.plot_trajectory)
        self.plot_thread.start()

        self.get_logger().info("OdomVisualizer Node Initialized")

    def add_noise(self, x, y):
        """Simulates sensor noise in odometry readings."""
        noise_x = np.random.normal(0, 0.05)  # Mean 0, std 0.05
        noise_y = np.random.normal(0, 0.05)
        return x + noise_x, y + noise_y

    def odom_callback(self, msg):
        """Callback for raw odometry."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        # Add artificial noise to simulate real-world conditions
        noisy_x, noisy_y = self.add_noise(x, y)

        # Kalman Filter update
        z = np.array([[noisy_x], [noisy_y]])  # Measurement
        self.kf.predict()
        self.kf.update(z)
        kf_x, kf_y = self.kf.x[0, 0], self.kf.x[1, 0]

        # Store trajectory points
        self.raw_trajectory.append((x, y))
        self.noisy_trajectory.append((noisy_x, noisy_y))
        self.kf_trajectory.append((kf_x, kf_y))

    def ekf_callback(self, msg):
        """Callback for EKF-filtered odometry."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.ekf_trajectory.append((x, y))

    def plot_trajectory(self):
        """Real-time visualization."""
        plt.figure(figsize=(8, 6))

        while rclpy.ok():
            if len(self.raw_trajectory) > 1:
                plt.clf()

                # Extract trajectories
                raw_x, raw_y = zip(*self.raw_trajectory)
                noisy_x, noisy_y = zip(*self.noisy_trajectory)
                kf_x, kf_y = zip(*self.kf_trajectory)
                ekf_x, ekf_y = zip(*self.ekf_trajectory) if self.ekf_trajectory else ([], [])

                # Plot data
                plt.plot(raw_x, raw_y, 'g-', label="Raw Odometry", alpha=0.4)
                plt.plot(noisy_x, noisy_y, 'r.', label="Noisy Odometry", markersize=3, alpha=0.3)
                plt.plot(kf_x, kf_y, 'b-', label="Kalman Filtered", linewidth=2)
                if ekf_x:
                    plt.plot(ekf_x, ekf_y, 'm-', label="EKF Odometry", linewidth=2)

                plt.xlabel("X Position (m)")
                plt.ylabel("Y Position (m)")
                plt.title("Raw vs Noisy vs KF vs EKF Trajectory")
                plt.legend()
                plt.grid(True)
                plt.pause(0.1)  # Refresh plot

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OdomVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
