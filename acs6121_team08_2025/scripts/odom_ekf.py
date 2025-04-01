#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from part2_navigation_modules.tb3_tools import quaternion_to_euler
import numpy as np

class OdomEKF(Node):
    def __init__(self):
        super().__init__("odom_ekf")
        
        # Subscribe to raw odometry data
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        
        # Publisher for EKF-filtered odometry
        self.ekf_publisher = self.create_publisher(Odometry, "odom_ekf", 10)

        # EKF state [x, y, theta]
        self.x = np.array([[0.0], [0.0], [0.0]])  
        self.P = np.eye(3) * 0.1  # Covariance matrix
        self.Q = np.diag([0.01, 0.01, 0.001])  # Process noise
        self.R = np.diag([0.1, 0.1, 0.05])  # Measurement noise

        self.prev_time = None
        self.get_logger().info(f"{self.get_name()} initialized.")

    def odom_callback(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Extract raw odometry data
        pos_x, pos_y, _ = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        roll, pitch, yaw = quaternion_to_euler(msg.pose.pose.orientation)
        vel_x = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # Prediction step using motion model
        theta = self.x[2, 0]
        F = np.eye(3)
        F[0, 2] = -vel_x * np.sin(theta) * dt
        F[1, 2] = vel_x * np.cos(theta) * dt

        B = np.array([
            [np.cos(theta) * dt, 0],
            [np.sin(theta) * dt, 0],
            [0, dt]
        ])

        u = np.array([[vel_x], [omega]])  # Control inputs
        self.x = self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q

        # Measurement update
        z = np.array([[pos_x], [pos_y], [yaw]])  # Measurement
        H = np.eye(3)  # Direct observation
        y = z - H @ self.x  # Innovation
        S = H @ self.P @ H.T + self.R  # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman Gain

        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        # Publish EKF-corrected odometry
        ekf_msg = msg
        ekf_msg.pose.pose.position.x = self.x[0, 0]
        ekf_msg.pose.pose.position.y = self.x[1, 0]
        self.ekf_publisher.publish(ekf_msg)

        self.get_logger().info(f"EKF: x={self.x[0,0]:.3f}, y={self.x[1,0]:.3f}, theta={self.x[2,0]:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
