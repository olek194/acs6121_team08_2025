#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'object_detection',
            10
        )

        self.get_logger().info("LiDAR Processor Node has started.")

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Remove invalid values
        ranges = np.where((ranges == 0.0) | np.isinf(ranges), np.nan, ranges)

        # Front-facing ±20 degrees
        total_angles = len(ranges)
        fov_degrees = 40
        mid_index = total_angles // 2
        angle_range = int(fov_degrees / 2 / (angle_increment * 180 / np.pi))

        front_ranges = ranges[mid_index - angle_range : mid_index + angle_range + 1]
        front_angles = np.linspace(-fov_degrees / 2, fov_degrees / 2, len(front_ranges))

        # Group into 2° chunks
        chunk_size = int(2 / (angle_increment * 180 / np.pi))
        min_distances = []
        min_angles = []

        for i in range(0, len(front_ranges), chunk_size):
            chunk = front_ranges[i:i+chunk_size]
            chunk_angles = front_angles[i:i+chunk_size]

            if len(chunk) == 0:
                continue

            # Skip NaN-only chunks
            valid_chunk = chunk[~np.isnan(chunk)]
            if len(valid_chunk) == 0:
                continue

            min_dist = np.min(valid_chunk)
            min_angle = chunk_angles[np.nanargmin(chunk)]
            
            min_distances.append(min_dist)
            min_angles.append(min_angle)

        if min_distances:
            closest_idx = int(np.argmin(min_distances))
            closest_distance = min_distances[closest_idx]
            closest_angle = min_angles[closest_idx]

            # Publish distance and angle as Float32MultiArray
            msg_out = Float32MultiArray()
            msg_out.data = [float(closest_distance), float(closest_angle)]
            self.publisher.publish(msg_out)

            self.get_logger().info(
                f"Closest object: {closest_distance:.2f} m at {closest_angle:.1f}°"
            )
        else:
            self.get_logger().info("No valid LiDAR data in front.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
