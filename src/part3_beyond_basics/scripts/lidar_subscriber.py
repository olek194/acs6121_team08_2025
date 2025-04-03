#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions 

from sensor_msgs.msg import LaserScan 

import numpy as np 

class LidarSubscriber(Node): 

    def __init__(self): 
        super().__init__("lidar_subscriber")

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) 

        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    def lidar_callback(self, scan_data: LaserScan): 
        subset = scan_data.ranges[70:110:]
        min_distance = min(subset)

        # Log the readings
        self.get_logger().info(
            f"Detected an object {min_distance:.3f} m away."
        )

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()