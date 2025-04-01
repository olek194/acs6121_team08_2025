#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import Odometry  # Odometry messages
import argparse


class MapSaverClient(Node):

    def __init__(self):
        super().__init__('map_saver_client')

        # Service Client to Save the Map
        self.client = self.create_client(SaveMap, 'map_saver')

        # Subscriber for Odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',  # Ensure this matches your robot's odometry topic
            self.odom_callback,
            10
        )

        # Argument Parser for File Name
        cli = argparse.ArgumentParser()
        cli.add_argument("-m", "--map-file", default="saved_map", type=str)
        self.args = cli.parse_args()

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for map_saver service...")

        # Control variable to ensure odometry is received before starting
        self.odom_received = False
        self.timer = None  # Timer is not started until odometry is received

    def odom_callback(self, msg):
        """Callback function that checks when odometry is available."""
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("✅ Odometry received. Starting map updates...")
            self.timer = self.create_timer(5.0, self.update_map)  # Start periodic map saving

    def update_map(self):
        """ Calls the map_saver service and prints a yellow marker in terminal. """
        if not self.odom_received:
            self.get_logger().warn("🚫 Skipping map update, waiting for odometry...")
            return

        request = SaveMap.Request()
        request.map_url = self.args.map_file

        # Call service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.map_saved_callback)

    def map_saved_callback(self, future):
        """ Callback function when map is successfully saved. """
        try:
            response = future.result()
            if response.success:
                # Print Yellow Marker in Terminal
                self.get_logger().info("🟡 MAP UPDATED! The map has been saved successfully.")
            else:
                self.get_logger().error("❌ Failed to save the map.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    
def main():
    rclpy.init()
    client = MapSaverClient()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
