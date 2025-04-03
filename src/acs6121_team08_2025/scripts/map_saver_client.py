#!/usr/bin/env python3
# Node to save SLAM map to a specific directory (acs6121_team08_2025/maps)

import rclpy
from rclpy.node import Node
import os
import subprocess
from ament_index_python.packages import PackageNotFoundError # Removed - not needed now
import time

class MapSaverClient(Node):

    def __init__(self):
        super().__init__("map_saver_client")

        # --- Configuration ---
        self.target_directory = os.path.expanduser("~/ros2_ws/src/acs6121_team08_2025/maps") # FIXED save location
        self.map_filename_base = 'nav_world_map_auto'  # Base name for map files
        self.save_delay_seconds = 90.0        # Time to wait before saving

        # Ensure the target directory exists
        try:
            os.makedirs(self.target_directory, exist_ok=True)
            self.get_logger().info(f"Ensured map directory exists: {self.target_directory}")
            self.map_saved = False # Initialization only when the dir exists

             # One-shot timer for map saving
            self.timer = self.create_timer(self.save_delay_seconds, self.save_map_callback)
            self.get_logger().info(f"Timer started. Will save map in {self.save_delay_seconds} seconds.")

        except OSError as e:
            self.get_logger().error(f"Failed to create map directory {self.target_directory}: {e}")
            self.target_directory = None # Invalidate the path
            self.map_saved = True # prevent saving map since directory fails
            self.timer = None
            self.get_logger().error("Map saving disabled due to directory creation failure.")


        self.get_logger().info(f"'{self.get_name()}' node initialized.")

    def save_map_callback(self):
        """Timer callback to save the map."""
        if not self.map_saved and self.target_directory: # Verify target_dir again
             # Only run the save if the directory is valid and the map isn't saved

            # Disable the rest of the runs of this function, as it will only be run once.
            self.destroy_timer(self.timer) # Prevents the function from being called twice.
            self.save_map()
        else:
            self.get_logger().warn("Map saving skipped either due to incorrect path or the timer expiring twice.")


    def save_map(self):
        """Saves the map using map_saver_cli."""
        # Verify path
        if not self.target_directory:
            self.get_logger().error("Cannot save map. Target directory is invalid.")
            return

        # Construct full file path
        map_path = os.path.join(self.target_directory, self.map_filename_base)
        self.get_logger().info(f"Attempting to save map files to: {map_path}")

        # Command
        command = [
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-f", map_path,
            # Optional: Specify map topic if not default '/map'
            # "--ros-args", "-p", "map_topic:=/map"
        ]

        try:
            # Run the command
            self.get_logger().info(f"Executing command: {' '.join(command)}")
            process = subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
            self.get_logger().info("Map saved successfully.")
            self.get_logger().info(f"map_saver_cli output:\n{process.stdout}")
            if process.stderr:
                self.get_logger().warn(f"map_saver_cli stderr output:\n{process.stderr}")
            self.map_saved = True

        except FileNotFoundError:
            self.get_logger().error(f"Failed to save map: 'ros2' or 'nav2_map_server' command not found. Is ROS 2 sourced and nav2_map_server installed?")
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"Failed to save map: map_saver_cli command timed out after 30 seconds.")
        except subprocess.CalledProcessError as e:
            # Log detailed error information
            self.get_logger().error(f"Failed to save map. map_saver_cli returned error code: {e.returncode}")
            self.get_logger().error(f"Command: {' '.join(e.cmd)}")
            self.get_logger().error(f"Output stdout:\n{e.stdout}")
            self.get_logger().error(f"Output stderr:\n{e.stderr}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during map saving: {e}")
        # End the timer now that it has completed.

        self.map_saved = True
        # Node destruction and shutdown are handled cleanly by rclpy.spin exiting
        self.get_logger().info("Map saving process finished (successfully or not). Node will continue running until shutdown.")


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
        pass
    finally:
         # Clean shutdown
         node.destroy_node()
         # Only shutdown if init was successful
         if rclpy.ok():
             rclpy.shutdown()
         print("Map Saver Node finished.")
if __name__ == '__main__':
    main()