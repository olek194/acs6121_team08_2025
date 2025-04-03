from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the Cartographer launch file
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tuos_simulations"),
                "launch", "cartographer.launch.py"
            )
        )
    )

    # Explorer node
    explorer_node = Node(
        package="acs6121_team08_2025",
        executable="explorer_node.py",
        name="explorer_node"
    )

    # Map saver client node
    map_saver_node = Node(
        package="acs6121_team08_2025",
        executable="map_saver_client.py",
        name="map_saver_client"
    )

    return LaunchDescription([
        cartographer_launch,
        explorer_node,
        map_saver_node
    ])