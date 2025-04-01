from launch import LaunchDescription 
from launch_ros.actions import Node 
import os
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
    return LaunchDescription([ 

        ## START SIMULATION WORLD
        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource( 
                os.path.join( 
                    get_package_share_directory("tuos_simulations"), 
                    "launch", "acs6121.launch.py" 
                )
            )
        ),
       
        ## LIDAR Subscriber NODE
        Node( 
            package='acs6121_team08_2025', 
            executable='lidar_subscriber.py', 
            name='lidar_subscriber' 
        ),

        ## Robot Odometry 
        Node(
            package='acs6121_team08_2025', 
            executable='odom_subscriber.py', 
            name='odom_subscriber_2'
        ),

        ## SLAM builder 
        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource( 
                os.path.join( 
                    get_package_share_directory("tuos_simulations"), 
                    "launch", "cartographer.launch.py" 
                )
            )
        ),

        ## Start Map Server with a delay to ensure dependencies are ready
        TimerAction(
            period=5.0,  # Wait for 5 seconds before starting the map server
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("nav2_map_server"),
                            "launch", "map_saver_server.launch.py"
                        )
                    )
                )
            ]
        ),

        ## Lifecycle Manager to Activate `map_saver_server`
        TimerAction(
            period=5.0,  # Wait for 10 seconds to ensure the server is started before activation
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_map",
                    parameters=[{
                        "autostart": True,
                        "node_names": ["map_saver"]
                    }]
                )
            ]
        ),
        # ## START NAVIGATION FILE
        # IncludeLaunchDescription( 
        #     PythonLaunchDescriptionSource( 
        #         os.path.join( 
        #             get_package_share_directory("nav2_bringup"), 
        #             "launch", "navigation_launch.py" 
        #         )
        #     )
        # ),
        ## Map Saver Client (Saves the map every 5 seconds)
        TimerAction(
            period=25.0,  # Ensure the server is fully activated before starting the client
            actions=[
                Node(
                    package='acs6121_team08_2025',  # Replace with your package name
                    executable='map_saver_client.py',  # Name of the executable in setup.py
                    name='map_saver_client',
                    output='screen',
                    parameters=[{
                        "map-file": os.path.expanduser("~/ros2_ws/src/acs6121_team08_2025/maps/updated_map_1_04_2025")
                    }]
                )
            ]
        ),
    ])
