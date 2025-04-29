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
        # Node(
        #     package='acs6121_team08_2025',
        #     executable='lidar_subscriber.py',
        #     name='lidar_subscriber'
        # ),

        # ## Robot Odometry
        # Node(
        #     package='acs6121_team08_2025',
        #     executable='odom_subscriber.py',
        #     name='odom_subscriber_2'
        # ),

        ## SLAM TOOLBOX (instead of cartographer)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch", "online_async_launch.py"
                )
            )
        ),

       
     ## START NAVIGATION (Nav2)
    
    
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("nav2_bringup"),
                            "launch", "navigation_launch.py"
                        )
                    )
                )
            ]
        ),

        # ## Start Map Server with a delay
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 os.path.join(
        #                     get_package_share_directory("nav2_map_server"),
        #                     "launch", "map_saver_server.launch.py"
        #                 )
        #             )
        #         )
        #     ]
        # ),

        # ## Lifecycle Manager to Activate `map_saver_server`
        # TimerAction(
        #     period=10.0,
        #     actions=[
        #         Node(
        #             package="nav2_lifecycle_manager",
        #             executable="lifecycle_manager",
        #             name="lifecycle_manager_map",
        #             parameters=[{
        #                 "autostart": True,
        #                 "node_names": ["map_saver"]
        #             }]
        #         )
        #     ]
        # ),

        # # MAP SAVER CLIENT
        # TimerAction(
        #     period=25.0,
        #     actions=[
        #         Node(
        #             package='acs6121_team08_2025',
        #             executable='map_saver_client.py',
        #             name='map_saver_client',
        #             output='screen',
        #             parameters=[{
        #                 "map-file": os.path.expanduser("~/ros2_ws/src/acs6121_team08_2025/maps/updated_map_1_04_2025")
        #             }]
        #         )
        #     ]
        # ),

        ## WAYPOINT NAVIGATOR (Sending multiple waypoints)
        TimerAction(
            period=15.0,  # Give time for SLAM and Nav2 to initialize
            actions=[
                Node(
                    package='acs6121_team08_2025',  # Replace with your package
                    executable='waypoint_navigator.py',  # Script that sends waypoints
                    name='waypoint_navigator',
                    output='screen'
                )
            ]
        ),
    ])
