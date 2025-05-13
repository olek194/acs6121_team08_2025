from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution



def generate_launch_description():
    return LaunchDescription([

        # ## START SIMULATION WORLD
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory("tuos_simulations"),
        #             "launch", "acs6121.launch.py"
        #         )
        #     )
        # ),

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

        # SLAM TOOLBOX (instead of cartographer)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory("slam_toolbox"),
        #             "launch", "online_async_launch.py"
        #         )
        #     )
        # ),

       
     ## START NAVIGATION (Nav2)
    
            
        # #  Set the model BEFORE launching anything
        # SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("turtlebot3_navigation2"),
                    "launch",
                    "navigation2.launch.py"
                ])
            ),
            launch_arguments={
                'slam': 'True',
                'params_file': '/home/student/ros2_ws/src/acs6121_team08_2025/scripts/waffle.yaml'
            }.items()
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

        

        ## WAYPOINT NAVIGATOR (Sending multiple waypoints)
        TimerAction(
             period=10.0,
             actions=[
                Node(
                    package='acs6121_team08_2025',  # Replace with your package
                    executable='waypoint_navigator.py',  # Script that sends waypoints
                    name='waypoint_navigator',
                    output='screen'
                ),
             ]
        ),
            
            # MAP SAVER CLIENT
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
   
        TimerAction(
            period=95.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', '/home/student/ros2_ws/src/acs6121_team08_2025/maps/explore_map'],
                    output='screen'
                )
            ]
        )

        
    
    ])