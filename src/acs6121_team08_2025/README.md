# acs6121_team08_2025

## Overview
This ROS 2 package enables a TurtleBot3 Waffle robot to autonomously explore a 4x4m arena for 90 seconds, avoiding obstacles and mapping the environment using SLAM. The robot aims to visit at least 12 of the outer zones (out of 16 total zones) while avoiding the central 1x1m red zone. The solution uses a Finite State Machine (FSM) for navigation, LiDAR for obstacle avoidance, and Cartographer for SLAM.

## Exploration Strategy
The robot uses a hybrid exploration strategy:
- **Wall-Following**: When near obstacles (e.g., walls or cylinders), the robot maintains a safe distance and follows the obstacle to explore outer zones.
- **Random Walk**: In open areas, the robot occasionally turns randomly to cover more ground and transition between zones.
- **Runtime**: The exploration runs for exactly 90 seconds, ensuring maximum runtime marks.

## Finite State Machine (FSM)
The `explorer_node.py` implements an FSM with the following states:
1. **Move Forward**: Default state when no obstacles are detected ahead (linear velocity = 0.2 m/s, angular velocity = 0.0 rad/s).
2. **Avoid Obstacle**: Triggered when an obstacle is within 0.5m in the forward zone (linear velocity = 0.0 m/s, angular velocity = ±0.5 rad/s, turning toward open space).
3. **Follow Wall**: Triggered when an obstacle is within 0.7m on the left or right (linear velocity = 0.15 m/s, angular velocity = ±0.2 rad/s to maintain distance).
4. **Random Turn**: Triggered randomly (5% chance) in open space (linear velocity = 0.1 m/s, angular velocity = ±0.3 rad/s, random direction).

## LiDAR Data Handling
- **Topic**: Subscribes to `/scan` (`sensor_msgs/LaserScan`).
- **Filtering**: Ignores `ranges` outside `range_min` and `range_max`.
- **Zones** (assuming 360 samples):
  - **Forward**: ±30° (indices 330–360 and 0–30).
  - **Left**: 30°–90° (indices 30–90).
  - **Right**: 270°–330° (indices 270–330).
- **Decision Logic**:
  - Forward < 0.5m: Avoid Obstacle.
  - Left or Right < 0.7m: Follow Wall.
  - All > 1m: Move Forward or Random Turn (5% chance).

## SLAM Mapping
- **Cartographer**: Launched via `tuos_simulations cartographer.launch.py`.
- **Map Saving**: After 90 seconds, `map_saver_client.py` saves the map using `ros2 run nav2_map_server map_saver_cli -f maps/explore_map`.
- **Output**: Saves `explore_map.pgm` and `explore_map.yaml` in the `maps/` directory.

## Package Structure
acs6121_team08_2025/
├── acs6121_team08_2025_modules/
│   ├── init.py
│   └── tb3_tools.py
├── include/
│   └── acs6121_team08_2025/
│       └── minimal_header.hpp
├── launch/
│   └── explore.launch.py
├── maps/
│   ├── explore_map.pgm
│   └── explore_map.yaml
├── scripts/
│   ├── explorer_node.py
│   └── map_saver_client.py
├── src/
│   └── minimal_node.cpp
├── CMakeLists.txt
├── package.xml
└── README.md


## Installation
1. Place the package in your ROS 2 workspace: 
~/ros2_ws/src/acs6121_team08_2025/

2. Build the workspace:
cd ~/ros2_ws/ && colcon build --packages-select acs6121_team08_2025 && source ~/.bashrc

## Running the Code
1. Ensure the TurtleBot3 Waffle is powered on and connected.
2. Launch the exploration:
ros2 launch acs6121_team08_2025 explore.launch.py

This will:
- Start Cartographer SLAM.
- Run the `explorer_node` for 90 seconds.
- Save the map using `map_saver_client` after 90 seconds.

## Notes
- The robot starts at the center of the arena, oriented perpendicular to an outer wall.
- The exploration strategy adapts to changing obstacle placements using real-time LiDAR data.
- The map files (`explore_map.pgm`, `explore_map.yaml`) are saved in the `maps/` directory.

