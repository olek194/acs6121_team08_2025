Real-World Exploration with TurtleBot3 Waffle
ACS 6121 Group 8 University of Sheffield

This repository contains the ROS2 package for the Real-World Exploration task in the ACS6121 Mobile Robotics and Autonomous Systems module. The goal is to program a TurtleBot3 Waffle robot to autonomously explore a 4x4 meter arena containing obstacles while avoiding collisions.

Table of Contents
Overview
Features
Installation
Usage
Team Collaboration
Submission Guidelines
Additional Resources
Overview
The robot must autonomously explore as much of the Diamond Computer Room 5 Robot Arena as possible within 90 seconds, avoiding obstacles such as wooden walls and colored cylinders. The arena is divided into 16 zones, and the robot must enter as many of the outer 12 zones as possible.

Key tasks include:

Implementing obstacle avoidance using LiDAR data.
Developing exploration strategies (e.g., random walk, wall following).
Optionally integrating SLAM for mapping the environment.
Features
Obstacle Avoidance: Uses LiDAR data to detect and avoid obstacles.
Exploration Strategy: Implements a Finite State Machine (FSM) for autonomous navigation.
SLAM Integration (Optional): Generates a map of the arena during exploration.
ROS2 Compatibility: Built for ROS2 Humble Hawksbill.
Installation
Prerequisites
Operating System: Ubuntu 22.04 (or compatible OS with ROS2 Humble installed).
ROS2 Humble: Follow the official ROS2 installation guide.
TurtleBot3 Packages: Install the TurtleBot3 simulation packages:
sudo apt install ros-humble-turtlebot3*
Clone the Repository
cd ~/ros2_ws/src
git clone https://github.com/your-repo/acs6121_teamXX_2025.git
cd ..
Build the Package
colcon build --packages-select acs6121_teamXX_2025 --symlink-install
source install/setup.bash
Usage
Launch the Exploration Node
To run the exploration algorithm:

ros2 launch acs6121_teamXX_2025 explore.launch.py
Run SLAM (Optional)
If SLAM is integrated, you can launch it alongside the exploration node:

ros2 launch tuos_simulations cartographer.launch.py
Save the Map (Optional)
After exploration, save the generated map programmatically or via the command line:

ros2 run nav2_map_server map_saver_cli -f maps/explore_map
Team Collaboration
Branching Strategy
Main Branch (main): Stable and production-ready code.
Feature Branches: Used for new features or fixes (e.g., feature/add-obstacle-avoidance, bugfix/fix-lidar-filter).
Pull Requests (PRs): All changes must be reviewed via PRs before merging into main.
Workflow
Create a new branch for your task:
git checkout -b feature/<task-name>
Commit your changes:
git add .
git commit -m "Add <description>"
Push the branch to GitHub:
git push origin feature/<task-name>
Create a PR on GitHub for review.
Submission Guidelines
Deadline: Submit your ROS package via Blackboard by the end of Week 11.
Package Structure:
Include a launch file named explore.launch.py.
If SLAM is implemented, ensure the map files (explore_map.png and explore_map.yaml) are saved in the maps directory.
Marking Criteria:
Exploration (12/25): Enter as many outer zones as possible.
Run Time (8/25): Maximize exploration time without collisions.
Mapping with SLAM (5/25): Generate and save a map of the arena.
Additional Resources
Simulation Lab Course: Review exercises and concepts from Part 3 (LiDAR, SLAM).
ACS6121 Lectures: Refer to lecture notes on Finite State Machines and Artificial Potential Fields.
ROS2 Documentation: Official ROS2 Documentation
Discussion Board: Use the ACS6121 Discussion Board on Blackboard for questions.
License
This project is licensed under the MIT License. See the LICENSE file for details.