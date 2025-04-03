cmake_minimum_required(VERSION 3.8)
project(acs6121_team08_2025)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME}_modules)

install(PROGRAMS
  scripts/explorer_node.py
  scripts/map_saver_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()