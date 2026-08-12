cmake_minimum_required(VERSION 3.5)
project(navigation_urc)

# main.cpp uses std::clamp, which needs C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cross_pkg_messages REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# GNSS waypoint follower node
add_executable(WaypointFollower_node src/WaypointFollower/main.cpp)
ament_target_dependencies(WaypointFollower_node rclcpp geometry_msgs cross_pkg_messages)

# Install the executables
install(TARGETS
  WaypointFollower_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the fake GPS test script -- lets `ros2 run navigation_urc fake_gps_node.py` find it
install(PROGRAMS
  scripts/fake_gps_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
