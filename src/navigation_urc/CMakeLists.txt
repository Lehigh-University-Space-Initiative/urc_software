# navigation_urc: autonomous navigation (currently GNSS waypoint following)
#
# Builds:
#   WaypointFollower_node - drives toward a GNSS waypoint by publishing /cmd_vel (src/WaypointFollower)
#
# Installs:
#   fake_gps_node.py - simulated GPS for hardware-free testing (scripts/)
#   launch/ - navigation_launch.py (real GPS) and navigation_sim_launch.py (fake GPS)

cmake_minimum_required(VERSION 3.5)
project(navigation_urc)

# main.cpp uses std::clamp, which needs C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Finding the ROS packages this package uses (each must also be listed in package.xml)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cross_pkg_messages REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(WaypointFollower_node src/WaypointFollower/main.cpp)
ament_target_dependencies(WaypointFollower_node rclcpp geometry_msgs cross_pkg_messages)

# Installing into lib/<package>, which is where `ros2 run` and launch files look for executables
install(TARGETS
  WaypointFollower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# PROGRAMS (rather than FILES) keeps the script executable, so `ros2 run navigation_urc fake_gps_node.py` works
install(PROGRAMS
  scripts/fake_gps_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
