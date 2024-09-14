cmake_minimum_required(VERSION 3.5)
project(main_computer_urc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Declare the DriveTrainManager executable
add_executable(DriveTrainManager_node src/DriveTrainManager/main.cpp)
ament_target_dependencies(DriveTrainManager_node rclcpp cross_pkg_messages std_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the node
install(TARGETS
  DriveTrainManager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
