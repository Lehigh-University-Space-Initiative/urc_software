cmake_minimum_required(VERSION 3.5)
project(main_computer_urc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Declare the DriveTrainManager executable
add_executable(DriveTrainManager_node src/DriveTrainManager/main.cpp)
ament_target_dependencies(DriveTrainManager_node rclcpp cross_pkg_messages std_msgs)

# Declare the StatusLED executable
add_executable(StatusLED_node src/StatusLED/StatusLED.cpp)
ament_target_dependencies(StatusLED_node rclcpp)

# Add VideoStreamer node
add_executable(VideoStreamer_node src/VideoStreamer/VideoStreamer.cpp)
ament_target_dependencies(VideoStreamer_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

# Install the executables
install(TARGETS
  DriveTrainManager_node
  StatusLED_node
  VideoStreamer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
