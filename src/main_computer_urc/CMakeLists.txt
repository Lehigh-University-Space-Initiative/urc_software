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
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

set(DRIVE_TRAIN_MANAGER_SRC
  src/DriveTrainManager/main.cpp
)

# Declare the DriveTrainManager executable
#add_executable(DriveTrainManager_node src/DriveTrainManager/main.cpp)
#ment_target_dependencies(DriveTrainManager_node rclcpp cross_pkg_messages std_msgs)

add_library(DriveTrainManager_node SHARED ${DRIVE_TRAIN_MANAGER_SRC})
set_target_properties(DriveTrainManager_node PROPERTIES OUTPUT_NAME "DriveTrainManager_node")
ament_target_dependencies(DriveTrainManager_node
  rclcpp
  pluginlib
  controller_interface
  cross_pkg_messages
)
target_link_libraries(DriveTrainManager_node)


pluginlib_export_plugin_description_file(controller_interface plugin.xml)

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
