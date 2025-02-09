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
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Build the DriveTrain Controller Plugin
add_library(drive_train_controller SHARED src/DriveTrainManager/DriveTrainController.cpp)
ament_target_dependencies(drive_train_controller rclcpp controller_interface pluginlib)

# Export DriveTrain Controller plugin
pluginlib_export_plugin_description_file(controller_interface main_computer_urc.xml)

# Build the Fake DriveTrain Hardware Plugin
add_library(drive_train_hardware SHARED src/DriveTrainManager/DriveTrainHardware.cpp)
ament_target_dependencies(drive_train_hardware rclcpp hardware_interface pluginlib cross_pkg_messages)

# Export Fake Hardware plugin
pluginlib_export_plugin_description_file(hardware_interface main_computer_urc.xml)

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
  drive_train_controller
  drive_train_hardware
  StatusLED_node
  VideoStreamer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the plugin description file
install(FILES
  main_computer_urc.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY
  launch
  description
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
