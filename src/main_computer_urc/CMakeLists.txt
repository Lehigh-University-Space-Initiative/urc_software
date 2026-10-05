# main_computer_urc: code that runs on the rover's main computer
#
# Builds:
#   DriveTrainManager_node - /cmd_vel to per-wheel speeds (src/DriveTrainManager)
#   StatusLED_node - status light placeholder (src/StatusLED, not launched yet)
#   VideoStreamer_node - camera capture and ArUco overlay (src/VideoStreamer)
#   libmock_arm_hw.so - ros2_control plugin bridging MoveIt to the arm Pi (src/MockArmHardware, see plugin.xml)
#
# Installs launch/, description/ (URDF/xacro), and config/ so `ros2 launch` and MoveIt can find them

cmake_minimum_required(VERSION 3.5)
project(main_computer_urc)

# Finding the ROS packages and libraries this package uses (each must also be listed in package.xml)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Executables: add_executable lists the sources; ament_target_dependencies links the ROS packages they use
add_executable(DriveTrainManager_node src/DriveTrainManager/main.cpp)
ament_target_dependencies(DriveTrainManager_node rclcpp cross_pkg_messages std_msgs)

add_executable(StatusLED_node src/StatusLED/StatusLED.cpp)
ament_target_dependencies(StatusLED_node rclcpp)

add_executable(VideoStreamer_node src/VideoStreamer/VideoStreamer.cpp)
ament_target_dependencies(VideoStreamer_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

# The arm bridge is a shared library (plugin), not an executable; ros2_control loads it at runtime
file(GLOB MockArmHardware_SRC
  src/MockArmHardware/ArmHardware.cpp
)

add_library(MockArmHardware SHARED ${MockArmHardware_SRC})
set_target_properties(MockArmHardware PROPERTIES OUTPUT_NAME "mock_arm_hw")  # Produces libmock_arm_hw.so (named in plugin.xml)
ament_target_dependencies(MockArmHardware
  rclcpp
  hardware_interface
  pluginlib
  cross_pkg_messages
)

# Registering plugin.xml so pluginlib can find the plugin by its name, main_computer_urc/MockArmHardware
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS
  MockArmHardware
  DESTINATION lib
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Installing executables into lib/<package>, which is where `ros2 run` and launch files look for them
install(TARGETS
  DriveTrainManager_node
  StatusLED_node
  VideoStreamer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  description
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
