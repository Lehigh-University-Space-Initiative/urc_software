# cross_pkg_messages: the custom ROS 2 message types shared by every package in this repo
#
# The folder is named cross_pkg_messages_urc, but the ROS package name is cross_pkg_messages (set by project() below)
#
# rosidl_generate_interfaces turns each msg/*.msg file into C++ and Python code, e.g.:
#   msg/RoverComputerDriveCMD.msg  ->  #include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
#                                      cross_pkg_messages::msg::RoverComputerDriveCMD (C++)
#                                      from cross_pkg_messages.msg import RoverComputerDriveCMD (Python)
#
# Every other package depends on these generated headers, so dockerBuild.sh builds this package first

cmake_minimum_required(VERSION 3.5)
project(cross_pkg_messages)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# To add a message: create msg/NewThing.msg, list it here, and rebuild this package before the others
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverComputerDriveCMD.msg"
  "msg/GPSData.msg"
  "msg/ArmInputRaw.msg"
  "msg/RoverComputerArmCMD.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
