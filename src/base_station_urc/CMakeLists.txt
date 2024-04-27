cmake_minimum_required(VERSION 3.5)
project(base_station_urc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ImGui REQUIRED)

# Add executable for JoyMapper
add_executable(JoyMapper_node src/joyMapper/joyMapper.cpp)
ament_target_dependencies(JoyMapper_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

# 
add_executable(my_gui_app src/my_gui_app.cpp)
add_executable(gui_node src/gui/gui_node.cpp)
ament_target_dependencies(gui_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install and export
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  JoyMapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()