cmake_minimum_required(VERSION 3.5)
project(base_station_urc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glfw3 REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(sockpp REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include/cs_libguarded
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cross_pkg_messages_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/sockpp 
  ../../libs/imgui
  ../../libs/imgui/backends
)

# Add executable for JoyMapper
add_executable(JoyMapper_node src/joyMapper/joyMapper.cpp)
ament_target_dependencies(JoyMapper_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)
target_include_directories(JoyMapper_node PRIVATE src/base_station_urc/include/cs_libguarded)

# Add executable for TelemetryPanel
# add_executable(TelemetryPanel_node src/gui/panels/TelemetryPanel.cpp)
# ament_target_dependencies(TelemetryPanel_node
#   rclcpp
#   std_msgs
#   cross_pkg_messages
#   cv_bridge
# )
# target_include_directories(TelemetryPanel_node PRIVATE src/base_station_urc/include/cs_libguarded ../../libs/imgui ../../libs/imgui/backends)

# GUI
file(GLOB GUIFiles
     "src/gui/*.cpp"
     "src/gui/panels/*.cpp"
     "../../libs/imgui/*.cpp"
     "../../libs/imgui/backends/imgui_impl_glfw.cpp"
     "../../libs/imgui/backends/imgui_impl_opengl3.cpp"
)

add_executable(GroundStationGUI ${GUIFiles})
target_include_directories(GroundStationGUI PRIVATE
  ../../libs/imgui
  ../../libs/imgui/backends
  src/base_station_urc/include/cs_libguarded
)
target_link_libraries(GroundStationGUI glfw GL dl opencv_highgui)
ament_target_dependencies(GroundStationGUI
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  geometry_msgs
  cross_pkg_messages
)

file(GLOB VisionFiles
      "src/LUSIVisionStreamer/*.cpp"
      "src/LUSIVisionStreamer/LUSIVisionTelem.cpp"
      "src/LUSIVisionStreamer/*.h"

      "src/LUSIVisionStreamer/sockpp/src/*.cpp"

      "src/LUSIVisionStreamer/sockpp/src/linux/*.cpp"
      "src/LUSIVisionStreamer/sockpp/src/unix*.h"
)
add_executable(LUSIVisionStreamer_node ${VisionFiles})
target_include_directories(LUSIVisionStreamer_node PRIVATE src/LUSIVisionStreamer/sockpp/include include/cs_libguarded)
target_link_libraries(LUSIVisionStreamer_node Sockpp::sockpp)
ament_target_dependencies(LUSIVisionStreamer_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  geometry_msgs
  cross_pkg_messages
)

# Declare the SpaceMouseMapper executable
add_executable(SpaceMouseMapper_node src/spaceMouseMapper/spaceMouseMapper.cpp)
ament_target_dependencies(SpaceMouseMapper_node rclcpp cross_pkg_messages std_msgs)

# Declare the ArmCommandEncoder executable
add_executable(ArmCommandEncoder_node src/ArmCommandEncoder/main.cpp)
ament_target_dependencies(ArmCommandEncoder_node rclcpp cross_pkg_messages std_msgs control_msgs moveit_servo)

# Install and export
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  JoyMapper_node
  # TelemetryPanel_node
  GroundStationGUI
  SpaceMouseMapper_node
  ArmCommandEncoder_node
  # ArmCommandEncoder_node
  LUSIVisionStreamer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
