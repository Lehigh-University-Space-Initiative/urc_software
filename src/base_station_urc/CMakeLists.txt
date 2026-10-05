# base_station_urc: code that runs on the operator's base station laptop
#
# Builds:
#   JoyMapper_node - joysticks to /cmd_vel (src/joyMapper)
#   GroundStationGUI - the ImGui operator window (src/gui), compiled together with Dear ImGui from libs/imgui
#   LUSIVisionStreamer_node - video and telemetry over TCP for LUSI Vision (src/LUSIVisionStreamer)
#   SpaceMouseMapper_node - SpaceMouse to arm commands (src/spaceMouseMapper)
#   ArmCommandEncoder_node - embedded MoveIt Servo teleop (src/ArmCommandEncoder; built but not launched)

cmake_minimum_required(VERSION 3.5)
project(base_station_urc)

# Finding the ROS packages and libraries this package uses (each must also be listed in package.xml)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glfw3 REQUIRED)  # Window and input library for the GUI
find_package(cross_pkg_messages REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(sockpp REQUIRED)  # TCP socket library, built and installed into the image by the Dockerfile

# Header search paths shared by every target in this package
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

add_executable(JoyMapper_node src/joyMapper/joyMapper.cpp)
ament_target_dependencies(JoyMapper_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)
target_include_directories(JoyMapper_node PRIVATE src/base_station_urc/include/cs_libguarded)

# The GUI compiles Dear ImGui's sources directly (ImGui is meant to be built into the app, not installed)
# Note: file(GLOB) only re-scans when CMake reconfigures, so do a clean build after adding or removing a .cpp file
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
# glfw = window/input, GL = OpenGL drawing, dl = runtime library loading (dlopen), opencv_highgui = image I/O
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
     "src/LUSIVisionStreamer/*.h"
)
add_executable(LUSIVisionStreamer_node ${VisionFiles})
target_include_directories(LUSIVisionStreamer_node PRIVATE include/cs_libguarded)
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

add_executable(SpaceMouseMapper_node src/spaceMouseMapper/spaceMouseMapper.cpp)
ament_target_dependencies(SpaceMouseMapper_node rclcpp cross_pkg_messages std_msgs)

add_executable(ArmCommandEncoder_node src/ArmCommandEncoder/main.cpp)
ament_target_dependencies(ArmCommandEncoder_node rclcpp cross_pkg_messages std_msgs control_msgs moveit_servo)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Installing executables into lib/<package>, which is where `ros2 run` and launch files look for them
install(TARGETS
  JoyMapper_node
  GroundStationGUI
  SpaceMouseMapper_node
  ArmCommandEncoder_node
  LUSIVisionStreamer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
