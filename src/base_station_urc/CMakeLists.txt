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

# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include/cs_libguarded
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cross_pkg_messages_INCLUDE_DIRS}
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

# Declare the DriveTrainManager executable
add_executable(SpaceMouseMapper_node src/spaceMouseMapper/spaceMouseMapper.cpp)
ament_target_dependencies(SpaceMouseMapper_node rclcpp cross_pkg_messages std_msgs)

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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
