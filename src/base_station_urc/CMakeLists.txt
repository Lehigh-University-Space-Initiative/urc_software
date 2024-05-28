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
include_directories(${sensor_msgs_INCLUDE_DIRS})
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glfw3 REQUIRED)

# Add executable for JoyMapper
add_executable(JoyMapper_node src/joyMapper/joyMapper.cpp)
ament_target_dependencies(JoyMapper_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

#GUI
# file(GLOB GUIFiles
#      "src/gui/*.cpp"
#      "src/gui/panels/*.cpp"
#      "../../libs/imgui/*.cpp"
#      "../../libs/imgui/backends/imgui_impl_glfw.cpp"
#      "../../libs/imgui/backends/imgui_impl_opengl3.cpp"
# )

# add_executable(GroundStationGUI ${GUIFiles})
# target_include_directories(GroundStationGUI PRIVATE ../../libs/imgui ../../libs/imgui/backends include/cs_libguarded)
# target_link_libraries(GroundStationGUI ${catkin_LIBRARIES} glfw GL dl opencv_highgui)
# ament_target_dependencies(GroundStationGUI
#   rclcpp
#   std_msgs
#   sensor_msgs
#   image_transport
#   cv_bridge
# )

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
