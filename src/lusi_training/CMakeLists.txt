cmake_minimum_required(VERSION 3.5)
project(lusi_training)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


# Include directories
# include_directories(
#   include
#   ${PROJECT_SOURCE_DIR}/include/cs_libguarded
#   ${rclcpp_INCLUDE_DIRS}
#   ${sensor_msgs_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${std_msgs_INCLUDE_DIRS}
#   ${cross_pkg_messages_INCLUDE_DIRS}
#   ${control_msgs_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/sockpp 
#   ../../libs/imgui
#   ../../libs/imgui/backends
# )

# Add executable
add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node
  rclcpp
)

install(TARGETS
  hello_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install and export
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
