cmake_minimum_required(VERSION 3.5)
project(driveline_urc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Motor Controller node executable
# file(GLOB MotorCtr_src
#      "src/MotorCtr/*.cpp"
# )

# add_executable(MotorCtr_node ${MotorCtr_src})
# ament_target_dependencies(MotorCtr_node rclcpp std_msgs cross_pkg_messages)
# target_link_libraries(MotorCtr_node pigpio)

# GPS Driver node executable
add_executable(GpsDriver_node src/GpsDriver/main.cpp)
ament_target_dependencies(GpsDriver_node rclcpp std_msgs cross_pkg_messages)
target_link_libraries(GpsDriver_node gps)

# Install the executables
install(TARGETS
#   MotorCtr_node
  GpsDriver_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
