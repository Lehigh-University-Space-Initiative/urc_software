cmake_minimum_required(VERSION 3.5)
project(driveline_urc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories (Added path for cs_libguarded)
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cs_libguarded  # Include path for cs_plain_guarded.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio      # pigpio path
  ${rclcpp_INCLUDE_DIRS}
)

# Add custom target to build pigpio
add_custom_target(pigpio_build ALL
  COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio
)

# Motor Controller node executable
file(GLOB MotorCtr_src
     "src/MotorCtr/*.cpp"
)
add_executable(MotorCtr_node ${MotorCtr_src})
ament_target_dependencies(MotorCtr_node rclcpp std_msgs cross_pkg_messages)

# Ensure pigpio is built before MotorCtr_node
add_dependencies(MotorCtr_node pigpio_build)

# Link the pigpio library to MotorCtr_node
target_link_libraries(MotorCtr_node pigpio)

# GPS Driver node executable
add_executable(GpsDriver_node src/GpsDriver/main.cpp)
ament_target_dependencies(GpsDriver_node rclcpp std_msgs cross_pkg_messages)
target_link_libraries(GpsDriver_node gps)

# Install the executables
install(TARGETS
  MotorCtr_node
  GpsDriver_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
