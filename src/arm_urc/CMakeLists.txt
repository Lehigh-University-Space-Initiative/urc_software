cmake_minimum_required(VERSION 3.5)
project(arm_urc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories (Added path for cs_libguarded)
include_directories(
  include
  ../shared_code
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cs_libguarded  # Include path for cs_plain_guarded.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio      # pigpio path
  ${rclcpp_INCLUDE_DIRS}
)

# Add custom target to build pigpio
add_custom_target(pigpio_build ALL
  COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio
)

file(GLOB ARM_URC_SRC
  ../shared_code/*.cpp
  src/ArmMotorManager.cpp
  src/main.cpp
)

set(ARM_URC_HW_SRC
  src/ArmHardware.cpp
)

# Declare the SpaceMouseMapper executable
add_executable(ArmMotorManager ${ARM_URC_SRC})
ament_target_dependencies(ArmMotorManager rclcpp cross_pkg_messages std_msgs)
target_link_libraries(ArmMotorManager pigpio)

install(TARGETS
  # arm_urc
  # arm_urc_hw
  ArmMotorManager
  DESTINATION lib/${PROJECT_NAME}
)

# install(FILES plugin.xml
#   DESTINATION share/${PROJECT_NAME}
# )

install(DIRECTORY
  # urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
