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

# Declare the ArmMotorManager executable
add_executable(arm_node src/main.cpp src/ArmMotorManager.cpp src/MotorManager.cpp ../shared_code/pid.cpp src/CANDriver.cpp )
target_link_libraries(arm_node pigpio)
ament_target_dependencies(arm_node rclcpp cross_pkg_messages std_msgs)

# Add custom target to build pigpio
add_custom_target(pigpio_build ALL
  COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio
)

set(ARM_URC_SRC
  ../shared_code/pid.cpp
  src/ArmMotorManager.cpp
)

set(ARM_URC_HW_SRC
  src/ArmHardware.cpp
)

add_library(arm_urc SHARED ${ARM_URC_SRC})
add_dependencies(${PROJECT_NAME} pigpio_build)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
  cross_pkg_messages
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME} pigpio)

#add_library(arm_urc_hw SHARED ${ARM_URC_HW_SRC})
#set_target_properties(arm_urc_hw PROPERTIES OUTPUT_NAME "arm_urc_hw")
#ament_target_dependencies(arm_urc_hw
#  rclcpp
#  hardware_interface
#  pluginlib
#  cross_pkg_messages
#)
#target_link_libraries(arm_urc_hw pigpio arm_urc)


#pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS
  arm_urc
#  arm_urc_hw
  DESTINATION lib
)

#install(FILES plugin.xml
#  DESTINATION share/${PROJECT_NAME}
#)

#install(DIRECTORY
#  urdf
#  launch
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
