# arm_urc: code that runs on the arm Raspberry Pi
#
# Builds:
#   ArmMotorManager - drives the arm joints and gripper over CAN (src/), plus the shared rover code in ../shared_code

cmake_minimum_required(VERSION 3.5)
project(arm_urc)

# Finding the ROS packages this package uses (each must also be listed in package.xml)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(sensor_msgs REQUIRED)

# pigpio (Raspberry Pi GPIO library) is built and installed into the Docker image by the Dockerfile ("make install")
# It used to be rebuilt here too, by a custom target running "make -C libs/pigpio" in the shared source folder
# driveline_urc had the same target, and colcon builds both packages at once
# Two makes then wrote libpigpio.so at the same time and corrupted it ("file truncated"), failing clean builds
find_library(PIGPIO_LIBRARY pigpio)
if(NOT PIGPIO_LIBRARY)
  message(FATAL_ERROR "pigpio library not found. Build inside the urc_software_builder Docker image (./softwareUpdate/dockerBuild.sh), or install pigpio: cd libs/pigpio && make && sudo make install")
endif()

include_directories(
  include
  ../shared_code
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cs_libguarded  # Vendored thread-safety helpers (cs_plain_guarded.h)
  ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio  # pigpio.h
  ${rclcpp_INCLUDE_DIRS}
)

# Compiling the shared rover code (CAN driver, MotorManager, PID) directly into this executable
# Note: file(GLOB) only re-scans when CMake reconfigures, so do a clean build after adding or removing a .cpp file
file(GLOB ARM_URC_SRC
  ../shared_code/*.cpp
  src/ArmMotorManager.cpp
  src/main.cpp
)

add_executable(ArmMotorManager ${ARM_URC_SRC})
ament_target_dependencies(ArmMotorManager rclcpp cross_pkg_messages std_msgs)
target_link_libraries(ArmMotorManager ${PIGPIO_LIBRARY})

# Installing into lib/<package>, which is where `ros2 run` and launch files look for executables
install(TARGETS
  ArmMotorManager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
