# driveline_urc: code that runs on the driveline Raspberry Pi
#
# Builds:
#   MotorCtr_node - drives the six wheel motors over CAN (src/), plus the shared rover code in ../shared_code

cmake_minimum_required(VERSION 3.5)
project(driveline_urc)

# Finding the ROS packages this package uses (each must also be listed in package.xml)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cross_pkg_messages REQUIRED)
find_package(sensor_msgs REQUIRED)

# pigpio is installed into the Docker image by the Dockerfile; see arm_urc/CMakeLists.txt for why it isn't rebuilt here
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

# Compiling the shared rover code (CAN driver, MotorManager, PID) together with this package's sources
# Note: file(GLOB) only re-scans when CMake reconfigures, so do a clean build after adding or removing a .cpp file
file(GLOB MotorCtr_src
  ../shared_code/*.cpp
  src/*.cpp
)
add_executable(MotorCtr_node ${MotorCtr_src})
ament_target_dependencies(MotorCtr_node rclcpp std_msgs cross_pkg_messages)
target_link_libraries(MotorCtr_node ${PIGPIO_LIBRARY})

# Installing into lib/<package>, which is where `ros2 run` and launch files look for executables
install(TARGETS
  MotorCtr_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
