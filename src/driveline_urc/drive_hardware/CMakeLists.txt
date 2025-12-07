cmake_minimum_required(VERSION 3.5)
project(drive_hardware)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface OPTIONAL)
find_package(ros2_control REQUIRED)

add_library(drive_hardware SHARED
    src/DriveHardware.cpp
    src/CANDriver.cpp
    src/pluginRegistration.cpp
)

target_include_directories(drive_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(drive_hardware
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    ros2_control
)

pluginlib_export_plugin_description_file(hardware_interface plugins/drive_plugin.xml)

install(TARGETS drive_hardware
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins/rover_drive_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf/ config/ launch/
  DESTINATION share/${PROJECT_NAME}
)


# # Include directories
# include_directories(
#   include
#   ../shared_code
#   ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio      # pigpio path
#   ${rclcpp_INCLUDE_DIRS}
# )

# # Add custom target to build pigpio
# add_custom_target(pigpio_build ALL
#   COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/pigpio
# )

# # Motor Controller node executable
# file(GLOB MotorCtr_src
#   ../shared_code/*.cpp
#   src/*.cpp
# )
# add_executable(MotorCtr_node ${MotorCtr_src})
# ament_target_dependencies(MotorCtr_node rclcpp std_msgs cross_pkg_messages)

# # Ensure pigpio is built before MotorCtr_node
# add_dependencies(MotorCtr_node pigpio_build)

# # Link the pigpio library to MotorCtr_node
# target_link_libraries(MotorCtr_node pigpio)

# # Install the executables
# install(TARGETS
#   MotorCtr_node
#   DESTINATION lib/${PROJECT_NAME})

# # Install the launch directory
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
