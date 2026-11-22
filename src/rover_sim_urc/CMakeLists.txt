# Installing the isolated Gazebo drivetrain test alongside the existing rover models
cmake_minimum_required(VERSION 3.8)
project(rover_sim_urc)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config description launch models worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/verify_assignment1.py scripts/command_bridge_2027.py scripts/command_gate_2027.py
  scripts/verify_command_bridge_2027.py scripts/four_wheel_bridge.py scripts/four_wheel_gate.py
  scripts/gate_health_monitor.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_assignment1_launch.py TIMEOUT 180)
  option(ROVER_SIM_TEST_2027_COMMAND_BRIDGE "Test the generated 2027 message bridge" OFF)
  if(ROVER_SIM_TEST_2027_COMMAND_BRIDGE)
    add_launch_test(test/test_command_bridge_2027_launch.py TIMEOUT 180)
    add_launch_test(test/test_command_bridge_failure_launch.py TIMEOUT 90)
    add_launch_test(test/test_command_gate_launch.py TIMEOUT 180)
    add_launch_test(test/test_command_gate_startup_launch.py TIMEOUT 90)
  endif()
  option(ROVER_SIM_TEST_FOUR_WHEEL "Test isolated four-wheel velocity control" OFF)
  if(ROVER_SIM_TEST_FOUR_WHEEL)
    add_launch_test(test/test_four_wheel_launch.py TIMEOUT 180)
    add_launch_test(test/test_gate_health_monitor_launch.py TIMEOUT 60)
    add_launch_test(test/test_gate_health_failure_launch.py TIMEOUT 120)
    add_launch_test(test/test_ros_cleanup_launch.py TIMEOUT 90)
    add_launch_test(test/test_four_wheel_shutdown_launch.py TIMEOUT 240)
  endif()
endif()

ament_package()
