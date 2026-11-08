cmake_minimum_required(VERSION 3.8)
project(drivetrain_2027_urc)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/dcs_stage1.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/dcs_core.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(stage1_core
    COMMAND "${Python3_EXECUTABLE}" -m pytest "${CMAKE_CURRENT_SOURCE_DIR}/test/test_core.py" -v -p no:cacheprovider
      "--junitxml=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/core.xunit.xml"
    RESULT_FILE "${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/core.xunit.xml"
    TIMEOUT 60)
  add_launch_test(test/test_stage1_launch.py TIMEOUT 90)
endif()
ament_package()
