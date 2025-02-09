cmake_minimum_required(VERSION 3.5)
project(shared_code_urc)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
