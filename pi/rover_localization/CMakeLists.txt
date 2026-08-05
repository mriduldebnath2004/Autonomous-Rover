cmake_minimum_required(VERSION 3.8)
project(rover_localization)

find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
