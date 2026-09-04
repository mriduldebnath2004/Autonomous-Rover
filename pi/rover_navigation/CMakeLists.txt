cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
