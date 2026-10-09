cmake_minimum_required(VERSION 3.8)
project(robot_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/movement.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()