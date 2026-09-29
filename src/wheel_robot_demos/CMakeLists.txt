cmake_minimum_required(VERSION 3.8)
project(wheel_robot_demos)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/dance_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
