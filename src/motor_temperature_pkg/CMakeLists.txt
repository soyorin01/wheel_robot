cmake_minimum_required(VERSION 3.8)
project(motor_temperature_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorTemperatures.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/motor_temperature_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_temperature_monitor
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
