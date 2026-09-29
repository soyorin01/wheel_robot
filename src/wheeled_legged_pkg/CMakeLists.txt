cmake_minimum_required(VERSION 3.8)
project(wheeled_legged_pkg)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(motor_temperature_pkg REQUIRED)

# 创建可执行文件
add_executable(wl_base_node
src/wl_base_node.cpp
src/robot_functions.cpp )

# 指定依赖
ament_target_dependencies(wl_base_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  diagnostic_msgs
  motor_temperature_pkg
)

# 安装可执行文件
install(TARGETS wl_base_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
