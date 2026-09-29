# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target wheel_robot::wheel_robot
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${wheel_robot_TARGETS}.
if(wheel_robot_TARGETS AND NOT TARGET wheel_robot::wheel_robot)
  add_library(wheel_robot::wheel_robot INTERFACE IMPORTED)
  set_target_properties(wheel_robot::wheel_robot PROPERTIES
    INTERFACE_LINK_LIBRARIES "${wheel_robot_TARGETS}")
endif()
