# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target motor_temperature_pkg::motor_temperature_pkg
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${motor_temperature_pkg_TARGETS}.
if(motor_temperature_pkg_TARGETS AND NOT TARGET motor_temperature_pkg::motor_temperature_pkg)
  add_library(motor_temperature_pkg::motor_temperature_pkg INTERFACE IMPORTED)
  set_target_properties(motor_temperature_pkg::motor_temperature_pkg PROPERTIES
    INTERFACE_LINK_LIBRARIES "${motor_temperature_pkg_TARGETS}")
endif()
