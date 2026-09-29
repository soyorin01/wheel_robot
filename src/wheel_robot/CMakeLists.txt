cmake_minimum_required(VERSION 3.8)
project(wheel_robot)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleStatus.msg"
  "msg/StepApproachStatus.msg"
  "msg/StepDetection.msg"
  DEPENDENCIES std_msgs
)

# The generic C++ type-support library loads the selected DDS implementation
# (Fast DDS on this system) with dlopen().  Give the dispatcher an explicit
# sibling-library search path so a terminal opened before the latest build does
# not fail to find the newly generated implementation library.
set_target_properties(${PROJECT_NAME}__rosidl_typesupport_cpp PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

set(
  ORBBEC_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/OrbbecSDK_ROS2/orbbec_camera/SDK"
  CACHE PATH
  "Path to the Orbbec legacy SDK bundled with OrbbecSDK_ROS2 main branch"
)
if(NOT EXISTS "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
  message(FATAL_ERROR
    "Orbbec legacy SDK not found. Run ${CMAKE_CURRENT_SOURCE_DIR}/tools/fetch_orbbec_sdk.sh"
  )
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(ORBBEC_SDK_PLATFORM arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(ORBBEC_SDK_PLATFORM x64)
else()
  message(FATAL_ERROR "Unsupported Orbbec SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(ORBBEC_SDK_LIB_DIR "${ORBBEC_SDK_ROOT}/lib/${ORBBEC_SDK_PLATFORM}")
find_library(
  ORBBEC_SDK_LIBRARY
  NAMES OrbbecSDK
  PATHS "${ORBBEC_SDK_LIB_DIR}"
  NO_DEFAULT_PATH
  REQUIRED
)

add_executable(astra_camera_node
  src/astra_camera_node.cpp
)
target_compile_features(astra_camera_node PUBLIC cxx_std_17)
target_compile_options(astra_camera_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(astra_camera_node
  ${OpenCV_LIBRARIES}
  ${ORBBEC_SDK_LIBRARY}
)
target_include_directories(astra_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  "${ORBBEC_SDK_ROOT}/include"
)
set_target_properties(astra_camera_node PROPERTIES
  BUILD_RPATH "${ORBBEC_SDK_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN"
)
ament_target_dependencies(astra_camera_node
  camera_info_manager
  rclcpp
  sensor_msgs
)

add_executable(depth_distance_node
  src/depth_distance_node.cpp
)
target_compile_features(depth_distance_node PUBLIC cxx_std_17)
target_compile_options(depth_distance_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(depth_distance_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_distance_node ${OpenCV_LIBRARIES})
ament_target_dependencies(depth_distance_node
  rclcpp
  sensor_msgs
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(depth_distance_node "${cpp_typesupport_target}")
set_target_properties(depth_distance_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

# Python application nodes are kept in scripts/ to match the workspace layout.
install(PROGRAMS
  scripts/keyboard_teleop.py
  scripts/lessop16_opencv.py
  scripts/diagnostics_monitor.py
  scripts/leg_height_feedback.py
  scripts/step_detector_node.py
  scripts/step_preview.py
  scripts/step_approach_test.py
  scripts/camera_viewer.py
  scripts/camera_recorder.py
  scripts/depth_preview.py
  scripts/line_follower.py
  scripts/dual_line_follower.py
  scripts/icn.py
  scripts/shiwai.py
  scripts/lesson17_vision_follower.py
  scripts/lesson18_person_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/models
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  astra_camera_node
  depth_distance_node
  DESTINATION lib/${PROJECT_NAME}
)

file(GLOB ORBBEC_SDK_RUNTIME_LIBS "${ORBBEC_SDK_LIB_DIR}/*.so*")
install(FILES
  ${ORBBEC_SDK_RUNTIME_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)

# Future launch files and YAML parameters are installed automatically.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_lesson17_vision_follower.py")
    ament_add_pytest_test(
      test_lesson17_vision_follower
      test/test_lesson17_vision_follower.py
      TIMEOUT 60
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_lesson18_person_patrol.py")
    ament_add_pytest_test(
      test_lesson18_person_patrol
      test/test_lesson18_person_patrol.py
      TIMEOUT 60
    )
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
