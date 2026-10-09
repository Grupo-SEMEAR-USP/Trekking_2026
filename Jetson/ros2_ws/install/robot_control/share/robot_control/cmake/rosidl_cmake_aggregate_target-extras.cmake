# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target robot_control::robot_control
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${robot_control_TARGETS}.
if(robot_control_TARGETS AND NOT TARGET robot_control::robot_control)
  add_library(robot_control::robot_control INTERFACE IMPORTED)
  set_target_properties(robot_control::robot_control PROPERTIES
    INTERFACE_LINK_LIBRARIES "${robot_control_TARGETS}")
endif()
