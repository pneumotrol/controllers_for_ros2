cmake_minimum_required(VERSION 3.8)
project(state_feedback_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# create parameter management library from yaml file
generate_parameter_library(state_feedback_controller_parameters
  src/state_feedback_controller.yaml
)

# add target as library
add_library(state_feedback_controller
  SHARED src/state_feedback_controller.cpp
)
target_include_directories(state_feedback_controller
  PRIVATE include
)
target_link_libraries(state_feedback_controller
  state_feedback_controller_parameters
)
ament_target_dependencies(state_feedback_controller
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
target_compile_definitions(state_feedback_controller
  PRIVATE "STATE_FEEDBACK_CONTROLLER_BUILDING_DLL"
)

# export controllers as plugin
pluginlib_export_plugin_description_file(
  controller_interface state_feedback_controller.xml
)

install(
  TARGETS state_feedback_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_libraries(state_feedback_controller)

ament_package()
