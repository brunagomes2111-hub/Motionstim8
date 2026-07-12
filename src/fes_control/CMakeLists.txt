cmake_minimum_required(VERSION 3.8)
project(fes_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# find_package(<dependency> REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)

find_package(pluginlib REQUIRED)
find_package(control_toolbox REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(fes_bringup REQUIRED)

#
# Controller
#

add_library(fes_controller SHARED
  src/fes_controller.cpp
)

target_include_directories(
  fes_controller
  PUBLIC 
    include
)

ament_target_dependencies(
  fes_controller
  hardware_interface
  controller_interface
  pluginlib
  control_toolbox
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  fes_bringup
)

pluginlib_export_plugin_description_file(
  controller_interface
  controller_plugin.xml
)

install(
  TARGETS
    fes_controller

  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  FILES
    controller_plugin.xml

  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/

  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
