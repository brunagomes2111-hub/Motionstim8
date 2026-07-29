cmake_minimum_required(VERSION 3.8)
project(motionstim8_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# Dependencies
#

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(motionstim8_driver REQUIRED)
find_package(fes_bringup REQUIRED)

#
# Hardware interface
#

add_library(${PROJECT_NAME} SHARED
  src/motionstim8_hardware.cpp
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
    _TTY_POSIX_
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  pluginlib
  hardware_interface
  motionstim8_driver
  fes_bringup
)

pluginlib_export_plugin_description_file(
  hardware_interface
  motionstim8_hardware_plugin.xml
)

#
# Installation
#

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  FILES
    motionstim8_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()