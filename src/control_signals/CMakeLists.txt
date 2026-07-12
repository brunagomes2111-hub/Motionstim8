cmake_minimum_required(VERSION 3.8)
project(control_signals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(control_signals_node
  src/control_signals_node.cpp
  Generate_Reference/generate_reference_node.cpp
  Generate_Reference/trajectories.cpp
  Angle_Estimation/AngleEstimation_node.cpp
)

target_include_directories(
  control_signals_node
  PUBLIC
    Generate_Reference/include
    Angle_Estimation/include
)

ament_target_dependencies(
  control_signals_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS
    control_signals_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    Generate_Reference/include/
    Angle_Estimation/include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()