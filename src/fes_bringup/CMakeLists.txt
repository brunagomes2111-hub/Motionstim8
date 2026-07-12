cmake_minimum_required(VERSION 3.8)
project(fes_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Configuration.msg"
)

add_executable(configuration_node
  src/configuration_node.cpp
)

target_include_directories(configuration_node PUBLIC
  include
)

ament_target_dependencies(configuration_node
  rclcpp
  std_msgs
  sensor_msgs
  controller_manager_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(configuration_node
  "${cpp_typesupport_target}"
)

install(TARGETS
  configuration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    urdf
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()