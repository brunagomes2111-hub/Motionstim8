cmake_minimum_required(VERSION 3.8)
project(motionstim8_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# Dependencies
#

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#
# Driver library
#

add_library(${PROJECT_NAME} SHARED
  src/motionstim8_driver.cpp
  third_party/stimlib/motionstim8.cpp
  third_party/stimlib/serial_linux.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    _TTY_POSIX_
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/stimlib>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(motionstim8_driver
  rclcpp
)

install(
  DIRECTORY
    third_party/stimlib/

  DESTINATION include

  FILES_MATCHING
    PATTERN "*.h"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()