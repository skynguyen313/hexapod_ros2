cmake_minimum_required(VERSION 3.8)
project(hexapod_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)

include_directories(include)
add_library(hexapod_controller_composition SHARED
            src/control.cpp
            src/gait.cpp
            src/ik.cpp
            src/servo_driver.cpp
            )
target_compile_definitions(hexapod_controller_composition
  PRIVATE "HEXAPOD_CONTROLLER_DLL")
ament_target_dependencies(hexapod_controller_composition rclcpp rclcpp_components geometry_msgs hexapod_msgs)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(hexapod_controller src/hexapod_controller.cpp)
target_link_libraries(hexapod_controller hexapod_controller_composition)
ament_target_dependencies(hexapod_controller rclcpp class_loader)

install(TARGETS
hexapod_controller_composition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  hexapod_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
