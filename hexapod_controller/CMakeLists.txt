cmake_minimum_required(VERSION 3.8)
project(hexapod_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hexapod_msgs REQUIRED)

# include directories
include_directories(include ${hexapod_msgs_INCLUDE_DIRS})

add_executable(hexapod_controller src/hexapod_controller.cpp)
ament_target_dependencies(hexapod_controller rclcpp hexapod_msgs)


# add libraries
add_library(ik SHARED src/ik.cpp)
add_library(gait SHARED src/gait.cpp)

target_link_libraries(hexapod_controller ik gait)

install(TARGETS 
  hexapod_controller
  DESTINATION lib/${PROJECT_NAME})


ament_package()
