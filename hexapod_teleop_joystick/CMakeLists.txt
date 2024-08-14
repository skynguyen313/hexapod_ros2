cmake_minimum_required(VERSION 3.8)
project(hexapod_teleop_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(hexapod_teleop_joystick src/hexapod_teleop_joystick.cpp)
target_include_directories(hexapod_teleop_joystick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hexapod_teleop_joystick PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hexapod_teleop_joystick
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "geometry_msgs"
)

install(TARGETS hexapod_teleop_joystick
  DESTINATION lib/${PROJECT_NAME})

ament_package()
