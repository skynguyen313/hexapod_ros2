cmake_minimum_required(VERSION 3.8)
project(hexapod_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/FeetPositions.msg"
  "msg/LegJoints.msg"
  "msg/LegsJoints.msg"
  "msg/Pose2D.msg"
  "msg/Pose3D.msg"
  "msg/RPY.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
