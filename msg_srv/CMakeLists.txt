cmake_minimum_required(VERSION 3.5)
project(msg_srv)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  

# Declare ROS 2 messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Robot.msg"  # Custom message
  "srv/Controller.srv"  # Custom service
  DEPENDENCIES std_msgs  # Include any other dependencies here if needed
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()