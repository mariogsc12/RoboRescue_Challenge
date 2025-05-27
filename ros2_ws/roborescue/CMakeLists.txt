cmake_minimum_required(VERSION 3.8)
project(roborescue)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
  ${PROJECT_NAME}/utils.py
  ${PROJECT_NAME}/drawer_manager.py
  ${PROJECT_NAME}/turtle_manager.py
  ${PROJECT_NAME}/goto_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoTo.action"
  DEPENDENCIES geometry_msgs 
)

ament_package()
