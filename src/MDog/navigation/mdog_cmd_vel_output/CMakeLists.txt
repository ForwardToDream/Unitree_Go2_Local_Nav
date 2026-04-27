cmake_minimum_required(VERSION 3.8)
project(mdog_cmd_vel_output)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mdog_planning_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mdog_cmd_vel_output_node src/cmd_vel_output_node.cpp)
ament_target_dependencies(mdog_cmd_vel_output_node geometry_msgs mdog_planning_msgs rclcpp)

install(TARGETS mdog_cmd_vel_output_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
