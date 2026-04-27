cmake_minimum_required(VERSION 3.8)
project(mdog_body_marker_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mdog_body_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(mdog_body_marker_viz_node src/body_marker_viz_node.cpp)
ament_target_dependencies(mdog_body_marker_viz_node mdog_body_msgs rclcpp visualization_msgs)

install(TARGETS mdog_body_marker_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
