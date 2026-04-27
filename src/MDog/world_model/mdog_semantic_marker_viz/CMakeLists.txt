cmake_minimum_required(VERSION 3.8)
project(mdog_semantic_marker_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mdog_grid_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(mdog_semantic_marker_viz_node src/semantic_marker_viz_node.cpp)
ament_target_dependencies(mdog_semantic_marker_viz_node
  geometry_msgs mdog_grid_msgs rclcpp std_msgs visualization_msgs
)

install(TARGETS mdog_semantic_marker_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
