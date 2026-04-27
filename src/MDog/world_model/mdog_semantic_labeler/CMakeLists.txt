cmake_minimum_required(VERSION 3.8)
project(mdog_semantic_labeler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mdog_grid_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mdog_semantic_labeler_node src/semantic_labeler_node.cpp)
ament_target_dependencies(mdog_semantic_labeler_node mdog_grid_msgs rclcpp)

install(TARGETS mdog_semantic_labeler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
