cmake_minimum_required(VERSION 3.8)
project(mdog_traversability_evaluator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mdog_grid_msgs REQUIRED)
find_package(mdog_traversability_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mdog_traversability_evaluator_node src/traversability_evaluator_node.cpp)
ament_target_dependencies(mdog_traversability_evaluator_node
  mdog_grid_msgs mdog_traversability_msgs rclcpp
)

install(TARGETS mdog_traversability_evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
