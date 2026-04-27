cmake_minimum_required(VERSION 3.8)
project(mdog_height_grid_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mdog_grid_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mdog_height_grid_builder_node src/height_grid_builder_node.cpp)
ament_target_dependencies(mdog_height_grid_builder_node mdog_grid_msgs rclcpp sensor_msgs)

install(TARGETS mdog_height_grid_builder_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
