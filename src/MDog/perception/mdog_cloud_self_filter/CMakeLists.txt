cmake_minimum_required(VERSION 3.8)
project(mdog_cloud_self_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mdog_body_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mdog_cloud_self_filter_node src/cloud_self_filter_node.cpp)
ament_target_dependencies(mdog_cloud_self_filter_node mdog_body_msgs rclcpp sensor_msgs)

install(TARGETS mdog_cloud_self_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
