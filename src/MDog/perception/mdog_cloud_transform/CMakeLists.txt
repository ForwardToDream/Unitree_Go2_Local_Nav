cmake_minimum_required(VERSION 3.8)
project(mdog_cloud_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mdog_cloud_transform_node src/cloud_transform_node.cpp)
ament_target_dependencies(mdog_cloud_transform_node
  rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros
)

install(TARGETS mdog_cloud_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
