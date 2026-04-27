cmake_minimum_required(VERSION 3.8)
project(mdog_owner_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mdog_body_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mdog_owner_model_node src/owner_model_node.cpp)
ament_target_dependencies(mdog_owner_model_node mdog_body_msgs rclcpp)

install(TARGETS mdog_owner_model_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
