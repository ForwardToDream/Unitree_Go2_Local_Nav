cmake_minimum_required(VERSION 3.8)
project(mdog_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mdog_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mdog_local_planner_node src/local_planner_node.cpp)
ament_target_dependencies(
  mdog_local_planner_node
  geometry_msgs
  mdog_interfaces
  rclcpp
)

install(
  TARGETS mdog_local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
