cmake_minimum_required(VERSION 3.8)
project(mdog_intent_arbitrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mdog_intent_msgs REQUIRED)
find_package(mdog_planning_msgs REQUIRED)
find_package(mdog_traversability_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mdog_intent_arbitrator_node src/intent_arbitrator_node.cpp)
ament_target_dependencies(mdog_intent_arbitrator_node
  geometry_msgs mdog_intent_msgs mdog_planning_msgs mdog_traversability_msgs rclcpp
)

install(TARGETS mdog_intent_arbitrator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
