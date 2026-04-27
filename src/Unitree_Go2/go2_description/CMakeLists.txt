cmake_minimum_required(VERSION 3.8)
project(go2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)

add_library(go2_kinematic_planar_move SHARED
  src/go2_kinematic_planar_move.cpp
)
ament_target_dependencies(go2_kinematic_planar_move
  gazebo_dev
  gazebo_ros
  geometry_msgs
  rclcpp
)

install(
  TARGETS go2_kinematic_planar_move
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(description_resource_dirs
  urdf
  dae
)

foreach(resource_dir IN LISTS description_resource_dirs)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_dir}")
    install(
      DIRECTORY ${resource_dir}/
      DESTINATION share/${PROJECT_NAME}/${resource_dir}
    )
  endif()
endforeach()

ament_package()
