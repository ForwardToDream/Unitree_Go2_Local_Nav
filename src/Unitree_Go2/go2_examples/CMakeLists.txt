cmake_minimum_required(VERSION 3.8)
project(go2_guide_dog_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include include/common include/nlohmann)
link_directories(src)

set(
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(read_low_state src/telemetry/read_low_state.cpp)
add_executable(read_motion_state src/telemetry/read_motion_state.cpp)
add_executable(read_wireless_controller src/telemetry/read_wireless_controller.cpp)
add_executable(go2_sport_client src/control/go2_sport_client.cpp src/common/ros2_sport_client.cpp)
add_executable(go2_stand_example src/control/go2_stand_example.cpp src/common/motor_crc.cpp)
add_executable(go2_robot_state_client src/control/go2_robot_state_client.cpp)
add_executable(guide_dog_manager src/guide_dog/guide_dog_manager.cpp src/common/ros2_sport_client.cpp)

target_compile_features(go2_sport_client PRIVATE cxx_std_20)
target_compile_features(go2_stand_example PRIVATE cxx_std_20)
target_compile_features(go2_robot_state_client PRIVATE cxx_std_20)
target_compile_features(guide_dog_manager PRIVATE cxx_std_20)

ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(go2_stand_example ${DEPENDENCY_LIST})
ament_target_dependencies(go2_robot_state_client ${DEPENDENCY_LIST})
ament_target_dependencies(guide_dog_manager ${DEPENDENCY_LIST})

install(
  TARGETS
    read_low_state
    read_motion_state
    read_wireless_controller
    go2_sport_client
    go2_stand_example
    go2_robot_state_client
    guide_dog_manager
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
