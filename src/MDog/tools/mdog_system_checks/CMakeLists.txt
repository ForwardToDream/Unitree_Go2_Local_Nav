cmake_minimum_required(VERSION 3.8)
project(mdog_system_checks)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/mdog_topic_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
