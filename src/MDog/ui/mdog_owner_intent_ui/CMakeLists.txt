cmake_minimum_required(VERSION 3.8)
project(mdog_owner_intent_ui)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/mdog_owner_intent_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
