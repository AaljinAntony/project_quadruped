cmake_minimum_required(VERSION 3.5)
project(spotmicro_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/invert_vel.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()