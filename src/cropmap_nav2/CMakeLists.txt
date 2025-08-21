cmake_minimum_required(VERSION 3.5)
project(cropmap_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  rviz2
  DESTINATION share/${PROJECT_NAME})

ament_package()

# Install Python scripts
install(PROGRAMS
  scripts/safety_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)
