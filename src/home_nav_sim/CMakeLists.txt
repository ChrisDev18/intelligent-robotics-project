cmake_minimum_required(VERSION 3.8)
project(home_nav_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
