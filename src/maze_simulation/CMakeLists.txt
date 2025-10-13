cmake_minimum_required(VERSION 3.8)
project(maze_simulation)

find_package(ament_cmake REQUIRED)

# Install assets so gz can find them via the package share dir
install(DIRECTORY
  launch
  worlds
  models
  rviz
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# Install launch and config so ros2 launch can find them from install space
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
