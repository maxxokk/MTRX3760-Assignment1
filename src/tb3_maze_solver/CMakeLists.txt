cmake_minimum_required(VERSION 3.8)
project(tb3_maze_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(maze_solver_lib
  src/maze_solver.cpp
)
ament_target_dependencies(maze_solver_lib rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs)

add_executable(maze_solver_node src/maze_solver_node.cpp)
target_link_libraries(maze_solver_node maze_solver_lib)
ament_target_dependencies(maze_solver_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs)

install(TARGETS
  maze_solver_lib
  maze_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
