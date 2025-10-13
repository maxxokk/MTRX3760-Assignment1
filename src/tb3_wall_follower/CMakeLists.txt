cmake_minimum_required(VERSION 3.8)
project(tb3_wall_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(wall_follower src/wall_follower_node.cpp)
ament_target_dependencies(wall_follower rclcpp sensor_msgs geometry_msgs)

install(TARGETS wall_follower
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
