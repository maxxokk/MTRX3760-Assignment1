cmake_minimum_required(VERSION 3.8)
project(mtrx3760_maze_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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

ament_package()

add_executable(maze_navigator src/maze_navigator.cpp src/wall_follower.cpp)
ament_target_dependencies(maze_navigator rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs visualization_msgs)
install(TARGETS maze_navigator DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# expose headers to the target and for install
target_include_directories(maze_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# install public headers
install(DIRECTORY include/ DESTINATION include)

add_executable(trajectory_node src/trajectory_node.cpp)
ament_target_dependencies(trajectory_node rclcpp nav_msgs visualization_msgs geometry_msgs)
install(TARGETS trajectory_node DESTINATION lib/${PROJECT_NAME})
