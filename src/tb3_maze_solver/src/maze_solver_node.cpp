#include "maze_solver/maze_solver.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tb3_maze::MazeSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
