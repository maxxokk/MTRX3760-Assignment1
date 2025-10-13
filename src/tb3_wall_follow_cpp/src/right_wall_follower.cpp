#include "tb3_wall_follow_cpp/right_wall_follower.hpp"
#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightWallFollower>());
  rclcpp::shutdown();
  return 0;
}
