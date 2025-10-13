#ifndef TB3_MAZE_SOLVER_MAZE_SOLVER_HPP
#define TB3_MAZE_SOLVER_MAZE_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>

namespace tb3_maze {

class MazeSolver : public rclcpp::Node {
public:
  explicit MazeSolver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MazeSolver() override = default;

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoopCallback();
  void computeCommand(geometry_msgs::msg::Twist & cmd);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double linear_speed_;
  double angular_speed_;
  double target_wall_distance_;
  double front_stop_distance_;
  int control_rate_hz_;

  std::vector<float> scan_;
  bool have_scan_{false};
  nav_msgs::msg::Path path_;
  bool have_odom_{false};

  static constexpr double DEFAULT_LINEAR_SPEED = 0.15;
  static constexpr double DEFAULT_ANGULAR_SPEED = 0.6;
  static constexpr double DEFAULT_WALL_DIST = 0.5;
  static constexpr double DEFAULT_FRONT_STOP = 0.3;
  static constexpr int DEFAULT_RATE_HZ = 20;
};

} // namespace tb3_maze
#endif
