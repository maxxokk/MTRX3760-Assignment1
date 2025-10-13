#include "maze_solver/maze_solver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace tb3_maze {

MazeSolver::MazeSolver(const rclcpp::NodeOptions & options)
: rclcpp::Node("tb3_maze_solver", options)
{
  this->declare_parameter<double>("linear_speed", DEFAULT_LINEAR_SPEED);
  this->declare_parameter<double>("angular_speed", DEFAULT_ANGULAR_SPEED);
  this->declare_parameter<double>("wall_distance", DEFAULT_WALL_DIST);
  this->declare_parameter<double>("front_stop_distance", DEFAULT_FRONT_STOP);
  this->declare_parameter<int>("control_rate_hz", DEFAULT_RATE_HZ);

  this->get_parameter("linear_speed", linear_speed_);
  this->get_parameter("angular_speed", angular_speed_);
  this->get_parameter("wall_distance", target_wall_distance_);
  this->get_parameter("front_stop_distance", front_stop_distance_);
  this->get_parameter("control_rate_hz", control_rate_hz_);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&MazeSolver::lidarCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&MazeSolver::odomCallback, this, std::placeholders::_1));

  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  path_.header.frame_id = "odom";

  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1, control_rate_hz_)));
  timer_ = this->create_wall_timer(period, std::bind(&MazeSolver::controlLoopCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "tb3_maze_solver up: v=%.2f w=%.2f wall=%.2f front=%.2f Hz=%d",
    linear_speed_, angular_speed_, target_wall_distance_, front_stop_distance_, control_rate_hz_);
}

void MazeSolver::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_ = msg->ranges;
  have_scan_ = true;
}

void MazeSolver::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  have_odom_ = true;
  geometry_msgs::msg::PoseStamped ps;
  ps.header = msg->header;
  ps.header.frame_id = "odom";
  ps.pose = msg->pose.pose;
  path_.header.stamp = msg->header.stamp;
  path_.poses.push_back(ps);
  if (path_.poses.size() > 2000) {
    path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 100);
  }
  path_pub_->publish(path_);
}

void MazeSolver::computeCommand(geometry_msgs::msg::Twist & cmd)
{
  if (!have_scan_ || scan_.empty()) { cmd.linear.x = 0.0; cmd.angular.z = 0.0; return; }

  const size_t N = scan_.size();
  auto clip = [](float v) {
    if (std::isnan(v) || std::isinf(v) || v <= 0.0f) return std::numeric_limits<float>::infinity();
    return v;
  };
  auto avg_region = [&](size_t center, int half_width)->float {
    int start = static_cast<int>(center) - half_width;
    int end   = static_cast<int>(center) + half_width;
    start = std::max(start, 0);
    end   = std::min(end, static_cast<int>(N) - 1);
    double sum = 0.0; int count = 0;
    for (int i = start; i <= end; ++i) {
      float d = clip(scan_[i]);
      if (std::isfinite(d)) { sum += d; ++count; }
    }
    return (count > 0) ? static_cast<float>(sum / count) : std::numeric_limits<float>::infinity();
  };

  size_t front_idx = N / 2;
  size_t right_idx = (3 * N) / 4;

  float d_front = avg_region(front_idx, 5);
  float d_right = avg_region(right_idx, 5);

  if (d_front < front_stop_distance_) {
    cmd.linear.x = 0.0;
    cmd.angular.z = angular_speed_;              // turn left to avoid
  } else if (d_right > target_wall_distance_ * 1.2) {
    cmd.linear.x = linear_speed_;
    cmd.angular.z = -0.5 * angular_speed_;       // steer right to reacquire wall
  } else if (d_right < target_wall_distance_ * 0.8) {
    cmd.linear.x = 0.05;                          // slow & bias left if too close
    cmd.angular.z = 0.5 * angular_speed_;
  } else {
    cmd.linear.x = linear_speed_;
    cmd.angular.z = 0.0;
  }
}

void MazeSolver::controlLoopCallback()
{
  geometry_msgs::msg::Twist cmd;
  computeCommand(cmd);
  vel_pub_->publish(cmd);
}

} // namespace tb3_maze
