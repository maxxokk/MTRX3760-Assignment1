#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

class RightWallFollower : public rclcpp::Node {
public:
  RightWallFollower() : Node("right_wall_follower") {
    target_right_ = declare_parameter<double>("target_right", 0.45);
    front_clear_  = declare_parameter<double>("front_clear", 0.55);
    kp_           = declare_parameter<double>("kp", 1.2);
    max_ang_      = declare_parameter<double>("max_ang", 1.8);
    fwd_          = declare_parameter<double>("fwd", 0.18);
    fwd_slow_     = declare_parameter<double>("fwd_slow", 0.10);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan &msg){ onScan(msg); });
  }

private:
  int idx(const sensor_msgs::msg::LaserScan &s, double ang) {
    int i = static_cast<int>(std::round((ang - s.angle_min)/s.angle_increment));
    return std::clamp(i, 0, static_cast<int>(s.ranges.size()) - 1);
  }
  double safe(double r) const { return std::isfinite(r) ? r : 10.0; }
  double clamp(double v, double lo, double hi) const { return std::max(lo, std::min(v, hi)); }

  void onScan(const sensor_msgs::msg::LaserScan &s) {
    // Forward and right samples
    const double fwd = safe(s.ranges[idx(s, 0.0)]);
    const double r90 = safe(s.ranges[idx(s, -M_PI/2.0)]);
    const double r60 = safe(s.ranges[idx(s, -M_PI/3.0)]);
    const double r30 = safe(s.ranges[idx(s, -M_PI/6.0)]);
    const double right = std::min({r90, r60, r30});

    geometry_msgs::msg::Twist cmd;
    if (fwd < front_clear_) {
      cmd.linear.x  = fwd_slow_;
      cmd.angular.z = 0.9 * max_ang_;     // turn left to clear obstacle
    } else {
      double err = target_right_ - right; // positive if too far from right wall
      cmd.angular.z = clamp(-kp_ * err, -max_ang_, max_ang_);
      cmd.linear.x  = fwd_;
    }
    pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  double target_right_, front_clear_, kp_, max_ang_, fwd_, fwd_slow_;
};
