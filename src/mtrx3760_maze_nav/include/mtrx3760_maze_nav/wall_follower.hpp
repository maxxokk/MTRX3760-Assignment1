#pragma once
#include <optional>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mtrx3760_maze_nav/config.hpp"

class WallFollower {
public:
  struct Params {
    double desired_wall = cfg::DESIRED_WALL;
    double front_stop   = cfg::FRONT_STOP;
    double lost_wall    = cfg::LOST_WALL;
    double lin_speed    = cfg::LIN_SPEED;
    double max_ang_speed= cfg::MAX_ANG_SPEED;
    double kp           = cfg::KP;
  };

  enum class State { FOLLOW_WALL, TURN_CORNER, ADJUST_TO_WALL };

  WallFollower() = default;

  void set_params(const Params& p) { params_ = p; }
  void update_from_scan(const sensor_msgs::msg::LaserScan &scan);
  geometry_msgs::msg::Twist compute_cmd();

  // For debugging/telemetry
  State state() const { return state_; }
  std::optional<double> front() const { return front_; }
  std::optional<double> right() const { return right_; }
  std::optional<double> front_right() const { return front_right_; }

private:
  // Helper: min valid range within angle window
  static std::optional<double> min_in_window(
    const sensor_msgs::msg::LaserScan &scan, double center_rad, double half_width_rad);

  State state_{State::FOLLOW_WALL};
  std::optional<double> front_{};
  std::optional<double> right_{};
  std::optional<double> front_right_{};
  Params params_{};
};
