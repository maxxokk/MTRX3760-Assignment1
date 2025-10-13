#include <cmath>
#include <limits>
#include "mtrx3760_maze_nav/wall_follower.hpp"
#include "mtrx3760_maze_nav/config.hpp"

using sensor_msgs::msg::LaserScan;
using geometry_msgs::msg::Twist;

static inline bool is_valid(double r) {
  return std::isfinite(r) && r > 0.02;
}

// shortest signed angle difference a-b in (-pi, pi]
static inline double ang_diff(double a, double b) {
  double d = a - b;
  while (d >  M_PI) d -= 2.0*M_PI;
  while (d <= -M_PI) d += 2.0*M_PI;
  return d;
}

std::optional<double> WallFollower::min_in_window(
  const LaserScan &scan, double center_rad, double half_width_rad)
{
  const size_t n = scan.ranges.size();
  if (n == 0 || scan.angle_increment == 0.0) return std::nullopt;

  double best = std::numeric_limits<double>::infinity();
  bool any = false;

  // Iterate all rays and keep those whose angle is within half_width of center,
  // using wrap-safe angular difference so it works for 0..2π or -π..π scans.
  for (size_t i = 0; i < n; ++i) {
    const double ang = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (std::fabs(ang_diff(ang, center_rad)) <= half_width_rad) {
      const double r = scan.ranges[i];
      if (is_valid(r) && r < best) { best = r; any = true; }
    }
  }
  if (!any) return std::nullopt;
  return best;
}

void WallFollower::update_from_scan(const LaserScan &scan) {
  front_       = min_in_window(scan, cfg::FRONT_ANGLE,       cfg::WINDOW);
  front_right_ = min_in_window(scan, cfg::FRONT_RIGHT_ANGLE, cfg::WINDOW);
  right_       = min_in_window(scan, cfg::RIGHT_ANGLE,       cfg::WINDOW);

  const double f = front_.value_or(std::numeric_limits<double>::infinity());
  const double r = right_.value_or(std::numeric_limits<double>::infinity());

  switch (state_) {
    case State::FOLLOW_WALL:
      if (f < params_.front_stop) {
        state_ = State::TURN_CORNER;
      } else if (r > params_.lost_wall) {
        state_ = State::ADJUST_TO_WALL;
      }
      break;

    case State::TURN_CORNER:
      if (f >= params_.front_stop * 1.2 && r < params_.lost_wall) {
        state_ = State::FOLLOW_WALL;
      }
      break;

    case State::ADJUST_TO_WALL:
      if (r <= params_.desired_wall * 1.2) {
        state_ = State::FOLLOW_WALL;
      } else if (f < params_.front_stop) {
        state_ = State::TURN_CORNER;
      }
      break;
  }
}

geometry_msgs::msg::Twist WallFollower::compute_cmd() {
  Twist t;
  if (!front_.has_value() && !right_.has_value()) return t;

  const double f = front_.value_or(std::numeric_limits<double>::infinity());
  const double r = right_.value_or(std::numeric_limits<double>::infinity());

  switch (state_) {
    case State::FOLLOW_WALL: {
      t.linear.x = params_.lin_speed;
      const double err = params_.desired_wall - r; // >0 => too far from wall => steer right
      double w = params_.kp * err;
      if (w >  params_.max_ang_speed) w =  params_.max_ang_speed;
      if (w < -params_.max_ang_speed) w = -params_.max_ang_speed;
      t.angular.z = w;
      break;
    }
    case State::TURN_CORNER: {
      t.linear.x  = 0.0;
      t.angular.z = +params_.max_ang_speed * 0.9;
      break;
    }
    case State::ADJUST_TO_WALL: {
      t.linear.x  = params_.lin_speed * 0.6;
      t.angular.z = -params_.max_ang_speed * 0.5;
      if (f < params_.front_stop) {
        t.linear.x  = 0.0;
        t.angular.z = +params_.max_ang_speed * 0.8;
      }
      break;
    }
  }
  return t;
}
