// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// http://www.apache.org/licenses/LICENSE-2.0
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Extended by: (you) — wall following (left/right selectable) + camera end-marker detection

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

#include <memory>
#include <algorithm>  // std::clamp
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Image + OpenCV
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

// Extra msgs
#include <std_msgs/msg/bool.hpp>

// === Choose which wall to follow ===
static constexpr bool FOLLOW_RIGHT = true; // set to false for LEFT-wall following

// Motion constants
static constexpr double LINEAR_VELOCITY  = 0.2; // m/s
static constexpr double ANGULAR_VELOCITY = 0.6; // rad/s

using namespace std::chrono_literals;

/* -------------------------------------------------------------------------- */
/*  Parameters (you can also expose these via declare_parameter if preferred)  */
/* -------------------------------------------------------------------------- */

// Camera topic (change if needed)
static constexpr const char* kCameraTopic = "/camera/image_raw";

// HSV bounds for the finish marker colour (example: bright GREEN).
// H: 0..179, S:0..255, V:0..255 (OpenCV HSV)
static const cv::Scalar kHSVLower = cv::Scalar(45,  80, 80);   // ~green lower
static const cv::Scalar kHSVUpper = cv::Scalar(85, 255, 255);  // ~green upper

// Minimal fraction of image pixels that must match to count as "seen"
static constexpr double kMinAreaFraction = 0.10; // 10%

// Optional morphology to clean the mask
static constexpr int kMorphKernel = 3;

/* ------------------------------ Class Impl --------------------------------- */

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  end_seen_ = false;
  end_announced_ = false;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Velocity publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Finish flag publisher
  finish_pub_ = this->create_publisher<std_msgs::msg::Bool>("/maze/finished", 1);

  // Laser and odom subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot3Drive::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  // Camera subscriber (raw image)
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    kCameraTopic, rclcpp::SensorDataQoS(),
    std::bind(&Turtlebot3Drive::image_callback, this, std::placeholders::_1));

  /************************************************************
  ** Timer
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 node initialised (wall-follow %s + camera finish).",
              FOLLOW_RIGHT ? "RIGHT" : "LEFT");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node terminated");
}

/********************************************************************************
** Callback functions
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto fetch = [&](double deg)->double {
    while (deg < 0.0)   deg += 360.0;
    while (deg >= 360.) deg -= 360.0;
    const double rad  = deg * M_PI/180.0;
    const double idxf = (rad - msg->angle_min) / msg->angle_increment;
    int idx = static_cast<int>(std::lround(idxf));
    idx = std::max(0, std::min(idx, static_cast<int>(msg->ranges.size()) - 1));
    double r = msg->ranges[idx];
    return std::isfinite(r) ? r : msg->range_max;
  };

  // Keep your originals
  scan_data_[CENTER] = fetch(0.0);     // front
  scan_data_[RIGHT]  = fetch(270.0);   // side (still available if you want it)

  // New: two side rays around 270° for flat-wall geometry
  side_r_285_ = fetch(285.0);          // 15° ahead on the right
  side_r_255_ = fetch(255.0);          // 15° behind on the right

  // If you use scan gating:
  // last_scan_time_ = msg->header.stamp;
  // scan_dirty_.store(true, std::memory_order_relaxed);
}



void Turtlebot3Drive::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Already done? Keep it cheap—skip processing once the end has been seen.
  if (end_seen_) return;

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "cv_bridge conversion failed: %s", e.what());
    return;
  }

  const cv::Mat& bgr = cv_ptr->image;
  if (bgr.empty()) return;

  // Convert to HSV and threshold
  cv::Mat hsv, mask;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, kHSVLower, kHSVUpper, mask);

  if (kMorphKernel > 1) {
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kMorphKernel, kMorphKernel));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
  }

  const double pixels = static_cast<double>(mask.rows) * static_cast<double>(mask.cols);
  if (pixels <= 0.0) return;

  const double on = static_cast<double>(cv::countNonZero(mask));
  const double frac = on / pixels;

  if (frac >= kMinAreaFraction) {
    end_seen_ = true; // latch
    RCLCPP_INFO(this->get_logger(),
      "Finish marker detected (area fraction=%.3f). Declaring success and stopping.", frac);
  }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub_->publish(cmd_vel);
}
/********************************************************************************
** Control loop
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  // Stop permanently if finish marker was seen
  if (end_seen_) {
    update_cmd_vel(0.0, 0.0);
    if (!end_announced_) {
      std_msgs::msg::Bool msg; msg.data = true;
      finish_pub_->publish(msg);
      end_announced_ = true;
      RCLCPP_INFO(this->get_logger(), "Maze finished! (/maze/finished published)");
    }
    return;
  }

  // ---------------- Tunables ----------------
  const double D_DES        = 0.40;              // desired right-wall clearance [m]
  const double BAND         = 0.10;              // allowed band ±
  const double EXIT_MARGIN  = 0.03;              // band hysteresis
  const double THETA_TOL    = 7.0 * M_PI/180.0;  // "parallel" tolerance

  const double V_STRAIGHT   = LINEAR_VELOCITY;
  const double V_NUDGE      = 0.12;
  const double W_NUDGE      = 0.30;

  const double K_THETA      = 1.2;               // align gain
  const double W_ALIGN_MAX  = 0.60;

  const double FRONT_STOP   = 0.32;              // nose safety
  const double FRONT_CLEAR  = 0.45;

  // Corner rounding (right)
  const double PRESENT_MAX  = 0.8;              // both rays < this ⇒ wall present
  const double OPENING_MIN  = 1.10;              // ray > this ⇒ opening candidate
  const int    OPEN_CONFIRM = 2;                 // scans to confirm opening
  const double CORNER_V     = 0.16;              // forward while rounding
  const double CORNER_W     = 0.45;              // right turn while rounding
  const double CORNER_MAX_S = 7.0;               // safety timeout (s)
  const double REACQ_THETA  = 10.0 * M_PI/180.0; // parallel-ish to exit corner

  // ---------------- Read measurements ----------------
  const double d_front = scan_data_[CENTER];
  const double r_n     = side_r_285_;   // 285° (forward-right)
  const double r_f     = side_r_255_;   // 255° (back-right)

  // If side rays invalid, just perform a gentle left spin to recover
  if (!(std::isfinite(r_n) && r_n > 0.0 && std::isfinite(r_f) && r_f > 0.0)) {
    update_cmd_vel(0.0, +0.4);
    return;
  }

  // Convert side rays to points
  const double a_n = 285.0 * M_PI/180.0;
  const double a_f = 255.0 * M_PI/180.0;
  const double x1 = r_n * std::cos(a_n), y1 = r_n * std::sin(a_n);
  const double x2 = r_f * std::cos(a_f), y2 = r_f * std::sin(a_f);

  // Perp distance to wall line
  const double cross = std::abs(x1*y2 - y1*x2);
  const double denom = std::hypot(x2 - x1, y2 - y1);
  const double d_perp = (denom > 1e-6) ? (cross / denom) : std::min(r_n, r_f);

  // Wall angle (sign: r_n < r_f → yawed into wall → need LEFT; r_n > r_f → away → RIGHT)
  const double theta_err = std::atan2(r_n - r_f, (30.0 * M_PI/180.0));

  // ---------------- Small state machine ----------------
  enum class Mode { TRACK, CORNER_RIGHT, FRONT_AVOID };
  static Mode mode = Mode::TRACK;
  static rclcpp::Time mode_enter_time;

  const auto now = this->get_clock()->now();
  auto time_in_mode = [&](){ return (now - mode_enter_time).seconds(); };
  auto enter = [&](Mode m){ if (mode != m) { mode = m; mode_enter_time = now; }};

  // Opening confirmation (temporal hysteresis)
  static int opening_count = 0;
  const bool opening_now = (r_n > OPENING_MIN) || (r_f > OPENING_MIN);
  if (opening_now) opening_count = std::min(opening_count + 1, 100);
  else if ((r_n < PRESENT_MAX) && (r_f < PRESENT_MAX)) opening_count = 0;

  // ---------------- Front safety (highest priority) ----------------
  if (mode != Mode::FRONT_AVOID && d_front < FRONT_STOP) enter(Mode::FRONT_AVOID);
  if (mode == Mode::FRONT_AVOID) {
    if (d_front > FRONT_CLEAR) {
      enter(Mode::TRACK);
      update_cmd_vel(std::min(V_STRAIGHT, 0.7 * V_STRAIGHT), 0.0);
      return;
    }
    update_cmd_vel(0.0, +ANGULAR_VELOCITY);  // left turn, right-wall convention
    return;
  }

  // ---------------- CORNER_RIGHT mode ----------------
  if (mode == Mode::CORNER_RIGHT) {
    // Exit if: wall present again AND roughly parallel, or timeout
    const bool wall_present = (r_n < PRESENT_MAX) && (r_f < PRESENT_MAX);
    const bool parallelish  = (std::abs(theta_err) < REACQ_THETA);
    if ((wall_present && parallelish) || time_in_mode() > CORNER_MAX_S) {
      enter(Mode::TRACK);
      update_cmd_vel(V_STRAIGHT, 0.0);
      return;
    }
    // Keep arcing right to round the corner
    update_cmd_vel(CORNER_V, -CORNER_W);
    return;
  }

  // ---------------- TRACK mode (align → straight/nudge) ----------------
  // Trigger corner rounding when opening confirmed, with a small preference
  // for "forward-right jumps first" (r_n dominating) to avoid false positives.
  if (mode == Mode::TRACK && opening_count >= OPEN_CONFIRM && r_n >= r_f) {
    enter(Mode::CORNER_RIGHT);
    update_cmd_vel(CORNER_V, -CORNER_W);
    return;
  }

  // 1) Align to be parallel first
  if (std::abs(theta_err) > THETA_TOL) {
    double w = std::clamp(-K_THETA * theta_err, -W_ALIGN_MAX, W_ALIGN_MAX);
    update_cmd_vel(0.0, w);
    return;
  }

  // 2) If parallel, distance band with hysteresis ⇒ straight vs nudge
  const double in_min = D_DES - BAND, in_max = D_DES + BAND;
  const double ex_min = D_DES - (BAND - EXIT_MARGIN);
  const double ex_max = D_DES + (BAND - EXIT_MARGIN);

  enum class BandMode { IN_BAND, TOO_CLOSE, TOO_FAR };
  static BandMode band_mode = BandMode::IN_BAND;

  if (band_mode == BandMode::IN_BAND) {
    if (d_perp < in_min)      band_mode = BandMode::TOO_CLOSE;
    else if (d_perp > in_max) band_mode = BandMode::TOO_FAR;
  } else if (band_mode == BandMode::TOO_CLOSE) {
    if (d_perp >= ex_min) band_mode = BandMode::IN_BAND;
  } else { // TOO_FAR
    if (d_perp <= ex_max) band_mode = BandMode::IN_BAND;
  }

  if (band_mode == BandMode::IN_BAND) {
    update_cmd_vel(V_STRAIGHT, 0.0);
  } else if (band_mode == BandMode::TOO_CLOSE) {
    update_cmd_vel(V_NUDGE, +W_NUDGE);   // nudge LEFT
  } else { // TOO_FAR
    update_cmd_vel(V_NUDGE, -W_NUDGE);   // nudge RIGHT
  }
}




/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();
  return 0;
}
