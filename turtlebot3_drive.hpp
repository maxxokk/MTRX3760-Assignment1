#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>

// in turtlebot3_drive.hpp
enum { CENTER = 0, LEFT = 1, RIGHT = 2 };

class Turtlebot3Drive : public rclcpp::Node {
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void update_callback();
  void update_cmd_vel(double linear, double angular);

  // pubs/subs
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // state
  double scan_data_[3];   // was [3]
  double robot_pose_, prev_robot_pose_;
  bool end_seen_;
  bool end_announced_;
  // turtlebot3_drive.hpp (private:)
  double side_r_255_{0.0};  // right side, 15° behind (255°)
  double side_r_285_{0.0};  // right side, 15° ahead  (285°)


  
  // You likely already have LINEAR_VELOCITY / ANGULAR_VELOCITY in your original header.
};
