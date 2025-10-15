// src/cmd_vel_mux.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

class CmdVelMux : public rclcpp::Node {
public:
  CmdVelMux() : Node("cmd_vel_mux") {
    declare_parameter<std::string>("output_topic", "cmd_vel");
    declare_parameter<std::string>("mode_topic", "control_mode");
    declare_parameter<std::string>("follower_topic", "cmd_vel_follower");
    declare_parameter<std::string>("nav_topic", "cmd_vel_nav");
    output_topic_   = get_parameter("output_topic").as_string();
    mode_topic_     = get_parameter("mode_topic").as_string();
    follower_topic_ = get_parameter("follower_topic").as_string();
    nav_topic_      = get_parameter("nav_topic").as_string();

    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

    sub_mode_ = create_subscription<std_msgs::msg::String>(
      mode_topic_, 10,
      [&](std_msgs::msg::String::SharedPtr m){
        if (m->data == "FOLLOWER" || m->data == "NAV2") {
          mode_ = m->data;
          RCLCPP_INFO(get_logger(), "Mode -> %s", mode_.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Unknown mode '%s' (expected FOLLOWER or NAV2)", m->data.c_str());
        }
      });

    sub_follower_ = create_subscription<geometry_msgs::msg::Twist>(
      follower_topic_, rclcpp::QoS(10),
      [&](geometry_msgs::msg::Twist::SharedPtr msg){
        if (mode_ == "FOLLOWER") pub_->publish(*msg);
      });

    sub_nav_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_topic_, rclcpp::QoS(10),
      [&](geometry_msgs::msg::Twist::SharedPtr msg){
        if (mode_ == "NAV2") pub_->publish(*msg);
      });

    // default to FOLLOWER
    std_msgs::msg::String init; init.data = mode_;
    mode_latch_pub_ = create_publisher<std_msgs::msg::String>(mode_topic_, 1);
    mode_latch_pub_->publish(init);
  }
private:
  std::string output_topic_, mode_topic_, follower_topic_, nav_topic_;
  std::string mode_ = "FOLLOWER";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_follower_, sub_nav_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_latch_pub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMux>());
  rclcpp::shutdown();
  return 0;
}
