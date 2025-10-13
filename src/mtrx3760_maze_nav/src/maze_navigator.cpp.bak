#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mtrx3760_maze_nav/wall_follower.hpp"
#include "mtrx3760_maze_nav/config.hpp"

using namespace std::chrono_literals;

class MazeNavigator : public rclcpp::Node {
public:
  MazeNavigator() : rclcpp::Node("maze_navigator") {
    // Declare tunable parameters with sane defaults
    this->declare_parameter<double>("desired_wall",  cfg::DESIRED_WALL);
    this->declare_parameter<double>("front_stop",    cfg::FRONT_STOP);
    this->declare_parameter<double>("lost_wall",     cfg::LOST_WALL);
    this->declare_parameter<double>("lin_speed",     cfg::LIN_SPEED);
    this->declare_parameter<double>("max_ang_speed", cfg::MAX_ANG_SPEED);
    this->declare_parameter<double>("kp",            cfg::KP);

    vels_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    vels_pub_ts_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_stamped", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        wf_.update_from_scan(*msg);
        last_scan_time_ = this->now();
      });

    timer_ = this->create_wall_timer(100ms, [this]() {
      // Pull latest params (cheap)
      WallFollower::Params p;
      this->get_parameter("desired_wall",  p.desired_wall);
      this->get_parameter("front_stop",    p.front_stop);
      this->get_parameter("lost_wall",     p.lost_wall);
      this->get_parameter("lin_speed",     p.lin_speed);
      this->get_parameter("max_ang_speed", p.max_ang_speed);
      this->get_parameter("kp",            p.kp);
      wf_.set_params(p);

      // Safety stop if scan is stale
      geometry_msgs::msg::Twist cmd;
      if (!last_scan_time_.nanoseconds() ||
          (this->now() - last_scan_time_) > rclcpp::Duration(500ms)) {
        cmd = geometry_msgs::msg::Twist{};
      } else {
        cmd = wf_.compute_cmd();
      }

      // Publish TwistStamped for ros_gz_bridge
      geometry_msgs::msg::TwistStamped ts;
      ts.header.stamp = this->now();
      ts.header.frame_id = "base_footprint";
      ts.twist = cmd;
      // publish plain Twist for Gazebo bridge
    vels_pub_->publish(cmd);
// publish stamped for debugging/plots
    vels_pub_ts_->publish(ts);

      // Debug (1 Hz)
      if (++tick_ % 10 == 0) {
        auto s = wf_.state();
        const char* name = (s == WallFollower::State::FOLLOW_WALL ? "FOLLOW" :
                            s == WallFollower::State::TURN_CORNER ? "TURN" : "ADJUST");
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "state=%s vx=%.2f wz=%.2f params{dw=%.2f fs=%.2f lw=%.2f lin=%.2f maxw=%.2f kp=%.2f}",
          name, cmd.linear.x, cmd.angular.z,
          p.desired_wall, p.front_stop, p.lost_wall, p.lin_speed, p.max_ang_speed, p.kp);
      }
    });

    RCLCPP_INFO(this->get_logger(), "MazeNavigator up (params enabled).");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vels_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vels_pub_ts_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  WallFollower wf_;
  rclcpp::Time last_scan_time_{};
  size_t tick_{0};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeNavigator>());
  rclcpp::shutdown();
  return 0;
}
