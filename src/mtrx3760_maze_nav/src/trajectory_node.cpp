#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class TrajectoryNode : public rclcpp::Node {
public:
  TrajectoryNode() : rclcpp::Node("trajectory_node") {
    this->declare_parameter<int>("max_points", 2000);
    this->declare_parameter<double>("publish_rate_hz", 5.0);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&TrajectoryNode::on_odom, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory_path", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/trajectory_marker", 10);

    double hz = this->get_parameter("publish_rate_hz").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / std::max(1.0, hz))),
      std::bind(&TrajectoryNode::publish, this));
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    int max_pts = this->get_parameter("max_points").as_int();
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose = msg->pose.pose;

    path_.header = msg->header;
    deque_.push_back(ps);
    if ((int)deque_.size() > max_pts) deque_.pop_front();
  }

  void publish() {
    if (deque_.empty()) return;

    // Build path message
    path_.poses.assign(deque_.begin(), deque_.end());
    path_pub_->publish(path_);

    // Build marker message
    visualization_msgs::msg::Marker m;
    m.header = path_.header;
    m.ns = "trajectory";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02; // line width (m)
    // Color (RViz ignores if all zeros) – simple white with some alpha
    m.color.a = 0.9f; m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f;

    m.points.reserve(deque_.size());
    for (const auto &ps : deque_) {
      geometry_msgs::msg::Point p;
      p.x = ps.pose.position.x;
      p.y = ps.pose.position.y;
      p.z = 0.01; // tiny lift so it draws above ground
      m.points.push_back(p);
    }
    marker_pub_->publish(m);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<geometry_msgs::msg::PoseStamped> deque_;
  nav_msgs::msg::Path path_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
