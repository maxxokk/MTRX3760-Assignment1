#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros_gz_interfaces/msg/visual.hpp>
#include <ros_gz_interfaces/msg/geometry.hpp>

using std::placeholders::_1;

class TrailMarkerNode : public rclcpp::Node {
public:
  TrailMarkerNode()
  : rclcpp::Node("turtlebot3_trail_marker"), have_pose_(false), counter_(0)
  {
    // Subscribe to the model's ground-truth pose (bridged from Gazebo)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/model/turtlebot3_burger_cam/pose", 10,
        std::bind(&TrailMarkerHybrid::poseCallback, this, std::placeholders::_1));

    // Publish Visuals into the Gazebo world (bridged /world/default/visual)
    vis_pub_ = this->create_publisher<ros_gz_interfaces::msg::Visual>(
        "/world/default/visual", 10);

    // Drop a sphere every 0.2 s
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TrailMarkerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "TrailMarkerNode started: model='burger_cam', world='default', dt=0.2s");
  }

private:
  void poseCallback(const geometry_msgs::msg::Pose &pose) {
    last_pose_ = pose;
    have_pose_ = true;
  }

  void timerCallback() {
    if (!have_pose_) return;

    ros_gz_interfaces::msg::Visual vis;

    // Unique name & id (id is arbitrary but should be unique-ish)
    vis.name = "trail_" + std::to_string(counter_);
    vis.id = static_cast<int32_t>(counter_);

    // Attach to world (no parent link required)
    vis.parent_name = "world";

    // Pose: drop sphere at the robot's current pose
    vis.pose = last_pose_;

    // Geometry: small sphere
    vis.geometry.type = ros_gz_interfaces::msg::Geometry::SPHERE;
    vis.geometry.sphere.radius = 0.03;  // 3 cm

    // (Optional) Make sure it’s visible (defaults are fine)
    vis.visible = true;
    vis.cast_shadows = false;
    vis.transparency = 0.0f;

    vis_pub_->publish(vis);
    ++counter_;
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Visual>::SharedPtr vis_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose last_pose_;
  bool have_pose_;
  uint32_t counter_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrailMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
