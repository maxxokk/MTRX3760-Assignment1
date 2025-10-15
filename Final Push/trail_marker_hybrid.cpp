#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Gazebo Transport + Msgs
#include <gz/transport/Node.hh>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/material.pb.h>

// Gazebo Math
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

using namespace std::chrono_literals;

class TrailMarkerHybrid : public rclcpp::Node
{
public:
  TrailMarkerHybrid()
  : rclcpp::Node("trail_marker_hybrid"),
    have_pose_(false),
    counter_(0),
    world_topic_("/world/default/visual")
  {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/model/burger_cam/pose", 10,
      std::bind(&TrailMarkerHybrid::poseCallback, this, std::placeholders::_1));

    pub_ = node_.Advertise<gz::msgs::Visual>(world_topic_);
    timer_ = this->create_wall_timer(200ms,
                std::bind(&TrailMarkerHybrid::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "TrailMarkerHybrid running — publishing to %s every 0.2 s",
      world_topic_.c_str());
  }

private:
  void poseCallback(const geometry_msgs::msg::Pose &msg)
  {
    last_pose_ = msg;
    have_pose_ = true;
  }

  void timerCallback()
  {
    if (!have_pose_)
      return;

    gz::msgs::Visual vis;
    vis.set_name("trail_" + std::to_string(counter_));
    vis.set_id(counter_);
    vis.set_parent_name("world");
    vis.set_visible(true);
    vis.set_cast_shadows(false);
    vis.set_transparency(0.0);

    // ---- Build gz::msgs::Pose manually ----
    gz::msgs::Pose *pose_msg = vis.mutable_pose();
    pose_msg->mutable_position()->set_x(last_pose_.position.x);
    pose_msg->mutable_position()->set_y(last_pose_.position.y);
    pose_msg->mutable_position()->set_z(last_pose_.position.z);
    pose_msg->mutable_orientation()->set_x(last_pose_.orientation.x);
    pose_msg->mutable_orientation()->set_y(last_pose_.orientation.y);
    pose_msg->mutable_orientation()->set_z(last_pose_.orientation.z);
    pose_msg->mutable_orientation()->set_w(last_pose_.orientation.w);
    // ---------------------------------------

    // Sphere geometry (3 cm radius)
    vis.mutable_geometry()->set_type(gz::msgs::Geometry::SPHERE);
    vis.mutable_geometry()->mutable_sphere()->set_radius(0.03);

    // Green colour
    auto *mat = vis.mutable_material();
    mat->mutable_ambient()->set_r(0.0f);
    mat->mutable_ambient()->set_g(1.0f);
    mat->mutable_ambient()->set_b(0.0f);
    mat->mutable_ambient()->set_a(1.0f);
    mat->mutable_diffuse()->set_r(0.0f);
    mat->mutable_diffuse()->set_g(1.0f);
    mat->mutable_diffuse()->set_b(0.0f);
    mat->mutable_diffuse()->set_a(1.0f);

    if (!pub_.Publish(vis))
      RCLCPP_WARN(this->get_logger(),
                  "Failed to publish visual to %s", world_topic_.c_str());

    counter_++;
  }

  // --- Members ---
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose last_pose_;
  bool have_pose_;
  int counter_;
  std::string world_topic_;
  gz::transport::Node node_;
  gz::transport::v13::Node::Publisher pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrailMarkerHybrid>());
  rclcpp::shutdown();
  return 0;
}
