#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <limits>
#include <cmath>

class WallFollower : public rclcpp::Node {
public:
  WallFollower() : Node("tb3_right_wall_follower")
  {
    declare_parameter<double>("desired", 0.35);
    declare_parameter<double>("front_clear", 0.45);
    declare_parameter<double>("v_forward", 0.18);
    declare_parameter<double>("kp", 1.6);
    declare_parameter<double>("kd", 0.25);
    declare_parameter<double>("max_w", 1.8);
    declare_parameter<double>("right_start_deg", -90.0);
    declare_parameter<double>("right_end_deg",   -20.0);
    declare_parameter<double>("front_start_deg",  -20.0);
    declare_parameter<double>("front_end_deg",     20.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&WallFollower::onScan, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Right-wall follower ready.");
  }

private:
  static double clamp(double x, double lo, double hi){
    return std::max(lo, std::min(x, hi));
  }
  struct Sector { double amin_deg, amax_deg; };

  double min_in_sector(const sensor_msgs::msg::LaserScan & s, Sector sec) {
    const double amin = s.angle_min;
    const double ainc = s.angle_increment;
    const int n = static_cast<int>(s.ranges.size());
    auto rad = [](double d){ return d * M_PI / 180.0; };
    const double a0 = rad(sec.amin_deg), a1 = rad(sec.amax_deg);
    auto idx = [&](double a)->int { return static_cast<int>( std::round((a - amin) / ainc) ); };
    int i0 = std::max(0, std::min(n-1, idx(a0)));
    int i1 = std::max(0, std::min(n-1, idx(a1)));
    if (i0 > i1) std::swap(i0, i1);
    double m = std::numeric_limits<double>::infinity();
    for (int i=i0; i<=i1; ++i){
      const float r = s.ranges[i];
      if (std::isfinite(r) && r > s.range_min) m = std::min(m, static_cast<double>(r));
    }
    return m;
  }

  void onScan(const sensor_msgs::msg::LaserScan & s){
    const double desired     = get_parameter("desired").as_double();
    const double front_clear = get_parameter("front_clear").as_double();
    const double v_forward   = get_parameter("v_forward").as_double();
    const double kp          = get_parameter("kp").as_double();
    const double kd          = get_parameter("kd").as_double();
    const double max_w       = get_parameter("max_w").as_double();

    Sector right{ get_parameter("right_start_deg").as_double(),
                  get_parameter("right_end_deg").as_double() };
    Sector front{ get_parameter("front_start_deg").as_double(),
                  get_parameter("front_end_deg").as_double() };

    const double d_right = min_in_sector(s, right);
    const double d_front = min_in_sector(s, front);

    geometry_msgs::msg::Twist cmd;

    if (d_front < front_clear) {
      cmd.linear.x  = 0.05;
      cmd.angular.z = 0.9;   // left to avoid front obstacle
      last_err_ = 0.0;
    } else {
      if (!std::isfinite(d_right) || d_right > 3.0) {
        cmd.linear.x  = 0.10;
        cmd.angular.z = -0.4; // search for right wall
      } else {
        const double err  = desired - d_right;
        const double dt   = (s.scan_time > 0.0) ? s.scan_time
                          : (s.time_increment > 0.0 ? s.time_increment * s.ranges.size() : 0.1);
        const double derr = (err - last_err_) / std::max(1e-3, dt);
        const double w    = kp * err + kd * derr;
        cmd.linear.x  = v_forward;
        cmd.angular.z = clamp(-w, -max_w, max_w); // minus = right-wall convention
        last_err_ = err;
      }
    }
    pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  double last_err_{0.0};
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}
