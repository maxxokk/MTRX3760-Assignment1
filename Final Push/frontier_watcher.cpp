// src/frontier_watcher.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>  // std::max
#include <cmath>      // std::abs

class FrontierWatcher : public rclcpp::Node {
public:
  FrontierWatcher() : rclcpp::Node("frontier_watcher")
  {
    // ---- parameters ----
    this->declare_parameter<std::string>("map_topic", "map");
    this->declare_parameter<std::string>("mode_topic", "control_mode");
    this->declare_parameter<int>("min_frontiers", 200);
    this->declare_parameter<double>("frontier_ratio_thresh", 0.003); // frontiers / free
    this->declare_parameter<int>("min_free_cells", 1500);
    this->declare_parameter<double>("hold_sec", 5.0);
    this->declare_parameter<double>("warmup_sec", 15.0);
    this->declare_parameter<bool>("use_ratio", true);
    this->declare_parameter<bool>("auto_switch", true);

    map_topic_  = this->get_parameter("map_topic").as_string();
    mode_topic_ = this->get_parameter("mode_topic").as_string();
    min_frontiers_  = this->get_parameter("min_frontiers").as_int();
    frontier_ratio_thresh_ = this->get_parameter("frontier_ratio_thresh").as_double();
    min_free_cells_ = this->get_parameter("min_free_cells").as_int();
    hold_sec_   = this->get_parameter("hold_sec").as_double();
    warmup_sec_ = this->get_parameter("warmup_sec").as_double();
    use_ratio_  = this->get_parameter("use_ratio").as_bool();
    auto_switch_= this->get_parameter("auto_switch").as_bool();

    // ---- pubs/subs ----
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&FrontierWatcher::mapCb, this, std::placeholders::_1));

    pub_mode_  = this->create_publisher<std_msgs::msg::String>(mode_topic_, 1);
    pub_count_ = this->create_publisher<std_msgs::msg::Int32>("/frontier_count", 10);
    pub_ratio_ = this->create_publisher<std_msgs::msg::Float32>("/frontier_ratio", 10);

    start_time_ = this->now();
    last_above_time_ = start_time_;
  }

private:
  void mapCb(nav_msgs::msg::OccupancyGrid::SharedPtr m)
  {
    const int w = static_cast<int>(m->info.width);
    const int h = static_cast<int>(m->info.height);
    const auto & data = m->data;
    if (w < 3 || h < 3) return;

    auto idx = [w](int x, int y){ return y*w + x; };
    auto is_unknown = [&](int i){ return data[i] == -1; };
    auto is_free    = [&](int i){ return data[i] == 0;  };

    int free_cells = 0;
    int frontiers  = 0;

    // Count free cells + simple 4-neighbor frontier test
    for (int y = 1; y < h - 1; ++y) {
      for (int x = 1; x < w - 1; ++x) {
        int i = idx(x, y);
        if (is_free(i)) ++free_cells;

        if (!is_unknown(i)) continue;
        if (is_free(idx(x+1, y)) || is_free(idx(x-1, y)) ||
            is_free(idx(x, y+1)) || is_free(idx(x, y-1))) {
          ++frontiers;
        }
      }
    }

    // Publish debug
    std_msgs::msg::Int32 cnt; cnt.data = frontiers;
    pub_count_->publish(cnt);

    if (free_cells > 0) {
      std_msgs::msg::Float32 fr;
      fr.data = static_cast<float>(frontiers) / static_cast<float>(free_cells);
      pub_ratio_->publish(fr);
    }

    const rclcpp::Time tnow = this->now();
    const double since_start = (tnow - start_time_).seconds();

    // Warm-up period: don't switch
    if (since_start < warmup_sec_) {
      last_above_time_ = tnow;     // keep resetting so hold_sec never elapses
      seen_high_ = false;
      decided_nav2_ = false;
      return;
    }

    // Decide "high" based on ratio or absolute
    bool is_high_now = false;
    if (use_ratio_) {
      const double ratio = (free_cells > 0)
        ? static_cast<double>(frontiers) / std::max(1, free_cells)
        : 1.0;
      is_high_now = (ratio >= frontier_ratio_thresh_);
    } else {
      is_high_now = (frontiers >= min_frontiers_);
    }

    // Require sufficient explored area before considering a switch
    if (free_cells < min_free_cells_) {
      last_above_time_ = tnow;
      return;
    }

    // Track if we've seen a "high" frontier state at least once
    if (is_high_now) {
      last_above_time_ = tnow;
      seen_high_ = true;
      decided_nav2_ = false;
      return;
    }

    // Frontiers are "low" now
    const double low_for = (tnow - last_above_time_).seconds();
    if (auto_switch_ && !decided_nav2_ && seen_high_ && low_for >= hold_sec_) {
      decided_nav2_ = true;
      std_msgs::msg::String mode; mode.data = "NAV2";
      RCLCPP_WARN(this->get_logger(),
                  "Frontiers low for %.1fs (frontiers=%d, free=%d) → switching to NAV2",
                  low_for, frontiers, free_cells);
      pub_mode_->publish(mode);
    }
  }

  // ---- params ----
  std::string map_topic_, mode_topic_;
  int    min_frontiers_{200};
  int    min_free_cells_{1500};
  double frontier_ratio_thresh_{0.003};
  double hold_sec_{5.0};
  double warmup_sec_{15.0};
  bool   use_ratio_{true};
  bool   auto_switch_{true};

  // ---- state ----
  rclcpp::Time start_time_;
  rclcpp::Time last_above_time_;
  bool seen_high_{false};
  bool decided_nav2_{false};

  // ---- ROS ----
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_mode_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   pub_count_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_ratio_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierWatcher>());
  rclcpp::shutdown();
  return 0;
}
