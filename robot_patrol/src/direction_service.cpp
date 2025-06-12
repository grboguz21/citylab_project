#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service") {
    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::handle_service, this, _1, _2));

    pub_right_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("laser_right", 10);
    pub_front_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("laser_front", 10);
    pub_left_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("laser_left", 10);

    RCLCPP_INFO(this->get_logger(), "DirectionService is ready.");
  }

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_front_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_left_;

  void handle_service(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {

    const auto &scan = request->laser_data;
    int total_ranges = scan.ranges.size();

    if (total_ranges < 10) {
      response->direction = "forward";
      RCLCPP_WARN(this->get_logger(),
                  "Laser data too small! Defaulting to forward.");
      return;
    }

    auto make_empty_scan = [&scan]() {
      sensor_msgs::msg::LaserScan new_scan;
      new_scan.header = scan.header;
      new_scan.angle_min = scan.angle_min;
      new_scan.angle_max = scan.angle_max;
      new_scan.angle_increment = scan.angle_increment;
      new_scan.time_increment = scan.time_increment;
      new_scan.scan_time = scan.scan_time;
      new_scan.range_min = scan.range_min;
      new_scan.range_max = scan.range_max;
      new_scan.ranges.resize(scan.ranges.size(), 0.0);
      return new_scan;
    };

    auto right_scan = make_empty_scan();
    auto front_scan = make_empty_scan();
    auto left_scan = make_empty_scan();

    float total_dist_right = 0.0;
    float total_dist_front = 0.0;
    float total_dist_left = 0.0;

    bool front_clear = true;

    for (int i = 0; i < total_ranges; ++i) {
      float angle = scan.angle_min + i * scan.angle_increment;
      float r = scan.ranges[i];
      if (std::isnan(r) || std::isinf(r))
        continue;

      if (angle >= -1.57 && angle < -0.52) {
        // Right
        total_dist_right += r;
        right_scan.ranges[i] = r;
      } else if (angle >= -0.52 && angle <= 0.52) {
        // Front
        total_dist_front += r;
        front_scan.ranges[i] = r;
        if (r < 0.35)
          front_clear = false;
      } else if (angle > 0.52 && angle <= 1.57) {
        // Left
        total_dist_left += r;
        left_scan.ranges[i] = r;
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Total ranges | Right: %.2f | Front: %.2f | Left: %.2f",
                total_dist_right, total_dist_front, total_dist_left);

    // Karar mantığı
    if (front_clear) {
      response->direction = "forward";
    } else {
      response->direction =
          (total_dist_left > total_dist_right) ? "left" : "right";
    }

    RCLCPP_INFO(this->get_logger(), "Decision: %s",
                response->direction.c_str());

    pub_right_->publish(right_scan);
    pub_front_->publish(front_scan);
    pub_left_->publish(left_scan);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
