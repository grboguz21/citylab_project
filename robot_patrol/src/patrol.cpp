#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>
#include <vector>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Patrol::controlLoop, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<float> laser_ranges_;

  float direction_ = 0.0;
  bool front_obstacle_ = false;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_ranges_ = msg->ranges;

    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;
    int total = laser_ranges_.size();

    int start_index =
        std::max(0, static_cast<int>((-M_PI_2 - angle_min) / angle_increment));
    int end_index = std::min(
        total - 1, static_cast<int>((M_PI_2 - angle_min) / angle_increment));

    front_obstacle_ = false;
    for (int i = start_index; i <= end_index; ++i) {
      if (std::isfinite(laser_ranges_[i]) && laser_ranges_[i] < 0.35) {
        front_obstacle_ = true;
        break;
      }
    }

    if (front_obstacle_) {
      float max_left_distance = 0.0;
      float max_right_distance = 0.0;
      int max_left_index = start_index;
      int max_right_index = start_index;

      float min_distance = std::numeric_limits<float>::infinity();
      int min_index = (start_index + end_index) / 2;

      for (int i = start_index; i <= end_index; ++i) {
        float d = laser_ranges_[i];
        if (!std::isfinite(d))
          continue;

        float angle = angle_min + i * angle_increment;

        if (d < min_distance) {
          min_distance = d;
          min_index = i;
        }

        if (angle < 0 && d > max_left_distance) {
          max_left_distance = d;
          max_left_index = i;
        }

        if (angle > 0 && d > max_right_distance) {
          max_right_distance = d;
          max_right_index = i;
        }
      }

      float min_angle = angle_min + min_index * angle_increment;

      if (min_angle > 0) {
        direction_ = angle_min + max_left_index * angle_increment;
      } else {
        direction_ = angle_min + max_right_index * angle_increment;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Obstacle Detected! 🔴 Min: %.2f m at %.2f rad | Turning to "
                  "safest %.2f rad",
                  min_distance, min_angle, direction_);
    } else {
      direction_ = 0.0;
    }
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;

    if (front_obstacle_) {
      cmd.angular.z = direction_ / 1.5;
    } else {
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
