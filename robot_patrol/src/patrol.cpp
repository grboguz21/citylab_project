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

    float front_angle_range = M_PI / 8; // ≈30°
    int index_front_min =
        std::max(0, static_cast<int>((-front_angle_range - angle_min) /
                                     angle_increment));
    int index_front_max =
        std::min(total - 1, static_cast<int>((front_angle_range - angle_min) /
                                             angle_increment));
    int center_index = static_cast<int>((0.0 - angle_min) / angle_increment);

    float min_distance = std::numeric_limits<float>::infinity();
    int min_index = -1;
    front_obstacle_ = false;

    for (int i = index_front_min; i <= index_front_max; ++i) {
      float distance = laser_ranges_[i];
      if (std::isfinite(distance)) {
        if (distance < min_distance) {
          min_distance = distance;
          min_index = i;
        }
        if (distance < 0.35) {
          front_obstacle_ = true;
        }
      }
    }

    float direction_angle = 0.0;
    if (front_obstacle_) {
      int index_min_90 = std::max(
          0, static_cast<int>((-M_PI_2 - angle_min) / angle_increment));
      int index_plus_90 = std::min(
          total - 1, static_cast<int>((M_PI_2 - angle_min) / angle_increment));

      float max_distance = 0.0;
      int best_index = -1;

      for (int i = index_min_90; i <= index_plus_90; ++i) {
        float distance = laser_ranges_[i];
        if (std::isfinite(distance) && distance > max_distance) {
          max_distance = distance;
          best_index = i;
        }
      }

      if (best_index != -1) {
        direction_angle = angle_min + best_index * angle_increment;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Obstacle detected! Safest direction: %.2f m | %.2f rad",
                  max_distance, direction_angle);
    }

    direction_ = direction_angle;

    float min_angle =
        (min_index != -1) ? angle_min + min_index * angle_increment : 0.0;
    RCLCPP_INFO(this->get_logger(), "Closest Obstacle: %.2f m | %.2f rad",
                min_distance, min_angle);
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd;

    if (front_obstacle_) {
      cmd.linear.x = 0.1;
      cmd.angular.z = direction_ / 2.0;
    } else {
      cmd.linear.x = 0.1;
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