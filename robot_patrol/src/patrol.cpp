#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    // laser scan sub
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

    // cmd_vel pub
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // timer 10hz
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Patrol::controlLoop, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> laser_ranges_;
  float direction_;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_ranges_ = msg->ranges;

    int total = laser_ranges_.size();
    int start_index = total / 4;
    int end_index = 3 * total / 4;

    float max_distance = 0.0;
    int best_index = start_index;

    for (int i = start_index; i < end_index; ++i) {
      if (std::isfinite(laser_ranges_[i]) && laser_ranges_[i] > max_distance) {
        max_distance = laser_ranges_[i];
        best_index = i;
      }
    }

    float angle = msg->angle_min + best_index * msg->angle_increment;
    direction_ = angle;

    RCLCPP_INFO(this->get_logger(), "Safest direction: %.2f rad (%.2f m)",
                direction_, max_distance);
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
