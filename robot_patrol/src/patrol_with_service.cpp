#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PatrolWithService : public rclcpp::Node {
public:
  PatrolWithService() : Node("patrol_with_service") {
    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PatrolWithService::laser_callback, this, _1));

    direction_client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");

    RCLCPP_INFO(this->get_logger(), "PatrolWithService node started.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr direction_client_;

  sensor_msgs::msg::LaserScan latest_scan_;
  bool scan_received_ = false;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!direction_client_->wait_for_service(500ms)) {
      RCLCPP_WARN(this->get_logger(), "Direction service not available.");
      return;
    }

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg;

    direction_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                   future) {
          std::string dir = future.get()->direction;
          RCLCPP_INFO(this->get_logger(), "Direction: %s", dir.c_str());

          geometry_msgs::msg::Twist cmd_vel;
          cmd_vel.linear.x = 0.1;

          if (dir == "forward") {
            cmd_vel.angular.z = 0.0;
          } else if (dir == "left") {
            cmd_vel.angular.z = 0.5;
          } else if (dir == "right") {
            cmd_vel.angular.z = -0.5;
          } else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
          }

          vel_pub_->publish(cmd_vel);
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolWithService>());
  rclcpp::shutdown();
  return 0;
}
