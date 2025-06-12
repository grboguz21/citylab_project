#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class ServiceTester : public rclcpp::Node {
public:
  ServiceTester() : Node("test_service") {
    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ServiceTester::laser_callback, this, _1));
    RCLCPP_INFO(this->get_logger(),
                "TestServiceNode started and waiting for laser data...");
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service not available.");
      return;
    }

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg;

    auto future = client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                   future_response) {
          auto response = future_response.get();
          RCLCPP_INFO(this->get_logger(), "Service responded: %s",
                      response->direction.c_str());
        });
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceTester>());
  rclcpp::shutdown();
  return 0;
}
