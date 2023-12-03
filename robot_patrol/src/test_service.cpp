#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/init_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/detail/get_direction__struct.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

constexpr char kNodeName[]{"test_service"};
constexpr char kServiceName[]{"direction_service"};
constexpr char kTopicName[]{"/scan"};

class TestService : public rclcpp::Node {
public:
  TestService()
      : Node{kNodeName},
        subscription_{this->create_subscription<sensor_msgs::msg::LaserScan>(
            kTopicName, 10,
            std::bind(&TestService::subscription_cb, this,
                      std::placeholders::_1))},
        client_{
            this->create_client<robot_patrol::srv::GetDirection>(kServiceName)},
        timer_{this->create_wall_timer(
            kWaitTime, std::bind(&TestService::timer_cb, this))} {}
  bool is_service_done() const { return this->service_done_; }

private:
  constexpr static std::chrono::nanoseconds kWaitTime{1s};

  void timer_cb();
  void subscription_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan msg_{};
  bool sensor_data_available_{false};
  bool service_done_{false};
};

void TestService::timer_cb() {
  if (!sensor_data_available_) {
    RCLCPP_INFO(this->get_logger(), "Sensor data not available yet.");
    return;
  }

  while (!client_->wait_for_service(kWaitTime)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service Unavailable.");
    return;
  }

  auto request{std::make_shared<robot_patrol::srv::GetDirection::Request>()};
  request->laser_data = msg_;

  auto result_future{client_->async_send_request(
      request,
      std::bind(&TestService::response_callback, this, std::placeholders::_1))};
}

void TestService::response_callback(
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
  auto status{future.wait_for(kWaitTime)};
  if (status == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Result: %s",
                future.get()->direction.c_str());
    service_done_ = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

void TestService::subscription_cb(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  msg_ = *msg;
  sensor_data_available_ = true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node{std::make_shared<TestService>()};
  while (!node->is_service_done()) {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
}