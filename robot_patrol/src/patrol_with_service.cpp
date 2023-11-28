#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <robot_patrol/srv/get_direction.hpp>

using namespace std::chrono_literals;

class PatrolNode : public rclcpp::Node {
public:
  PatrolNode()
      : Node{kNodeName}, callback_group_{this->create_callback_group(
                             rclcpp::CallbackGroupType::Reentrant)},
        options_{[this]() {
          rclcpp::SubscriptionOptions opt;
          opt.callback_group = callback_group_;
          return opt;
        }()},
        scan_subscriber_{this->create_subscription<sensor_msgs::msg::LaserScan>(
            kLaserTopicName, 10,
            std::bind(&PatrolNode::scan_cb, this, std::placeholders::_1),
            options_)},
        client_{
            this->create_client<robot_patrol::srv::GetDirection>(kServiceName)},
        twist_publisher_{this->create_publisher<geometry_msgs::msg::Twist>(
            kCmdTopicName, 1)},
        control_loop_timer_{this->create_wall_timer(
            200ms, std::bind(&PatrolNode::control_loop_cb, this),
            callback_group_)} {}

private:
  // interface names
  constexpr static char kNodeName[]{"patrol_with_service"};
  constexpr static char kServiceName[]{"direction_service"};
  constexpr static char kLaserTopicName[]{"/scan"};
  constexpr static char kCmdTopicName[]{"cmd_vel"};

  // math constants
  static constexpr double kPi{3.1416};

  // settings
  static constexpr double kAngleMin{-kPi / 2};
  static constexpr double kAngleMax{kPi / 2};
  std::chrono::nanoseconds kServiceTimeout{0ms};
  std::chrono::nanoseconds kFutureTimeout{100ms};

  // helper functions
  bool valid(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // callbacks
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_loop_cb();

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::SubscriptionOptions options_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscriber_{};
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{};
  rclcpp::TimerBase::SharedPtr control_loop_timer_{};
  std::vector<std::optional<double>> sliding_min_;
  sensor_msgs::msg::LaserScan last_laser_{};
};

bool PatrolNode::valid(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (msg->angle_increment <= 0) {
    RCLCPP_WARN(this->get_logger(),
                "Found unexpected value of angle_increment<=0 in the laser "
                "scan message.");
    return false;
  }
  if (msg->angle_min > kAngleMin) {
    RCLCPP_WARN(this->get_logger(),
                "The angle of -pi/2 is outside of the laser scan range.");
    return false;
  }
  if (msg->angle_min + msg->angle_increment * msg->ranges.size() < kAngleMax) {
    RCLCPP_WARN(this->get_logger(),
                "The angle of pi/2 is outside of the laser scan range.");
    return false;
  }
  return true;
}

void PatrolNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Started scan_cb() callback.");

  if (!valid(msg)) {
    RCLCPP_WARN(this->get_logger(), "Message not valid, ignoring.");
    return;
  }

  last_laser_ = *msg;
}

void PatrolNode::control_loop_cb() {
  RCLCPP_DEBUG(this->get_logger(), "Started control_loop_cb() callback.");

  auto request{std::make_shared<robot_patrol::srv::GetDirection::Request>()};
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture result_future{};
  std::future_status status{};
  std::string direction{};
  geometry_msgs::msg::Twist twist{};

  if (!client_->wait_for_service(kServiceTimeout)) {
    RCLCPP_WARN(this->get_logger(), "Service Unavailable.");
    goto end;
  }

  request->laser_data = last_laser_;
  result_future = client_->async_send_request(request);
  status = result_future.wait_for(kFutureTimeout);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Future timed out.");
    goto end;
  }
  direction = result_future.get()->direction;

  if (direction == "front") {
    twist.linear.x = 0.1;
    twist.angular.z = 0.0;
  } else if (direction == "left") {
    twist.linear.x = 0.1;
    twist.angular.z = 0.5;
  } else if (direction == "right") {
    twist.linear.x = 0.1;
    twist.angular.z = -0.5;
  } else {
    RCLCPP_WARN(this->get_logger(), "Got invalid direction:%s", direction);
  }

end:
  twist_publisher_->publish(twist);
  RCLCPP_DEBUG(this->get_logger(), "Exiting control_loop_cb() callback.");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const auto node{std::make_shared<PatrolNode>()};
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}