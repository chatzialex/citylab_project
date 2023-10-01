#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class PatrolNode : public rclcpp::Node {
public:
  PatrolNode()
      : Node{"patrol"},
        scan_subscriber_{this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PatrolNode::scan_cb, this, std::placeholders::_1))},
        twist_publisher_{
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10)},
        control_loop_timer_{this->create_wall_timer(
            100ms, std::bind(&PatrolNode::control_loop_cb, this))} {}

private:
  static constexpr double kPi{3.1416};

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_loop_cb();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscriber_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{};
  rclcpp::TimerBase::SharedPtr control_loop_timer_{};
  double direction_{0};
};

void PatrolNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Started scan_cb() callback.");

  if (msg->angle_increment <= 0) {
    RCLCPP_WARN(this->get_logger(),
                "Found unexpected value of angle_increment<=0 in the laser "
                "scan message.");
    return;
  }

  if (msg->angle_min > -kPi / 2) {
    RCLCPP_WARN(this->get_logger(),
                "The angle of -pi/2 is outside of the laser scan range.");
    return;
  }

  if (msg->angle_min + msg->angle_increment * msg->ranges.size() < kPi / 2) {
    RCLCPP_WARN(this->get_logger(),
                "The angle of pi/2 is outside of the laser scan range.");
    return;
  }

  const size_t index_start{
      static_cast<size_t>((-kPi / 2 - msg->angle_min) / msg->angle_increment)};
  const size_t index_end{
      static_cast<size_t>((kPi / 2 - msg->angle_min) / msg->angle_increment)};

  RCLCPP_DEBUG(this->get_logger(), "index_start=%ld, index_end=%ld",
               index_start, index_end);

  size_t index_max{index_start};
  bool found_one{false};
  for (size_t i{index_start}; i < index_end; ++i) {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) {
      continue;
    }
    found_one = true;
    if (msg->ranges[i] > msg->ranges[index_max]) {
      index_max = i;
    }
  }

  direction_ =
      found_one ? msg->angle_min + index_max * msg->angle_increment : 0;
  RCLCPP_DEBUG(this->get_logger(), "found_one=%d, direction_=%f", found_one,
               direction_);
}

void PatrolNode::control_loop_cb() {
  geometry_msgs::msg::Twist twist{};
  twist.linear.x = 0.1;
  twist.angular.z = direction_ / 2;
  RCLCPP_DEBUG(this->get_logger(),
               "Publishing a twist message with angular.z=%f", twist.angular.z);
  twist_publisher_->publish(twist);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNode>());
  rclcpp::shutdown();
  return 0;
}