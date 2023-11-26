#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>

using namespace std::chrono_literals;

class PatrolNode : public rclcpp::Node {
public:
  PatrolNode()
      : Node{"patrol"}, callback_group_{this->create_callback_group(
                            rclcpp::CallbackGroupType::Reentrant)},
        options_{[this]() {
          rclcpp::SubscriptionOptions opt;
          opt.callback_group = callback_group_;
          return opt;
        }()},
        scan_subscriber_{this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PatrolNode::scan_cb, this, std::placeholders::_1),
            options_)},
        twist_publisher_{
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1)},
        control_loop_timer_{this->create_wall_timer(
            200ms, std::bind(&PatrolNode::control_loop_cb, this),
            callback_group_)},
        tf_broadcaster_{
            std::make_unique<tf2_ros::TransformBroadcaster>(*this)} {}

private:
  // math constants
  static constexpr double kPi{3.1416};

  // settings
  static constexpr double kAngleMin{-kPi / 2};
  static constexpr double kAngleMax{kPi / 2};
  static constexpr size_t kSlidingSize{50};

  // helper functions
  bool valid(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  size_t findMaxPos(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // callbacks
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_loop_cb();

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::SubscriptionOptions options_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscriber_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{};
  rclcpp::TimerBase::SharedPtr control_loop_timer_{};
  double direction_{0};
  std::vector<std::optional<double>> sliding_min_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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

size_t
PatrolNode::findMaxPos(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  const auto angleToIndex{[msg](double angle) {
    return static_cast<size_t>(
        std::ceil((angle - msg->angle_min) / msg->angle_increment));
  }};
  const auto isWithinRange{[msg](double range) {
    return msg->range_min <= range && range <= msg->range_max;
  }};

  const auto i0{angleToIndex(kAngleMin)};
  const auto i1{angleToIndex(kAngleMax)};

  sliding_min_.resize(i1 - i0);
  std::optional<double> min{};

  for (size_t i{0}; i < sliding_min_.size(); ++i) {
    min = std::nullopt;
    for (size_t j{i0 + i - kSlidingSize}; j <= i0 + i + kSlidingSize; ++j) {
      if (!isWithinRange(msg->ranges[j])) {
        continue;
      }
      if (!min || (msg->ranges[j] < min)) {
        min = msg->ranges[j];
      }
    }
    sliding_min_[i] = min;
  }

  return std::distance(
             sliding_min_.begin(),
             std::max_element(sliding_min_.begin(), sliding_min_.end())) +
         i0;
}

void PatrolNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Started scan_cb() callback.");

  if (!valid(msg)) {
    RCLCPP_WARN(this->get_logger(), "Message not valid, ignoring.");
  }

  const auto max_pos{findMaxPos(msg)};

  if (msg->ranges[max_pos]) {
    direction_ = msg->angle_min + max_pos * msg->angle_increment;
  }

  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "goal";
  constexpr double length{1.0};
  t.transform.translation.x = std::cos(direction_) * length;
  t.transform.translation.y = std::sin(direction_) * length;
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, direction_);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
  RCLCPP_INFO(this->get_logger(), "max=%f at direction_=%f [deg]",
              msg->ranges[max_pos], 180 * direction_ / kPi);
}

void PatrolNode::control_loop_cb() {
  RCLCPP_INFO(this->get_logger(), "Started control_loop_cb() callback.");

  geometry_msgs::msg::Twist twist{};
  // twist.linear.x = 0.1;
  twist.angular.z = direction_ / 2;
  twist_publisher_->publish(twist);

  RCLCPP_INFO(this->get_logger(), "Exiting control_loop_cb() callback.");
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