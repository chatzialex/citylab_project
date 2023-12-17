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
#include <cstddef>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <robot_patrol/srv/get_direction.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

class Direction {
public:
  static const Direction left;
  static const Direction front;
  static const Direction right;

  const std::string &get() const { return value; }

private:
  explicit Direction(std::string_view direction) : value{direction} {};

  std::string value;
};

const Direction Direction::left("left");
const Direction Direction::front("front");
const Direction Direction::right("right");

class DirectionBuffer {
public:
  DirectionBuffer(size_t buffer_size);

  bool isFull() const;
  bool isEmpty() const;

  const std::string &getMostFrequentValue() const;
  size_t getValueCount(const std::string &value) const {
    return values_count_.at(value);
  }
  size_t getDirectionChangesSignCount(int sign) const {
    return direction_changes_sign_count_.at(sign);
  }

  void push(const Direction &direction);

private:
  int compDirectionChange(const std::string &direction_initial,
                          const std::string &direction_final);
  int compDirectionChangeSign(const std::string &direction_initial,
                              const std::string &direction_final);

  size_t head_{0};
  size_t tail_{0};
  std::vector<std::string> values_{};
  std::map<std::string, size_t> values_count_ = {{Direction::left.get(), 0},
                                                 {Direction::front.get(), 0},
                                                 {Direction::right.get(), 0}};
  std::map<int, size_t> direction_changes_sign_count_ = {
      {-1, 0}, {0, 0}, {1, 0}};
};

DirectionBuffer::DirectionBuffer(size_t buffer_size) {
  if (buffer_size == 0) {
    throw(std::logic_error{
        "Attempted to initialize DirectionBuffer with size 0."});
  }

  const auto size{buffer_size + 1};
  values_.resize(size);
}

bool DirectionBuffer::isEmpty() const { return head_ == tail_; }

bool DirectionBuffer::isFull() const {
  return head_ == (tail_ == 0 ? values_.size() - 1 : tail_ - 1);
}

const std::string &DirectionBuffer::getMostFrequentValue() const {
  using pairType = decltype(values_count_)::value_type;

  return std::max_element(values_count_.begin(), values_count_.end(),
                          [](const pairType &p1, const pairType &p2) {
                            return p1.second < p2.second;
                          })
      ->first;
}

void DirectionBuffer::push(const Direction &direction) {
  const auto direction_initial{[this](size_t pos) -> decltype(auto) {
    return isEmpty() ? Direction::front.get()
                     : values_[pos == 0 ? values_.size() - 1 : pos - 1];
  }};
  const auto direction_final{
      [this](size_t pos) -> decltype(auto) { return values_[pos]; }};

  const auto &direction_new{direction.get()};

  ++values_count_.at(direction_new);
  ++direction_changes_sign_count_.at(
      compDirectionChangeSign(direction_initial(head_), direction_new));

  if (isFull()) {
    --values_count_.at(direction_final(tail_));
    --direction_changes_sign_count_.at(compDirectionChangeSign(
        direction_initial(tail_), direction_final(tail_)));
    tail_ = (tail_ + 1) % values_.size();
  }
  values_[head_] = direction_new;
  head_ = (head_ + 1) % values_.size();
}

int DirectionBuffer::compDirectionChange(const std::string &direction_initial,
                                         const std::string &direction_final) {
  if (direction_initial == direction_final) {
    return 0;
  } else if (direction_initial == Direction::left.get()) {
    if (direction_final == Direction::front.get()) {
      return 1;
    } else /* direction_final == Direction::right.get() */ {
      return 2;
    }
  } else if (direction_initial == Direction::front.get()) {
    if (direction_final == Direction::left.get()) {
      return -1;
    } else /* direction_final == Direction::right.get() */ {
      return 1;
    }
  } else /* direction_initial == Direction.right.get() */ {
    if (direction_final == Direction::front.get()) {
      return -1;
    } else /* direction_final == Direction::left.get() */ {
      return -2;
    }
  }
}

int DirectionBuffer::compDirectionChangeSign(
    const std::string &direction_initial, const std::string &direction_final) {
  if (direction_initial != Direction::left.get() &&
      direction_final == Direction::left.get()) {
    return -1;
  } else if (direction_initial != Direction::right.get() &&
             direction_final == Direction::right.get()) {
    return 1;
  } else {
    return 0;
  }
}

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
  std::chrono::nanoseconds kServiceTimeout{50ms};
  std::chrono::nanoseconds kFutureTimeout{50ms};
  static constexpr size_t kBufferSize{10};
  static constexpr size_t kMaxDirectionChanges{0};

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
  sensor_msgs::msg::LaserScan last_laser_{};

  DirectionBuffer buffer_{kBufferSize};
  std::string direction_{""};
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
  auto t_start{std::chrono::steady_clock::now()};

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

  if (direction == "left") {
    buffer_.push(Direction::left);
  } else if (direction == "front") {
    buffer_.push(Direction::front);
  } else if (direction == "right") {
    buffer_.push(Direction::right);
  } else {
    RCLCPP_WARN(this->get_logger(), "Got invalid direction:%s", direction);
    goto end;
  }

  if ((direction == "left" &&
       buffer_.getDirectionChangesSignCount(1) > kMaxDirectionChanges) ||
      (direction == "right" &&
       buffer_.getDirectionChangesSignCount(-1) > kMaxDirectionChanges)) {
    RCLCPP_INFO(this->get_logger(), "Direction jittering detected.");
  } else {
    direction_ = direction;
  }

  twist.linear.x = 0.1;
  if (direction_ == "front") {
    twist.angular.z = 0.0;
  } else if (direction_ == "left") {
    twist.angular.z = 0.5;
  } else /* direction_ == "right" */ {
    twist.angular.z = -0.5;
  };

end:
  /*std::cout << "to_left:" << buffer_.getDirectionChangesSignCount().at(-1)
            << " no_change:" << buffer_.getDirectionChangesSignCount().at(0)
            << " to_right:" << buffer_.getDirectionChangesSignCount().at(1)
            << "\n";*/

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