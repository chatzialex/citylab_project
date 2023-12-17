#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "robot_patrol/srv/detail/get_direction__builder.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <numeric>

using GetDirection = robot_patrol::srv::GetDirection;

constexpr char kNodeName[]{"direction_service"};
constexpr char kServiceName[]{"direction_service"};

class DirectionService : public rclcpp::Node {
public:
  DirectionService()
      : Node{kNodeName}, service_{create_service<GetDirection>(
                             kServiceName,
                             std::bind(&DirectionService::service_cb, this,
                                       std::placeholders::_1,
                                       std::placeholders::_2))} {}

private:
  constexpr static double kPi{3.1416};
  constexpr static double kAngleRightMin{-kPi / 2};
  constexpr static double kAngleFrontMin{kAngleRightMin + kPi / 3};
  constexpr static double kAngleLeftMin{kAngleFrontMin + kPi / 3};
  constexpr static double kAngleLeftMax{kPi / 2};
  static constexpr size_t kSlidingSize{50};

  void service_cb(const std::shared_ptr<GetDirection::Request> request,
                  std::shared_ptr<GetDirection::Response> response);

  rclcpp::Service<GetDirection>::SharedPtr service_{};

  std::vector<float> sliding_min_;
};

void DirectionService::service_cb(
    const std::shared_ptr<GetDirection::Request> request,
    std::shared_ptr<GetDirection::Response> response) {
  const auto angleToIndex{[&request](float angle) {
    return static_cast<size_t>(
        std::ceil((angle - request->laser_data.angle_min) /
                  request->laser_data.angle_increment));
  }};
  const auto isWithinRange{[&request](float range) {
    return request->laser_data.range_min <= range &&
           range <= request->laser_data.range_max;
  }};

  const auto i0{angleToIndex(kAngleRightMin)};
  const auto i1{angleToIndex(kAngleFrontMin)};
  const auto i2{angleToIndex(kAngleLeftMin)};
  const auto i3{angleToIndex(kAngleLeftMax)};

  sliding_min_.resize(i3 - i0);
  std::fill(sliding_min_.begin(), sliding_min_.end(), 0.0);
  float min{};

  for (size_t i{0}; i < sliding_min_.size(); ++i) {
    min = 0;
    for (size_t j{i0 + i - kSlidingSize}; j <= i0 + i + kSlidingSize; ++j) {
      if (!isWithinRange(request->laser_data.ranges[j])) {
        continue;
      }
      if (!min || (request->laser_data.ranges[j] < min)) {
        min = request->laser_data.ranges[j];
      }
    }
    sliding_min_[i] = min;
  }

  const std::array<double, 3> sums{
      std::accumulate(sliding_min_.begin() /* + i0 - i0*/,
                      sliding_min_.begin() + i1 - i0, 0.0),
      std::accumulate(sliding_min_.begin() + i1 - i0,
                      sliding_min_.begin() + i2 - i0, 0.0),
      std::accumulate(sliding_min_.begin() + i2 - i0,
                      sliding_min_.begin() + i3 - i0, 0.0)};

  const auto max_sum{std::max_element(sums.begin(), sums.end()) - sums.begin()};

  switch (max_sum) {
  case 0:
    response->direction = "right";
    break;
  case 1:
    response->direction = "front";
    break;
  case 2:
    response->direction = "left";
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<DirectionService>());

  rclcpp::shutdown();
}