#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>

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
  constexpr static double kAngleLeftMin{-kPi / 2};
  constexpr static double kAngleFrontMin{kAngleLeftMin + kPi / 3};
  constexpr static double kAngleRightMin{kAngleFrontMin + kPi / 3};
  constexpr static double kAngleRightMax{kPi / 2};

  void service_cb(const std::shared_ptr<GetDirection::Request> request,
                  std::shared_ptr<GetDirection::Response> response);

  rclcpp::Service<GetDirection>::SharedPtr service_{};
};

void DirectionService::service_cb(
    const std::shared_ptr<GetDirection::Request> request,
    std::shared_ptr<GetDirection::Response> response) {

  const auto pos{[request](double angle) {
    const auto index{
        static_cast<size_t>(std::ceil((angle - request->laser_data.angle_min) /
                                      request->laser_data.angle_increment))};
    return request->laser_data.ranges.begin() + index;
  }};

  const auto isWithinRange{[request](float range) {
    return request->laser_data.range_min <= range &&
           range <= request->laser_data.range_max;
  }};

  const auto range_aware_sum{[&isWithinRange](
                                 std::vector<float>::iterator start,
                                 std::vector<float>::iterator end) {
    return std::accumulate(start, end, 0.0, [&isWithinRange](float a, float b) {
      return isWithinRange(b) ? a + b : a;
    });
  }};

  const std::array<double, 3> sums{
      range_aware_sum(pos(kAngleLeftMin), pos(kAngleFrontMin)),
      range_aware_sum(pos(kAngleFrontMin), pos(kAngleRightMin)),
      range_aware_sum(pos(kAngleRightMin), pos(kAngleRightMax))};

  const auto max_sum{std::max_element(sums.begin(), sums.end()) - sums.begin()};

  switch (max_sum) {
  case 0:
    response->direction = "left";
    break;
  case 1:
    response->direction = "front";
    break;
  case 2:
    response->direction = "right";
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<DirectionService>());

  rclcpp::shutdown();
}