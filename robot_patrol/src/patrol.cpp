#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>

class PatrolNode : public rclcpp::Node {
public:
  PatrolNode()
      : Node{"patrol"},
        twist_publisher_{
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10)},
        scan_subscriber_{this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PatrolNode::scan_cb, this, std::placeholders::_1))} {}

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscriber_{};
};

void PatrolNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  (void)msg;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNode>());
  rclcpp::shutdown();
  return 0;
}