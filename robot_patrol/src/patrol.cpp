#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

class PatrolNode : public rclcpp::Node {
public:
  PatrolNode() : Node("patrol") {}

private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNode>());
  rclcpp::shutdown();
  return 0;
}