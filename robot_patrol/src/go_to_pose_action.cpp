#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/action/detail/go_to_pose__struct.hpp"
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_patrol/action/go_to_pose.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

constexpr char kNodeName[]{"GoToPose"};
constexpr char kActionName[]{"go_to_pose"};
constexpr char kOdometryTopicName[]{"/odom"};
constexpr char kVelocityCommandTopicName[]{"cmd_vel"};

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose()
      : Node{kNodeName},
        subscription_{this->create_subscription<nav_msgs::msg::Odometry>(
            kOdometryTopicName, 1,
            std::bind(&GoToPose::subscription_cb, this,
                      std::placeholders::_1))},
        publisher_{this->create_publisher<geometry_msgs::msg::Twist>(
            kVelocityCommandTopicName, 1)},
        action_server_{rclcpp_action::create_server<GoToPoseAction>(
            this, kActionName,
            std::bind(&GoToPose::handle_goal, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&GoToPose::handle_cancel, this, std::placeholders::_1),
            std::bind(&GoToPose::handle_accepted, this,
                      std::placeholders::_1))} {}

private:
  // math constants
  constexpr static double kPi{3.1416};

  // settings
  constexpr static double kGoalPosTol{1e-1};
  constexpr static double kGoalThetaTol{1e-2};
  constexpr static double kLoopRate{5}; // Hz

  // subscription
  void subscription_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

  // action
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{};
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;

  geometry_msgs::msg::Pose2D desired_pos_{};
  geometry_msgs::msg::Pose2D current_pos_{};
};

void GoToPose::subscription_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
  double yaw{};
  double pitch{};
  double roll{};

  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  current_pos_.x = msg->pose.pose.position.x;
  current_pos_.y = msg->pose.pose.position.y;
  current_pos_.theta = yaw;
}

rclcpp_action::GoalResponse
GoToPose::handle_goal(const rclcpp_action::GoalUUID &uuid,
                      std::shared_ptr<const GoToPoseAction::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request.");
  desired_pos_ = goal->goal_pos;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
GoToPose::handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToPose::handle_accepted(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up
  // a new thread
  std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
}

void GoToPose::execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  auto feedback{std::make_shared<GoToPoseAction::Feedback>()};
  auto result{std::make_shared<GoToPoseAction::Result>()};
  geometry_msgs::msg::Twist twist{};

  rclcpp::Rate loop_rate(kLoopRate);

  while (rclcpp::ok()) {
    const double dx{desired_pos_.x - current_pos_.x};
    const double dy{desired_pos_.y - current_pos_.y};
    const double ds{std::sqrt(std::pow(dx, 2) + std::pow(dy, 2))};
    const bool goal_pos_reached{ds <= kGoalPosTol};
    const double theta_des{goal_pos_reached ? desired_pos_.theta
                                            : std::atan2(dy, dx)};
    const double dtheta{std::atan2(std::sin(theta_des - current_pos_.theta),
                                   std::cos(theta_des - current_pos_.theta))};
    const bool goal_theta_reached{std::abs(dtheta) <= kGoalThetaTol};
    const bool goal_reached{goal_pos_reached && goal_theta_reached};

    if (goal_handle->is_canceling()) {
      result->status = false;
      publisher_->publish(geometry_msgs::msg::Twist{});
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    feedback->current_pos = current_pos_;
    RCLCPP_DEBUG(this->get_logger(), "Publish feedback");
    goal_handle->publish_feedback(feedback);

    if (goal_reached) {
      result->status = true;
      publisher_->publish(geometry_msgs::msg::Twist{});
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      return;
    }

    twist.linear.x = goal_pos_reached ? 0.0 : 0.2;
    twist.angular.z = {dtheta / 2};
    publisher_->publish(twist);

    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<GoToPose>());

  rclcpp::shutdown();
}