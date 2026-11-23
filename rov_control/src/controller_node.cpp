// Copyright 2026 sam
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// controller_node: drives the planar simulator to one goal pose.
//
// It offers a "follow_path" action (nav2_msgs/action/FollowPath). For this
// milestone the path must contain exactly ONE pose in the odom frame.
// On every timer tick the node:
//   1. stops if the goal was canceled or odometry is stale,
//   2. drives toward the target position (turning first if needed),
//   3. once close enough, turns in place to the requested final yaw,
//   4. publishes zero velocity and reports success.
//
// No callback ever waits or sleeps: the action callbacks only store the
// goal, and all the work happens in the timer callback.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rov_control
{

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

// Heading (rotation about z) of a quaternion, in radians.
double YawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Wrap an angle into the range [-pi, pi].
double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node")
  {
    const double control_rate_hz = declare_parameter("control_rate_hz", 20.0);
    odom_frame_id_ = declare_parameter("odom_frame_id", std::string("odom"));
    position_tolerance_ = declare_parameter("position_tolerance", 0.10);  // m
    yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.05);  // rad
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.5);  // m/s
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.0);  // rad/s
    linear_gain_ = declare_parameter("linear_gain", 1.0);
    angular_gain_ = declare_parameter("angular_gain", 2.0);
    // If the heading error is larger than this, turn in place before driving.
    turn_in_place_angle_ = declare_parameter("turn_in_place_angle", 0.5);  // rad
    odom_timeout_sec_ = declare_parameter("odom_timeout_sec", 0.5);
    if (control_rate_hz <= 0.0 || odom_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("control_rate_hz and odom_timeout_sec must be positive");
    }

    // Relative names, so they follow the node's namespace.
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [this](const nav_msgs::msg::Odometry & odom) {OnOdom(odom);});
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<FollowPath>(
      this, "follow_path",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal> goal) {
        return OnGoalRequest(*goal);
      },
      [this](std::shared_ptr<GoalHandle>) {return OnCancelRequest();},
      [this](std::shared_ptr<GoalHandle> goal_handle) {OnGoalAccepted(goal_handle);});

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    control_timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() {OnControlTimer();});

    RCLCPP_INFO(get_logger(), "controller_node ready, action server 'follow_path'");
  }

private:
  enum class Phase
  {
    kDriveToPosition,
    kAlignYaw,
  };

  enum class GoalOutcome
  {
    kSucceeded,
    kAborted,
    kCanceled,
  };

  // ---- Subscription callback ----

  void OnOdom(const nav_msgs::msg::Odometry & odom)
  {
    x_ = odom.pose.pose.position.x;
    y_ = odom.pose.pose.position.y;
    yaw_ = YawFromQuaternion(odom.pose.pose.orientation);
    // Time of arrival on this node's clock, used to detect stale odometry.
    last_odom_time_ = now();
    has_odom_ = true;
  }

  // ---- Action server callbacks (all quick, never blocking) ----

  // Decide whether to accept a new goal. Only a single odom-frame pose is supported.
  rclcpp_action::GoalResponse OnGoalRequest(const FollowPath::Goal & goal)
  {
    const auto & poses = goal.path.poses;
    if (poses.size() != 1) {
      RCLCPP_WARN(
        get_logger(), "Rejecting goal: path has %zu poses, only 1 is supported", poses.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    // The pose's own frame wins; an empty pose frame means "same as the path".
    const std::string & frame_id = poses[0].header.frame_id.empty() ?
      goal.path.header.frame_id : poses[0].header.frame_id;
    if (frame_id != odom_frame_id_) {
      RCLCPP_WARN(
        get_logger(), "Rejecting goal: frame '%s' is not supported, use '%s'",
        frame_id.c_str(), odom_frame_id_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto & q = poses[0].pose.orientation;
    if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-6) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: orientation quaternion is all zeros");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse OnCancelRequest()
  {
    // The timer notices the cancel request and stops the robot.
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Store the new goal; the timer does the actual driving.
  void OnGoalAccepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    // Only one goal is active at a time: a new goal replaces the old one.
    if (active_goal_ && active_goal_->is_active()) {
      FinishGoal(
        FollowPath::Result::UNKNOWN, "Replaced by a newer goal", GoalOutcome::kAborted);
    }

    const auto & target = goal_handle->get_goal()->path.poses[0].pose;
    target_x_ = target.position.x;
    target_y_ = target.position.y;
    target_yaw_ = YawFromQuaternion(target.orientation);
    phase_ = Phase::kDriveToPosition;
    active_goal_ = goal_handle;
    RCLCPP_INFO(
      get_logger(), "New goal: x=%.2f, y=%.2f, yaw=%.2f rad", target_x_, target_y_, target_yaw_);
  }

  // ---- Control loop ----

  void OnControlTimer()
  {
    if (!active_goal_) {
      return;  // Nothing to do; do not publish, so manual teleop still works.
    }

    if (active_goal_->is_canceling()) {
      FinishGoal(FollowPath::Result::NONE, "Canceled", GoalOutcome::kCanceled);
      return;
    }

    const bool odom_is_fresh =
      has_odom_ && (now() - last_odom_time_).seconds() < odom_timeout_sec_;
    if (!odom_is_fresh) {
      FinishGoal(
        FollowPath::Result::UNKNOWN, "Odometry is missing or stale", GoalOutcome::kAborted);
      return;
    }

    const double dx = target_x_ - x_;
    const double dy = target_y_ - y_;
    const double distance = std::hypot(dx, dy);

    geometry_msgs::msg::Twist cmd;  // All fields start at zero.

    if (phase_ == Phase::kDriveToPosition) {
      if (distance <= position_tolerance_) {
        phase_ = Phase::kAlignYaw;  // Close enough; now only fix the heading.
      } else {
        const double heading_error = NormalizeAngle(std::atan2(dy, dx) - yaw_);
        cmd.angular.z = angular_gain_ * heading_error;
        // Drive forward only when roughly facing the target.
        if (std::abs(heading_error) < turn_in_place_angle_) {
          cmd.linear.x = linear_gain_ * distance * std::cos(heading_error);
        }
      }
    }

    if (phase_ == Phase::kAlignYaw) {
      const double yaw_error = NormalizeAngle(target_yaw_ - yaw_);
      if (std::abs(yaw_error) <= yaw_tolerance_) {
        FinishGoal(FollowPath::Result::NONE, "", GoalOutcome::kSucceeded);
        return;
      }
      cmd.angular.z = angular_gain_ * yaw_error;
    }

    cmd.linear.x = std::clamp(cmd.linear.x, -max_linear_speed_, max_linear_speed_);
    cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
    cmd_vel_publisher_->publish(cmd);

    auto feedback = std::make_shared<FollowPath::Feedback>();
    feedback->distance_to_goal = static_cast<float>(distance);
    feedback->speed = static_cast<float>(cmd.linear.x);
    active_goal_->publish_feedback(feedback);
  }

  // Stop the robot, report the result, and forget the goal.
  void FinishGoal(uint16_t error_code, const std::string & message, GoalOutcome outcome)
  {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());  // Zero velocity.

    auto result = std::make_shared<FollowPath::Result>();
    result->error_code = error_code;
    result->error_msg = message;
    switch (outcome) {
      case GoalOutcome::kSucceeded:
        active_goal_->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
        break;
      case GoalOutcome::kAborted:
        active_goal_->abort(result);
        RCLCPP_WARN(get_logger(), "Goal aborted: %s", message.c_str());
        break;
      case GoalOutcome::kCanceled:
        active_goal_->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        break;
    }
    active_goal_.reset();
  }

  // Settings (from parameters).
  std::string odom_frame_id_;
  double position_tolerance_;
  double yaw_tolerance_;
  double max_linear_speed_;
  double max_angular_speed_;
  double linear_gain_;
  double angular_gain_;
  double turn_in_place_angle_;
  double odom_timeout_sec_;

  // Latest odometry.
  bool has_odom_ = false;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  rclcpp::Time last_odom_time_;

  // Active goal (empty when idle).
  std::shared_ptr<GoalHandle> active_goal_;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_yaw_ = 0.0;
  Phase phase_ = Phase::kDriveToPosition;

  // ROS interfaces, kept as members so they stay alive.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace rov_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
