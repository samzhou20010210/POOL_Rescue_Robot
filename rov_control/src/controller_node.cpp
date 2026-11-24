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

// controller_node: drives an omnidirectional planar vehicle (the ROV can move
// forward, sideways, and turn independently) along a path.
//
// It offers a "follow_path" action (nav2_msgs/action/FollowPath). The path
// can have one pose (move straight to it) or many poses in order (for
// example a route from Nav2's planner that goes around obstacles).
// On every timer tick the node:
//   1. stops if the goal was canceled, odometry is stale, or the robot pose
//      cannot be transformed into the path's frame,
//   2. translates toward a "target point" on the path a short distance
//      ahead, moving that point forward along the path in order. The
//      position error is rotated into base_link and sent as linear.x
//      (forward) and linear.y (left), while angular.z holds the heading the
//      robot had when the goal started,
//   3. once the target point is the last pose and the robot is close enough
//      to it, stops translating and turns in place to the goal's yaw,
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
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

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

// A 2D pose: position and heading.
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node")
  {
    const double control_rate_hz = declare_parameter("control_rate_hz", 20.0);
    position_tolerance_ = declare_parameter("position_tolerance", 0.10);  // m
    yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.05);  // rad
    // Limit on the total planar speed, hypot(linear.x, linear.y).
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.5);  // m/s
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.0);  // rad/s
    linear_gain_ = declare_parameter("linear_gain", 1.0);
    angular_gain_ = declare_parameter("angular_gain", 2.0);
    // How far ahead on the path the robot aims. Small values follow the
    // path closely (keeping the turns around obstacles).
    lookahead_distance_ = declare_parameter("lookahead_distance", 0.3);  // m
    odom_timeout_sec_ = declare_parameter("odom_timeout_sec", 0.5);
    if (control_rate_hz <= 0.0 || odom_timeout_sec_ <= 0.0 || lookahead_distance_ <= 0.0) {
      throw std::invalid_argument(
              "control_rate_hz, odom_timeout_sec and lookahead_distance must be positive");
    }

    // TF: used to express the robot pose in the path's frame (e.g. map).
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

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
    kFollowPath,
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
    // Keep the whole pose with its frame, so it can be transformed later.
    robot_pose_odom_.header = odom.header;
    robot_pose_odom_.pose = odom.pose.pose;
    // Time of arrival on this node's clock, used to detect stale odometry.
    last_odom_time_ = now();
    has_odom_ = true;
  }

  // ---- Action server callbacks (all quick, never blocking) ----

  // Decide whether to accept a new goal.
  rclcpp_action::GoalResponse OnGoalRequest(const FollowPath::Goal & goal)
  {
    const auto & path = goal.path;
    if (path.poses.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: path is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (path.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: path has no frame_id");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Every pose must be in the path's frame (an empty pose frame means "same as the path").
    for (const auto & pose : path.poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) {
        RCLCPP_WARN(
          get_logger(), "Rejecting goal: pose frame '%s' differs from path frame '%s'",
          pose.header.frame_id.c_str(), path.header.frame_id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    const auto & q = path.poses.back().pose.orientation;
    if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-6) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: final orientation quaternion is all zeros");
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

    const auto & path = goal_handle->get_goal()->path;
    path_frame_id_ = path.header.frame_id;
    path_.clear();
    for (const auto & pose : path.poses) {
      path_.push_back(
        {pose.pose.position.x, pose.pose.position.y, YawFromQuaternion(pose.pose.orientation)});
    }
    // length_to_end_[i] = path length from pose i to the last pose. Used for
    // the speed, so a route that ends near its start is still driven at speed.
    length_to_end_.assign(path_.size(), 0.0);
    for (size_t i = path_.size() - 1; i > 0; --i) {
      length_to_end_[i - 1] = length_to_end_[i] + DistanceTo(path_[i - 1], path_[i]);
    }
    target_index_ = 0;
    phase_ = Phase::kFollowPath;
    has_hold_yaw_ = false;  // Set from the robot's heading on the first control tick.
    active_goal_ = goal_handle;
    RCLCPP_INFO(
      get_logger(), "New goal: %zu poses in '%s', final x=%.2f, y=%.2f, yaw=%.2f rad",
      path_.size(), path_frame_id_.c_str(), path_.back().x, path_.back().y, path_.back().yaw);
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

    Pose2D robot;
    std::string tf_error;
    if (!GetRobotPoseInPathFrame(robot, tf_error)) {
      FinishGoal(FollowPath::Result::TF_ERROR, tf_error, GoalOutcome::kAborted);
      return;
    }

    // Heading to hold while translating: the robot's heading when the goal started.
    if (!has_hold_yaw_) {
      hold_yaw_ = robot.yaw;
      has_hold_yaw_ = true;
    }

    const Pose2D & final_pose = path_.back();
    geometry_msgs::msg::Twist cmd;  // All fields start at zero.

    if (phase_ == Phase::kFollowPath) {
      // Move the target forward along the path (in order) while it is
      // closer than the lookahead distance. The last pose is never skipped.
      const size_t last_index = path_.size() - 1;
      while (target_index_ < last_index &&
        DistanceTo(robot, path_[target_index_]) < lookahead_distance_)
      {
        ++target_index_;
      }

      // Finish driving only on the final segment (target is the last pose),
      // so a route that passes near its end point earlier is not cut short.
      const bool on_final_segment = target_index_ == last_index;
      if (on_final_segment && DistanceTo(robot, final_pose) <= position_tolerance_) {
        phase_ = Phase::kAlignYaw;  // Close enough; now only fix the heading.
      } else {
        // Position error to the target point, in the path frame ...
        const Pose2D & target = path_[target_index_];
        const double error_x = target.x - robot.x;
        const double error_y = target.y - robot.y;
        // ... rotated by -yaw into base_link (x forward, y left).
        const double cos_yaw = std::cos(robot.yaw);
        const double sin_yaw = std::sin(robot.yaw);
        const double error_forward = cos_yaw * error_x + sin_yaw * error_y;
        const double error_left = -sin_yaw * error_x + cos_yaw * error_y;
        const double error_length = std::hypot(error_forward, error_left);
        if (error_length > 1e-6) {
          // Move along the error direction. Slow down as the distance left
          // along the route gets small.
          const double speed = std::min(
            linear_gain_ * RemainingDistance(robot), max_linear_speed_);
          cmd.linear.x = speed * error_forward / error_length;
          cmd.linear.y = speed * error_left / error_length;
        }
        // Keep the starting heading while translating.
        cmd.angular.z = angular_gain_ * NormalizeAngle(hold_yaw_ - robot.yaw);
      }
    }

    if (phase_ == Phase::kAlignYaw) {
      const double yaw_error = NormalizeAngle(final_pose.yaw - robot.yaw);
      if (std::abs(yaw_error) <= yaw_tolerance_) {
        FinishGoal(FollowPath::Result::NONE, "", GoalOutcome::kSucceeded);
        return;
      }
      cmd.angular.z = angular_gain_ * yaw_error;  // Turn in place; no translation.
    }

    // linear.x and linear.y were already limited together (by speed above).
    cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
    cmd_vel_publisher_->publish(cmd);

    auto feedback = std::make_shared<FollowPath::Feedback>();
    feedback->distance_to_goal = static_cast<float>(RemainingDistance(robot));
    feedback->speed = static_cast<float>(std::hypot(cmd.linear.x, cmd.linear.y));
    active_goal_->publish_feedback(feedback);
  }

  // Express the latest odometry pose in the path's frame using TF.
  // Returns false (and fills error) if the transform is not available.
  bool GetRobotPoseInPathFrame(Pose2D & robot, std::string & error)
  {
    geometry_msgs::msg::PoseStamped pose_in_path_frame;
    try {
      // Latest available transform; lookupTransform does not wait here.
      const auto transform = tf_buffer_->lookupTransform(
        path_frame_id_, robot_pose_odom_.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(robot_pose_odom_, pose_in_path_frame, transform);
    } catch (const tf2::TransformException & ex) {
      error = "Cannot transform robot pose into '" + path_frame_id_ + "': " + ex.what();
      return false;
    }
    robot.x = pose_in_path_frame.pose.position.x;
    robot.y = pose_in_path_frame.pose.position.y;
    robot.yaw = YawFromQuaternion(pose_in_path_frame.pose.orientation);
    return true;
  }

  static double DistanceTo(const Pose2D & a, const Pose2D & b)
  {
    return std::hypot(b.x - a.x, b.y - a.y);
  }

  // Distance left along the route: robot -> target point -> ... -> last pose.
  double RemainingDistance(const Pose2D & robot) const
  {
    return DistanceTo(robot, path_[target_index_]) + length_to_end_[target_index_];
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
  double position_tolerance_;
  double yaw_tolerance_;
  double max_linear_speed_;
  double max_angular_speed_;
  double linear_gain_;
  double angular_gain_;
  double lookahead_distance_;
  double odom_timeout_sec_;

  // Latest odometry (in the odom frame).
  bool has_odom_ = false;
  geometry_msgs::msg::PoseStamped robot_pose_odom_;
  rclcpp::Time last_odom_time_;

  // Active goal (empty when idle).
  std::shared_ptr<GoalHandle> active_goal_;
  std::string path_frame_id_;
  std::vector<Pose2D> path_;
  std::vector<double> length_to_end_;
  size_t target_index_ = 0;
  Phase phase_ = Phase::kFollowPath;
  // Heading held while translating (the robot's heading when the goal started).
  bool has_hold_yaw_ = false;
  double hold_yaw_ = 0.0;

  // ROS interfaces, kept as members so they stay alive.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
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
