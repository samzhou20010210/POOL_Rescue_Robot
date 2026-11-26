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

// trajectory_controller_node: tracks a time-parameterized trajectory with
// PD + model feedforward (TrackingController) and outputs a body wrench.
//
// Action server:
//   follow_trajectory  rov_interfaces/action/FollowTrajectory
// Subscribes:
//   odom               nav_msgs/Odometry (pose in odom, twist in base_link)
// Publishes:
//   wrench_command     geometry_msgs/WrenchStamped in base_link, at
//                      control_rate_hz while a goal is active. Fz, Mx and My
//                      are 0, so the allocator keeps roll and pitch moments
//                      at zero. When a goal ends (except by preemption, where
//                      the new goal takes over at once, or by success with
//                      hold_after_success, see Hold), one zero wrench is sent;
//                      after that nothing is published and the allocator's
//                      command timeout keeps the thrust at zero.
// Uses TF (trajectory frame <- odom frame) to express the robot pose in the
// trajectory's frame (for example map).
//
// Goal rules:
//   - header.stamp is the start time t0; a zero stamp means "start now".
//   - A new goal is validated when it arrives: an invalid trajectory is
//     REJECTED and the active goal keeps running untouched. A valid new goal
//     replaces the active one, which ends with PREEMPTED.
//   - Position error above max_position_error_m, or no odometry for
//     odom_timeout_sec, ends with TRACKING_ERROR_EXCEEDED and zero wrench.
//   - After the last point the controller keeps holding the final pose.
//     SUCCEEDED once the robot has stayed within the goal tolerances and
//     stopped for stop_hold_sec (a single slow sample can be a speed zero
//     crossing while still braking); TRACKING_ERROR_EXCEEDED if that does
//     not happen within settle_timeout_sec.
//   - Hold (hold_after_success): after SUCCEEDED the controller keeps
//     regulating to the final pose (zero velocity) without a goal, until a
//     new goal arrives or odometry times out (then one zero wrench). This is
//     the "hold position" after a stop or arrival (design 3 item 7, 5).
// All the work happens in the timer callback; callbacks never wait.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rov_control/planar_model.hpp"
#include "rov_control/tracking_controller.hpp"
#include "rov_interfaces/action/follow_trajectory.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace rov_control
{

using FollowTrajectory = rov_interfaces::action::FollowTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

class TrajectoryControllerNode : public rclcpp::Node
{
public:
  TrajectoryControllerNode()
  : Node("trajectory_controller_node")
  {
    // Parameters (see config/trajectory_controller.yaml).
    std::string hydrodynamics_file = declare_parameter("hydrodynamics_file", std::string(""));
    if (hydrodynamics_file.empty()) {
      hydrodynamics_file = ament_index_cpp::get_package_share_directory("rov_description") +
        "/config/hydrodynamics_v4.yaml";
    }
    TrackingGains gains;
    gains.position = declare_parameter("position_gain", gains.position);
    gains.velocity = declare_parameter("velocity_gain", gains.velocity);
    gains.yaw = declare_parameter("yaw_gain", gains.yaw);
    gains.yaw_rate = declare_parameter("yaw_rate_gain", gains.yaw_rate);
    gains.use_feedforward = declare_parameter("use_feedforward", gains.use_feedforward);
    const double control_rate_hz = declare_parameter("control_rate_hz", 50.0);
    const double feedback_rate_hz = declare_parameter("feedback_rate_hz", 10.0);
    odom_timeout_sec_ = declare_parameter("odom_timeout_sec", 0.2);
    max_position_error_ = declare_parameter("max_position_error_m", 1.0);
    goal_position_tolerance_ = declare_parameter("goal_position_tolerance_m", 0.15);
    goal_yaw_tolerance_ = declare_parameter("goal_yaw_tolerance_rad", 0.1);
    stop_speed_ = declare_parameter("stop_speed_mps", 0.05);
    stop_yaw_rate_ = declare_parameter("stop_yaw_rate_radps", 0.05);
    settle_timeout_sec_ = declare_parameter("settle_timeout_sec", 10.0);
    stop_hold_sec_ = declare_parameter("stop_hold_sec", 0.5);
    hold_after_success_ = declare_parameter("hold_after_success", true);
    base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));
    if (control_rate_hz <= 0.0 || feedback_rate_hz <= 0.0 || odom_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("rates and odom_timeout_sec must be positive");
    }
    feedback_every_ =
      std::max(1, static_cast<int>(std::lround(control_rate_hz / feedback_rate_hz)));

    controller_ = std::make_unique<TrackingController>(
      PlanarModel(LoadHydrodynamicParams(hydrodynamics_file)), gains);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Relative names, so they follow the node's namespace.
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](const nav_msgs::msg::Odometry & message) {
        odom_ = message;
        odom_receive_time_ = now();
        has_odom_ = true;
      });
    wrench_publisher_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_command", 10);
    action_server_ = rclcpp_action::create_server<FollowTrajectory>(
      this, "follow_trajectory",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowTrajectory::Goal> goal) {
        // Reject an invalid trajectory before it can preempt the active goal.
        std::vector<TrajectorySample> unused;
        const std::string problem = ConvertGoal(*goal, &unused);
        if (!problem.empty()) {
          RCLCPP_WARN(get_logger(), "Rejected a trajectory goal: %s", problem.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GoalHandle> goal) {pending_goal_ = goal;});

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() {OnTimer();});

    RCLCPP_INFO(
      get_logger(), "trajectory_controller_node: Kp %.0f, Kd %.0f, Kp_yaw %.0f, Kd_yaw %.0f, "
      "feedforward %s, %.0f Hz", gains.position, gains.velocity, gains.yaw, gains.yaw_rate,
      gains.use_feedforward ? "on" : "off", control_rate_hz);
  }

private:
  // Goal -> samples; empty string if valid, otherwise the problem.
  static std::string ConvertGoal(
    const FollowTrajectory::Goal & goal, std::vector<TrajectorySample> * samples)
  {
    samples->clear();
    for (const auto & point : goal.trajectory.points) {
      TrajectorySample sample;
      sample.time = rclcpp::Duration(point.time_from_start).seconds();
      sample.position = Eigen::Vector2d(point.x, point.y);
      sample.velocity = Eigen::Vector2d(point.vx, point.vy);
      sample.acceleration = Eigen::Vector2d(point.ax, point.ay);
      sample.yaw = point.yaw;
      sample.yaw_rate = point.yaw_rate;
      samples->push_back(sample);
    }
    std::string problem = ValidateTrajectory(*samples);
    if (problem.empty() && goal.trajectory.header.frame_id.empty()) {
      problem = "header.frame_id is empty";
    }
    return problem;
  }

  // Keep the final pose of the last succeeded goal (no goal active).
  void HoldStep()
  {
    if (!has_odom_ || (now() - odom_receive_time_).seconds() > odom_timeout_sec_) {
      hold_active_ = false;
      PublishWrench(Eigen::Vector3d::Zero());
      RCLCPP_WARN(get_logger(), "Hold ended: odometry timeout (zero wrench)");
      return;
    }
    Eigen::Vector3d pose;
    if (!RobotPose(pose)) {
      return;
    }
    const auto & twist = odom_.twist.twist;
    const Eigen::Vector3d body_velocity(twist.linear.x, twist.linear.y, twist.angular.z);
    PublishWrench(
      controller_->Compute(pose, body_velocity, SampleTrajectory(hold_samples_, 0.0)).wrench);
  }

  // Robot pose [x, y, yaw] in the trajectory frame, from the latest odometry.
  bool RobotPose(Eigen::Vector3d & pose)
  {
    geometry_msgs::msg::PoseStamped in_odom;
    in_odom.header.frame_id = odom_.header.frame_id;
    in_odom.pose = odom_.pose.pose;
    geometry_msgs::msg::PoseStamped in_frame;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        frame_id_, odom_.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(in_odom, in_frame, transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No transform %s <- %s: %s", frame_id_.c_str(),
        odom_.header.frame_id.c_str(), error.what());
      return false;
    }
    pose << in_frame.pose.position.x, in_frame.pose.position.y,
      tf2::getYaw(in_frame.pose.orientation);
    return true;
  }

  void OnTimer()
  {
    if (pending_goal_) {
      StartGoal(pending_goal_);
      pending_goal_.reset();
    }
    if (!active_goal_) {
      if (hold_active_) {
        HoldStep();
      }
      return;
    }
    if (active_goal_->is_canceling()) {
      Finish(FollowTrajectory::Result::CANCELED, "canceled");
      return;
    }
    if (!has_odom_ || (now() - odom_receive_time_).seconds() > odom_timeout_sec_) {
      Finish(FollowTrajectory::Result::TRACKING_ERROR_EXCEEDED, "odometry timeout");
      return;
    }
    Eigen::Vector3d pose;
    if (!RobotPose(pose)) {
      // Give TF a moment (e.g. the static map -> odom just started).
      if ((now() - goal_start_wall_).seconds() > 1.0) {
        Finish(FollowTrajectory::Result::TRACKING_ERROR_EXCEEDED, "no transform to the robot");
      } else {
        PublishWrench(Eigen::Vector3d::Zero());
      }
      return;
    }
    const auto & twist = odom_.twist.twist;
    const Eigen::Vector3d body_velocity(twist.linear.x, twist.linear.y, twist.angular.z);

    const double t = (now() - start_time_).seconds();
    const TrackingOutput output =
      controller_->Compute(pose, body_velocity, SampleTrajectory(samples_, t));
    PublishWrench(output.wrench);

    const double position_error = output.position_error.norm();
    last_position_error_ = position_error;
    last_yaw_error_ = output.yaw_error;
    if (t <= duration_) {
      error_square_sum_ += position_error * position_error;
      error_max_ = std::max(error_max_, position_error);
      ++error_count_;
    }
    if (position_error > max_position_error_) {
      Finish(
        FollowTrajectory::Result::TRACKING_ERROR_EXCEEDED,
        "position error " + std::to_string(position_error) + " m above the limit");
      return;
    }

    if (++tick_ % feedback_every_ == 0) {
      auto feedback = std::make_shared<FollowTrajectory::Feedback>();
      feedback->progress = duration_ > 0.0 ? std::clamp(t / duration_, 0.0, 1.0) : 1.0;
      feedback->time_remaining = std::max(duration_ - t, 0.0);
      feedback->position_error = position_error;
      feedback->yaw_error = output.yaw_error;
      active_goal_->publish_feedback(feedback);
    }

    if (t >= duration_) {
      const double speed = std::hypot(body_velocity(0), body_velocity(1));
      const bool at_goal = position_error <= goal_position_tolerance_ &&
        std::abs(output.yaw_error) <= goal_yaw_tolerance_ &&
        speed <= stop_speed_ && std::abs(body_velocity(2)) <= stop_yaw_rate_;
      if (!at_goal) {
        at_goal_since_.reset();
      } else if (!at_goal_since_) {
        at_goal_since_ = t;
      }
      if (at_goal_since_ && t - *at_goal_since_ >= stop_hold_sec_) {
        Finish(FollowTrajectory::Result::SUCCEEDED, "reached the end and stopped");
      } else if (t > duration_ + settle_timeout_sec_) {
        Finish(
          FollowTrajectory::Result::TRACKING_ERROR_EXCEEDED,
          "did not settle at the end within settle_timeout_sec");
      }
    }
  }

  void StartGoal(const std::shared_ptr<GoalHandle> & goal)
  {
    // Validated again (it was checked on arrival): an invalid goal ends
    // without touching the active one.
    std::vector<TrajectorySample> samples;
    const std::string problem = ConvertGoal(*goal->get_goal(), &samples);
    if (!problem.empty()) {
      auto result = std::make_shared<FollowTrajectory::Result>();
      result->error_code = FollowTrajectory::Result::INVALID_TRAJECTORY;
      result->message = problem;
      goal->abort(result);
      return;
    }
    if (active_goal_) {
      Finish(FollowTrajectory::Result::PREEMPTED, "replaced by a newer trajectory");
    }
    hold_active_ = false;
    active_goal_ = goal;
    const auto & trajectory = goal->get_goal()->trajectory;
    samples_ = std::move(samples);
    frame_id_ = trajectory.header.frame_id;
    const rclcpp::Time stamp(trajectory.header.stamp, get_clock()->get_clock_type());
    start_time_ = stamp.nanoseconds() == 0 ? now() : stamp;
    duration_ = samples_.back().time;
    goal_start_wall_ = now();
    error_square_sum_ = 0.0;
    error_max_ = 0.0;
    error_count_ = 0;
    tick_ = 0;
    at_goal_since_.reset();
    RCLCPP_INFO(
      get_logger(), "New trajectory: %zu points, %.2f s, frame %s", samples_.size(), duration_,
      frame_id_.c_str());
  }

  // End the active goal with the given code, and send a zero wrench.
  void Finish(uint8_t code, const std::string & reason)
  {
    auto result = std::make_shared<FollowTrajectory::Result>();
    result->error_code = code;
    result->success = code == FollowTrajectory::Result::SUCCEEDED;
    result->final_position_error = last_position_error_;
    result->final_yaw_error = last_yaw_error_;
    const double rms = error_count_ > 0 ? std::sqrt(error_square_sum_ / error_count_) : 0.0;
    char stats[128];
    std::snprintf(
      stats, sizeof(stats), "; tracking error RMS %.3f m, max %.3f m", rms, error_max_);
    result->message = reason + stats;

    if (code == FollowTrajectory::Result::SUCCEEDED) {
      active_goal_->succeed(result);
      if (hold_after_success_) {
        // Hold the final pose (at rest) until a new goal arrives.
        TrajectorySample hold = samples_.back();
        hold.time = 0.0;
        hold.velocity.setZero();
        hold.acceleration.setZero();
        hold.yaw_rate = 0.0;
        TrajectorySample later = hold;
        later.time = 1.0;
        hold_samples_ = {hold, later};
        hold_active_ = true;
      }
    } else if (code == FollowTrajectory::Result::CANCELED) {
      active_goal_->canceled(result);
    } else {
      active_goal_->abort(result);
    }
    RCLCPP_INFO(
      get_logger(), "Trajectory ended (code %u): %s; final error %.3f m, %.3f rad", code,
      result->message.c_str(), last_position_error_, last_yaw_error_);
    active_goal_.reset();
    last_position_error_ = 0.0;
    last_yaw_error_ = 0.0;
    // A preempted goal is replaced in the same timer tick, so no zero wrench
    // (it would be a one-message thrust gap at a seamless handoff).
    if (code != FollowTrajectory::Result::PREEMPTED && !hold_active_) {
      PublishWrench(Eigen::Vector3d::Zero());
    }
  }

  void PublishWrench(const Eigen::Vector3d & planar)
  {
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = now();
    message.header.frame_id = base_frame_id_;
    message.wrench.force.x = planar(0);
    message.wrench.force.y = planar(1);
    message.wrench.torque.z = planar(2);
    wrench_publisher_->publish(message);
  }

  // Settings.
  double odom_timeout_sec_;
  double max_position_error_;
  double goal_position_tolerance_;
  double goal_yaw_tolerance_;
  double stop_speed_;
  double stop_yaw_rate_;
  double settle_timeout_sec_;
  double stop_hold_sec_;
  bool hold_after_success_ = true;
  bool hold_active_ = false;
  std::vector<TrajectorySample> hold_samples_;  // final pose at rest
  std::string base_frame_id_;
  int feedback_every_ = 5;
  std::unique_ptr<TrackingController> controller_;

  // Latest odometry.
  nav_msgs::msg::Odometry odom_;
  rclcpp::Time odom_receive_time_;
  bool has_odom_ = false;

  // Goals: pending is set by the action callback, used by the timer.
  std::shared_ptr<GoalHandle> pending_goal_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::vector<TrajectorySample> samples_;
  std::string frame_id_;
  rclcpp::Time start_time_;
  rclcpp::Time goal_start_wall_;
  double duration_ = 0.0;
  int tick_ = 0;
  std::optional<double> at_goal_since_;  // t when the robot came to rest at the goal

  // Tracking statistics of the active goal (during the trajectory).
  double last_position_error_ = 0.0;
  double last_yaw_error_ = 0.0;
  double error_square_sum_ = 0.0;
  double error_max_ = 0.0;
  int error_count_ = 0;

  // ROS interfaces, kept as members so they stay alive.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  rclcpp_action::Server<FollowTrajectory>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rov_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_control::TrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
