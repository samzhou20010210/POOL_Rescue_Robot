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

// sim_node: a very simple planar (2D) omnidirectional vehicle simulator.
//
// It listens to velocity commands on "cmd_vel", given in base_link:
//   - linear.x  is the forward speed in m/s
//   - linear.y  is the sideways speed in m/s (positive = to the left)
//   - angular.z is the yaw rate in rad/s
// On every timer tick it moves the vehicle by (velocity * elapsed time), then
// publishes the new pose on "odom" and as the odom -> base_link TF.
// If no command arrives for "cmd_timeout_sec" seconds, the vehicle stops.

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace rov_simulation
{

class SimNode : public rclcpp::Node
{
public:
  SimNode()
  : Node("sim_node")
  {
    // Parameters can be changed at startup, for example:
    //   ros2 run rov_simulation sim_node --ros-args -p cmd_timeout_sec:=1.0
    const double update_rate_hz = declare_parameter("update_rate_hz", 50.0);
    cmd_timeout_sec_ = declare_parameter("cmd_timeout_sec", 0.5);
    odom_frame_id_ = declare_parameter("odom_frame_id", std::string("odom"));
    base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));
    if (update_rate_hz <= 0.0 || cmd_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("update_rate_hz and cmd_timeout_sec must be positive");
    }

    // Relative topic names, so they follow the node's namespace.
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist & cmd) {OnCmdVel(cmd);});
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // The timer runs on the node's clock, the same clock now() uses, so it
    // also works with simulated time (use_sim_time:=true).
    last_update_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz);
    update_timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() {OnUpdateTimer();});

    RCLCPP_INFO(
      get_logger(), "sim_node started at the origin (rate %.1f Hz, cmd timeout %.2f s)",
      update_rate_hz, cmd_timeout_sec_);
  }

private:
  // Remember the newest command and when it arrived.
  void OnCmdVel(const geometry_msgs::msg::Twist & cmd)
  {
    forward_speed_ = cmd.linear.x;
    lateral_speed_ = cmd.linear.y;
    yaw_rate_ = cmd.angular.z;
    last_cmd_time_ = now();
    has_received_cmd_ = true;
  }

  // Move the vehicle forward in time, then publish the new pose.
  void OnUpdateTimer()
  {
    const rclcpp::Time current_time = now();
    const double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    if (dt <= 0.0) {
      return;  // The clock did not move forward (e.g. simulated time paused).
    }

    // Only apply the command while it is fresh.
    double forward = 0.0;
    double lateral = 0.0;
    double turn_rate = 0.0;
    if (has_received_cmd_ && (current_time - last_cmd_time_).seconds() < cmd_timeout_sec_) {
      forward = forward_speed_;
      lateral = lateral_speed_;
      turn_rate = yaw_rate_;
    }

    // Rotate the base_link velocity into the odom frame. Use the heading
    // halfway through the step; this is more accurate than the heading at
    // the start when the vehicle is turning.
    const double mid_yaw = yaw_ + 0.5 * turn_rate * dt;
    x_ += (forward * std::cos(mid_yaw) - lateral * std::sin(mid_yaw)) * dt;
    y_ += (forward * std::sin(mid_yaw) + lateral * std::cos(mid_yaw)) * dt;
    yaw_ = std::atan2(std::sin(yaw_ + turn_rate * dt), std::cos(yaw_ + turn_rate * dt));

    PublishPose(current_time, forward, lateral, turn_rate);
  }

  // Publish odom and the TF using the same timestamp.
  void PublishPose(
    const rclcpp::Time & stamp, double forward, double lateral, double turn_rate)
  {
    // A rotation of yaw_ around the z axis, written as a quaternion.
    geometry_msgs::msg::Quaternion orientation;
    orientation.z = std::sin(yaw_ / 2.0);
    orientation.w = std::cos(yaw_ / 2.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = forward;  // Twist is given in base_link.
    odom.twist.twist.linear.y = lateral;
    odom.twist.twist.angular.z = turn_rate;
    odom_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation = orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  // Settings (from parameters).
  double cmd_timeout_sec_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  // Latest command.
  double forward_speed_ = 0.0;
  double lateral_speed_ = 0.0;
  double yaw_rate_ = 0.0;
  bool has_received_cmd_ = false;
  rclcpp::Time last_cmd_time_;

  // Simulated pose in the odom frame. The vehicle starts at the origin.
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  rclcpp::Time last_update_time_;

  // ROS interfaces, kept as members so they stay alive.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace rov_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_simulation::SimNode>());
  rclcpp::shutdown();
  return 0;
}
