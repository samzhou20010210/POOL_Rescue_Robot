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

// dynamics_sim_node: thrust-driven planar simulator of the V4 ROV.
//
// Unlike sim_node (which follows velocity commands exactly), this node
// applies thruster forces to a hydrodynamic model (see planar_dynamics.hpp):
// the vehicle accelerates, is slowed by drag, and coasts when the thrust
// stops.
//
// Subscribes:
//   thruster_command  rov_interfaces/ThrusterCommand, 8 forces in N
//                     (frame_id must be base_link).
// Publishes, at publish_rate_hz:
//   odom              nav_msgs/Odometry: pose in odom_frame_id, twist in
//                     base_link (velocity over ground).
//   TF                odom_frame_id -> base_frame_id.
// The model is integrated with RK4 steps of at most 1 / integration_rate_hz.
// If no command arrives for cmd_timeout_sec, the thrust command becomes zero.
// The vehicle starts at rest at initial_pose [x, y, yaw] in the odom frame.
// Service:
//   reset_vehicle     std_srvs/Trigger: back to initial_pose, at rest, with
//                     zero thrust (used by the open-loop tests).

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_interfaces/msg/thruster_command.hpp"
#include "rov_simulation/planar_dynamics.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace rov_simulation
{

class DynamicsSimNode : public rclcpp::Node
{
public:
  DynamicsSimNode()
  : Node("dynamics_sim_node")
  {
    // Parameters (see config/dynamics_sim.yaml). Empty file names mean the
    // V4 files installed by rov_description.
    const std::string description_dir =
      ament_index_cpp::get_package_share_directory("rov_description") + "/config/";
    std::string hydrodynamics_file =
      declare_parameter("hydrodynamics_file", std::string(""));
    std::string thruster_file = declare_parameter("thruster_config_file", std::string(""));
    if (hydrodynamics_file.empty()) {
      hydrodynamics_file = description_dir + "hydrodynamics_v4.yaml";
    }
    if (thruster_file.empty()) {
      thruster_file = description_dir + "thrusters_v4.yaml";
    }
    const double integration_rate_hz = declare_parameter("integration_rate_hz", 200.0);
    const double publish_rate_hz = declare_parameter("publish_rate_hz", 50.0);
    const double cmd_timeout_sec = declare_parameter("cmd_timeout_sec", 0.2);
    const std::vector<double> water_current =
      declare_parameter("water_current", std::vector<double>{0.0, 0.0});
    const std::vector<double> initial_pose =
      declare_parameter("initial_pose", std::vector<double>{0.0, 0.0, 0.0});
    odom_frame_id_ = declare_parameter("odom_frame_id", std::string("odom"));
    base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));
    if (integration_rate_hz <= 0.0 || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("integration_rate_hz and publish_rate_hz must be positive");
    }
    if (water_current.size() != 2) {
      throw std::invalid_argument("water_current must be [x, y] in m/s");
    }
    if (initial_pose.size() != 3) {
      throw std::invalid_argument("initial_pose must be [x, y, yaw] in m and rad");
    }
    initial_state_.pose = Eigen::Vector3d(initial_pose[0], initial_pose[1], initial_pose[2]);

    // Everything physical comes from the YAML files.
    const HydrodynamicParams params = LoadHydrodynamicParams(hydrodynamics_file);
    const AllocationMatrix allocation_matrix =
      rov_control::BuildAllocationMatrix(rov_control::LoadThrusterLayout(thruster_file));
    const double thrust_time_constant = LoadThrusterTimeConstant(thruster_file);
    const PlanarDynamics dynamics(
      params, allocation_matrix, thrust_time_constant,
      Eigen::Vector2d(water_current[0], water_current[1]));

    // The simulator counts seconds from the node's start.
    start_time_ = now();
    simulator_ = std::make_unique<PlanarSimulator>(
      dynamics, 1.0 / integration_rate_hz, cmd_timeout_sec, 0.0);
    simulator_->Reset(initial_state_);

    // Relative topic names, so they follow the node's namespace.
    command_subscription_ = create_subscription<rov_interfaces::msg::ThrusterCommand>(
      "thruster_command", 10,
      [this](const rov_interfaces::msg::ThrusterCommand & message) {OnThrusterCommand(message);});
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "reset_vehicle",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {OnReset(*response);});

    // Test hook: publish_odometry = false stops odometry and TF (the vehicle
    // keeps moving), to exercise odometry-timeout handling.
    publish_odometry_ = declare_parameter("publish_odometry", true);
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const rclcpp::Parameter & parameter : parameters) {
          if (parameter.get_name() == "publish_odometry") {
            publish_odometry_ = parameter.as_bool();
            RCLCPP_WARN(
              get_logger(), "Test hook: odometry %s", publish_odometry_ ? "resumed" : "stopped");
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // The timer runs on the node's clock, so it also works with use_sim_time.
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    publish_timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() {OnPublishTimer();});

    RCLCPP_INFO(
      get_logger(),
      "dynamics_sim_node: m %.1f kg, I_z %.2f kg m^2, tau_m %.3f s, integrate %.0f Hz, "
      "publish %.0f Hz, cmd timeout %.2f s, current (%.2f, %.2f) m/s",
      params.mass, params.inertia_z, thrust_time_constant, integration_rate_hz,
      publish_rate_hz, cmd_timeout_sec, water_current[0], water_current[1]);
    RCLCPP_INFO(
      get_logger(), "parameter files: %s, %s", hydrodynamics_file.c_str(), thruster_file.c_str());
  }

private:
  // Seconds since the node started, on the node's clock.
  double SimTime() {return (now() - start_time_).seconds();}

  void OnThrusterCommand(const rov_interfaces::msg::ThrusterCommand & message)
  {
    if (message.header.frame_id != base_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring thruster_command in frame '%s'; expected '%s'",
        message.header.frame_id.c_str(), base_frame_id_.c_str());
      return;
    }
    ThrustVector forces;
    for (int i = 0; i < kNumThrusters; ++i) {
      forces(i) = message.force[i];
    }
    // Integrate up to now with the old command, then switch to the new one.
    const double time = SimTime();
    simulator_->AdvanceTo(time);
    if (!simulator_->SetCommand(forces, time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring thruster_command with NaN or infinite force");
    }
  }

  void OnReset(std_srvs::srv::Trigger::Response & response)
  {
    simulator_->AdvanceTo(SimTime());
    simulator_->Reset(initial_state_);
    was_fresh_ = false;
    response.success = true;
    response.message = "vehicle reset to initial_pose, at rest, zero thrust";
    RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
  }

  void OnPublishTimer()
  {
    const double time = SimTime();
    simulator_->AdvanceTo(time);

    // Log when the command times out, once per timeout.
    const bool fresh = simulator_->CommandIsFresh(time);
    if (was_fresh_ && !fresh) {
      RCLCPP_WARN(get_logger(), "thruster_command timed out: thrust cut, vehicle coasting");
    }
    was_fresh_ = fresh;

    if (publish_odometry_) {
      PublishPose(now());
    }
  }

  void PublishPose(const rclcpp::Time & stamp)
  {
    const PlanarState & state = simulator_->state();
    const Eigen::Vector3d velocity = simulator_->dynamics().GroundVelocity(state);
    const double yaw = state.pose(2);

    // A rotation of yaw around the z axis, written as a quaternion.
    geometry_msgs::msg::Quaternion orientation;
    orientation.z = std::sin(yaw / 2.0);
    orientation.w = std::cos(yaw / 2.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = state.pose(0);
    odom.pose.pose.position.y = state.pose(1);
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = velocity(0);  // Twist is given in base_link.
    odom.twist.twist.linear.y = velocity(1);
    odom.twist.twist.angular.z = velocity(2);
    odom_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = state.pose(0);
    transform.transform.translation.y = state.pose(1);
    transform.transform.rotation = orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_id_;
  std::string base_frame_id_;
  rclcpp::Time start_time_;
  std::unique_ptr<PlanarSimulator> simulator_;
  bool publish_odometry_ = true;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  PlanarState initial_state_;
  bool was_fresh_ = false;

  // ROS interfaces, kept as members so they stay alive.
  rclcpp::Subscription<rov_interfaces::msg::ThrusterCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rov_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_simulation::DynamicsSimNode>());
  rclcpp::shutdown();
  return 0;
}
