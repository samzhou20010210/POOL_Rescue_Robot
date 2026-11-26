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

// thrust_allocator_node: a thin ROS 2 wrapper around ThrustAllocator.
//
// Subscribes:
//   wrench_command    geometry_msgs/WrenchStamped, requested body wrench in
//                     base_link (N, N*m). Messages in another frame are
//                     ignored.
// Publishes, at update_rate_hz:
//   thruster_command  rov_interfaces/ThrusterCommand, 8 forces in N, in the
//                     documented order (index i = thruster_0(i+1)).
//   wrench_achieved   geometry_msgs/WrenchStamped, B * forces (base_link).
//   wrench_residual   geometry_msgs/WrenchStamped, requested - achieved.
//   thrust_saturated  std_msgs/Bool, true if any thruster is at its limit.
//   diagnostics       diagnostic_msgs/DiagnosticArray, once per second and
//                     whenever the level changes: OK, WARN (saturated) or
//                     ERROR (allocation fault, e.g. a NaN request).
// On an allocation fault every thruster force is zero.
// If no wrench arrives for command_timeout_sec, the request becomes zero, so
// every thruster force becomes zero.
// Safety (design 5): while the latest rov_interfaces/SafetyStatus on
// "safety_status" (at most safety_status_max_age_s old) has zero_thrust set,
// every thruster force is 0, whatever the wrench command.
// Test hook: test_force_limit_scale (settable at run time) scales both force
// limits (1.0 = configured limits), to provoke genuine thrust saturation.

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_interfaces/msg/safety_status.hpp"
#include "rov_interfaces/msg/thruster_command.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rov_control
{

namespace
{

WrenchVector ToWrenchVector(const geometry_msgs::msg::Wrench & wrench)
{
  WrenchVector vector;
  vector << wrench.force.x, wrench.force.y, wrench.force.z,
    wrench.torque.x, wrench.torque.y, wrench.torque.z;
  return vector;
}

geometry_msgs::msg::WrenchStamped ToWrenchStamped(
  const WrenchVector & vector, const rclcpp::Time & stamp, const std::string & frame_id)
{
  geometry_msgs::msg::WrenchStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.wrench.force.x = vector(0);
  message.wrench.force.y = vector(1);
  message.wrench.force.z = vector(2);
  message.wrench.torque.x = vector(3);
  message.wrench.torque.y = vector(4);
  message.wrench.torque.z = vector(5);
  return message;
}

}  // namespace

class ThrustAllocatorNode : public rclcpp::Node
{
public:
  ThrustAllocatorNode()
  : Node("thrust_allocator_node")
  {
    // Parameters (units in the names). See config/thrust_allocator.yaml.
    std::string thruster_file = declare_parameter("thruster_config_file", std::string(""));
    if (thruster_file.empty()) {
      thruster_file = ament_index_cpp::get_package_share_directory("rov_description") +
        "/config/thrusters_v4.yaml";
    }
    AllocatorSettings settings;
    settings.max_forward_force =
      declare_parameter("max_forward_force_n", settings.max_forward_force);
    settings.max_reverse_force =
      declare_parameter("max_reverse_force_n", settings.max_reverse_force);
    const std::vector<double> default_weights(
      settings.weights.data(), settings.weights.data() + settings.weights.size());
    const std::vector<double> weights = declare_parameter("wrench_weights", default_weights);
    if (weights.size() != 6) {
      throw std::invalid_argument("wrench_weights must have 6 values [Fx, Fy, Fz, Mx, My, Mz]");
    }
    for (int i = 0; i < 6; ++i) {
      settings.weights(i) = weights[i];
    }
    settings.regularization = declare_parameter("regularization", settings.regularization);
    settings.saturation_tolerance =
      declare_parameter("saturation_tolerance_n", settings.saturation_tolerance);
    const double update_rate_hz = declare_parameter("update_rate_hz", 50.0);
    command_timeout_sec_ = declare_parameter("command_timeout_sec", 0.2);
    if (update_rate_hz <= 0.0 || command_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("update_rate_hz and command_timeout_sec must be positive");
    }

    safety_max_age_ = declare_parameter("safety_status_max_age_s", 1.0);
    const std::vector<Thruster> thrusters = LoadThrusterLayout(thruster_file);
    matrix_ = BuildAllocationMatrix(thrusters);
    settings_ = settings;
    allocator_ = std::make_unique<ThrustAllocator>(matrix_, settings_);
    declare_parameter("test_force_limit_scale", 1.0);
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const rclcpp::Parameter & parameter : parameters) {
          if (parameter.get_name() != "test_force_limit_scale") {
            continue;
          }
          const double scale = parameter.as_double();
          if (!(scale > 0.0) || scale > 1.0) {
            result.successful = false;
            result.reason = "test_force_limit_scale must be in (0, 1]";
            return result;
          }
          AllocatorSettings scaled = settings_;
          scaled.max_forward_force *= scale;
          scaled.max_reverse_force *= scale;
          allocator_ = std::make_unique<ThrustAllocator>(matrix_, scaled);
          RCLCPP_WARN(
            get_logger(), "Test hook: force limits scaled by %.2f (+%.1f / -%.1f N)", scale,
            scaled.max_forward_force, scaled.max_reverse_force);
        }
        return result;
      });

    // Relative topic names, so they follow the node's namespace.
    wrench_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "wrench_command", 10,
      [this](const geometry_msgs::msg::WrenchStamped & message) {OnWrenchCommand(message);});
    thruster_publisher_ =
      create_publisher<rov_interfaces::msg::ThrusterCommand>("thruster_command", 10);
    achieved_publisher_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_achieved", 10);
    residual_publisher_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_residual", 10);
    saturated_publisher_ = create_publisher<std_msgs::msg::Bool>("thrust_saturated", 10);
    safety_subscription_ = create_subscription<rov_interfaces::msg::SafetyStatus>(
      "safety_status", 10, [this](const rov_interfaces::msg::SafetyStatus & status) {
        zero_thrust_ = status.zero_thrust;
        safety_time_ = now();
      });
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

    // The timer runs on the node's clock, so it also works with use_sim_time.
    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz);
    update_timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() {OnUpdateTimer();});

    RCLCPP_INFO(
      get_logger(),
      "thrust_allocator_node: %s, limits +%.1f / -%.1f N, %.1f Hz, command timeout %.2f s",
      thruster_file.c_str(), settings.max_forward_force, settings.max_reverse_force,
      update_rate_hz, command_timeout_sec_);
  }

private:
  void OnWrenchCommand(const geometry_msgs::msg::WrenchStamped & message)
  {
    if (message.header.frame_id != kBaseFrame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring wrench_command in frame '%s'; it must be '%s'",
        message.header.frame_id.c_str(), kBaseFrame);
      return;
    }
    requested_wrench_ = ToWrenchVector(message.wrench);
    last_command_time_ = now();
    has_command_ = true;
  }

  void OnUpdateTimer()
  {
    const rclcpp::Time stamp = now();

    // A stale (or missing) command means "no force": all thrusters go to 0.
    WrenchVector requested = WrenchVector::Zero();
    if (has_command_ && (stamp - last_command_time_).seconds() < command_timeout_sec_) {
      requested = requested_wrench_;
    }
    if (zero_thrust_ && (stamp - safety_time_).seconds() <= safety_max_age_) {
      requested.setZero();  // safety monitor: all thruster forces 0
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Safety monitor: zero thrust commanded");
    }

    const AllocationResult result = allocator_->Allocate(requested);
    if (result.fault != AllocationFault::kNone) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Allocation fault (%s): commanding zero thrust",
        AllocationFaultToString(result.fault));
    } else if (result.saturated) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%d thruster(s) at their limit; residual force %.2f N, moment %.2f N*m",
        result.num_saturated, result.residual.head<3>().norm(), result.residual.tail<3>().norm());
    }

    rov_interfaces::msg::ThrusterCommand command;
    command.header.stamp = stamp;
    command.header.frame_id = kBaseFrame;
    for (int i = 0; i < kNumThrusters; ++i) {
      command.force[i] = result.forces(i);
    }
    thruster_publisher_->publish(command);
    achieved_publisher_->publish(ToWrenchStamped(result.achieved, stamp, kBaseFrame));
    residual_publisher_->publish(ToWrenchStamped(result.residual, stamp, kBaseFrame));
    std_msgs::msg::Bool saturated;
    saturated.data = result.saturated;
    saturated_publisher_->publish(saturated);
    PublishDiagnostics(stamp, result);
  }

  // Report the allocator's health on the standard diagnostics topic.
  void PublishDiagnostics(const rclcpp::Time & stamp, const AllocationResult & result)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    DiagnosticStatus status;
    status.name = std::string(get_name()) + ": thrust allocation";
    status.hardware_id = "rov_v4";
    if (result.fault != AllocationFault::kNone) {
      status.level = DiagnosticStatus::ERROR;
      status.message = std::string("fault: ") + AllocationFaultToString(result.fault) +
        "; zero thrust commanded";
    } else if (result.saturated) {
      status.level = DiagnosticStatus::WARN;
      status.message = "thrust saturated";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "ok";
    }

    // Publish once per second, or at once when the level changes.
    const bool level_changed = status.level != last_diagnostic_level_;
    if (!level_changed && has_published_diagnostics_ &&
      (stamp - last_diagnostics_time_).seconds() < 1.0)
    {
      return;
    }
    status.values.push_back(MakeKeyValue("fault", AllocationFaultToString(result.fault)));
    status.values.push_back(MakeKeyValue("saturated_thrusters", result.num_saturated));
    status.values.push_back(
      MakeKeyValue("residual_force_n", result.residual.head<3>().norm()));
    status.values.push_back(
      MakeKeyValue("residual_moment_nm", result.residual.tail<3>().norm()));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
    last_diagnostic_level_ = status.level;
    last_diagnostics_time_ = stamp;
    has_published_diagnostics_ = true;
  }

  template<typename T>
  static diagnostic_msgs::msg::KeyValue MakeKeyValue(const std::string & key, const T & value)
  {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key;
    if constexpr (std::is_convertible_v<T, std::string>) {
      key_value.value = value;
    } else {
      key_value.value = std::to_string(value);
    }
    return key_value;
  }

  static constexpr const char * kBaseFrame = "base_link";

  std::unique_ptr<ThrustAllocator> allocator_;
  AllocationMatrix matrix_;
  AllocatorSettings settings_;
  double safety_max_age_ = 1.0;
  bool zero_thrust_ = false;
  rclcpp::Time safety_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<rov_interfaces::msg::SafetyStatus>::SharedPtr safety_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  double command_timeout_sec_;

  // Latest request.
  WrenchVector requested_wrench_ = WrenchVector::Zero();
  rclcpp::Time last_command_time_;
  bool has_command_ = false;

  // Last diagnostics sent (to limit the rate).
  uint8_t last_diagnostic_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
  rclcpp::Time last_diagnostics_time_;
  bool has_published_diagnostics_ = false;

  // ROS interfaces, kept as members so they stay alive.
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
  rclcpp::Publisher<rov_interfaces::msg::ThrusterCommand>::SharedPtr thruster_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr achieved_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr residual_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr saturated_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace rov_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_control::ThrustAllocatorNode>());
  rclcpp::shutdown();
  return 0;
}
