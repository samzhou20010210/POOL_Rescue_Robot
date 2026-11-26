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

// safety_monitor_node: independent safety monitor (design 5, step 10).
//
// At rate_hz it evaluates every design 5 rule (safety_rules.hpp) on the
// latest inputs and publishes rov_interfaces/SafetyStatus on
// "safety_status"; the actions act directly on planning and control:
//   - speed_limit, stop_hold, replan_now, replan_slow: trajectory generator;
//   - zero_thrust: thrust allocator (all forces 0);
//   - abort (hold / return / surface and return): mission manager.
// Every rule trigger and clear is logged with its time and reason, and
// published as one line on "safety_report" (std_msgs/String).
//
// Inputs: odom (receive time, velocity), TF map -> base_link (pose), map
// (wall distance of the stowed hull outline, terminal_pose.yaml),
// swimmer_tracks (map frame; stale or out-of-order tracks dropped as in the
// planner), shore_heartbeat (std_msgs/Header, by its stamp),
// thrust_saturated (std_msgs/Bool), battery_state (sensor_msgs/BatteryState),
// leak_alarm (rov_interfaces/LeakAlarm), mission_state (phase: heartbeat
// rules in surface phases, mission timeout from the alarm), planner_status
// (verified arrival stops the mission timeout).

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/msg/leak_alarm.hpp"
#include "rov_interfaces/msg/mission_state.hpp"
#include "rov_interfaces/msg/planner_status.hpp"
#include "rov_interfaces/msg/safety_status.hpp"
#include "rov_interfaces/msg/swimmer_track_array.hpp"
#include "rov_mission/safety_rules.hpp"
#include "rov_planning/swimmer_prediction.hpp"
#include "rov_planning/terminal_pose.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace rov_mission
{

namespace
{

std::string ShareFile(const std::string & path, const std::string & package, const std::string & f)
{
  return path.empty() ?
         ament_index_cpp::get_package_share_directory(package) + "/config/" + f : path;
}

double Seconds(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec + stamp.nanosec * 1e-9;
}

}  // namespace

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : Node("safety_monitor_node")
  {
    SafetySettings & s = settings_;
    s.swimmer_slow_radius = declare_parameter("swimmer_slow_radius_m", s.swimmer_slow_radius);
    s.swimmer_slow_speed = declare_parameter("swimmer_slow_speed_mps", s.swimmer_slow_speed);
    s.swimmer_clearance = declare_parameter("swimmer_clearance_m", s.swimmer_clearance);
    s.prediction_horizon = declare_parameter("prediction_horizon_s", s.prediction_horizon);
    s.prediction_step = declare_parameter("prediction_step_s", s.prediction_step);
    s.wall_distance = declare_parameter("wall_distance_m", s.wall_distance);
    s.wall_speed = declare_parameter("wall_speed_mps", s.wall_speed);
    s.odometry_timeout = declare_parameter("odometry_timeout_s", s.odometry_timeout);
    s.heartbeat_stop = declare_parameter("heartbeat_stop_s", s.heartbeat_stop);
    s.heartbeat_abort = declare_parameter("heartbeat_abort_s", s.heartbeat_abort);
    s.saturation_duration = declare_parameter("saturation_duration_s", s.saturation_duration);
    s.slow_speed_factor = declare_parameter("slow_speed_factor", s.slow_speed_factor);
    s.battery_minimum = declare_parameter("battery_minimum", s.battery_minimum);
    s.mission_timeout = declare_parameter("mission_timeout_s", s.mission_timeout);
    s.sensor_max_age = declare_parameter("sensor_max_age_s", s.sensor_max_age);
    s.future_tolerance = declare_parameter("future_tolerance_s", s.future_tolerance);
    const std::string problem = ValidateSafetySettings(s);
    if (!problem.empty()) {
      throw std::invalid_argument(problem);
    }
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    const double rate_hz = declare_parameter("rate_hz", 10.0);
    terminal_ = rov_planning::LoadTerminalPoseSettings(ShareFile(
        declare_parameter("terminal_pose_file", std::string("")), "rov_planning",
        "terminal_pose.yaml"));
    // Wall check: the outline must keep wall_distance_m; search that far.
    terminal_.wall_clearance = s.wall_distance;
    footprint_.front = terminal_.bow_offset + terminal_.stowed_arm_protrusion;
    footprint_.rear = terminal_.stern_offset;
    footprint_.half_width = terminal_.hull_half_width;
    swimmers_.emplace(rov_planning::LoadSwimmerPredictionSettings(ShareFile(
        declare_parameter("swimmer_prediction_file", std::string("")), "rov_planning",
        "swimmer_prediction.yaml")));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry & m) {
          odom_ = m;
          odom_time_ = now().seconds();
        }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Header>(
        "shore_heartbeat", 10, [this](const std_msgs::msg::Header & m) {
          const double stamp = Seconds(m.stamp);
          if (!heartbeat_ || stamp > *heartbeat_) {
            heartbeat_ = stamp;  // out-of-order beats are ignored
          }
        }));
    subscriptions_.push_back(create_subscription<rov_interfaces::msg::SwimmerTrackArray>(
        "swimmer_tracks", rclcpp::SensorDataQoS(),
        [this](const rov_interfaces::msg::SwimmerTrackArray & m) {OnSwimmers(m);}));
    subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid & m) {OnMap(m);}));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
        "thrust_saturated", 10, [this](const std_msgs::msg::Bool & m) {
          if (m.data && !saturated_since_) {
            saturated_since_ = now().seconds();
          } else if (!m.data) {
            saturated_since_.reset();
          }
        }));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::BatteryState>(
        "battery_state", 10, [this](const sensor_msgs::msg::BatteryState & m) {
          battery_ = StampedValue{Seconds(m.header.stamp), m.percentage};
        }));
    subscriptions_.push_back(create_subscription<rov_interfaces::msg::LeakAlarm>(
        "leak_alarm", 10, [this](const rov_interfaces::msg::LeakAlarm & m) {
          leak_ = StampedValue{Seconds(m.header.stamp), m.leak ? 1.0 : 0.0};
        }));
    subscriptions_.push_back(create_subscription<rov_interfaces::msg::MissionState>(
        "mission_state", 10,
        [this](const rov_interfaces::msg::MissionState & m) {OnMissionState(m);}));
    subscriptions_.push_back(create_subscription<rov_interfaces::msg::PlannerStatus>(
        "planner_status", 10, [this](const rov_interfaces::msg::PlannerStatus & m) {
          if (m.event == rov_interfaces::msg::PlannerStatus::ARRIVED && m.verified &&
          m.goal_kind == rov_interfaces::msg::PlannerStatus::GOAL_VICTIM)
          {
            arrived_ = true;
          }
        }));
    status_publisher_ = create_publisher<rov_interfaces::msg::SafetyStatus>("safety_status", 10);
    report_publisher_ = create_publisher<std_msgs::msg::String>("safety_report", 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_hz), [this]() {Evaluate();});
    RCLCPP_INFO(
      get_logger(), "safety_monitor_node: %.0f Hz; r_slow %.1f m, r_swim %.1f m, wall %.2f m, "
      "t_odom %.2f s, heartbeat %.1f / %.1f s, t_sat %.1f s, battery %.0f %%, t_mission %.0f s",
      rate_hz, s.swimmer_slow_radius, s.swimmer_clearance, s.wall_distance, s.odometry_timeout,
      s.heartbeat_stop, s.heartbeat_abort, s.saturation_duration, 100.0 * s.battery_minimum,
      s.mission_timeout);
  }

private:
  void OnSwimmers(const rov_interfaces::msg::SwimmerTrackArray & message)
  {
    const double t = now().seconds();
    for (const auto & track : message.tracks) {
      const std::string & frame =
        track.header.frame_id.empty() ? message.header.frame_id : track.header.frame_id;
      const auto & stamp = (track.header.stamp.sec || track.header.stamp.nanosec) ?
        track.header.stamp : message.header.stamp;
      if (frame != map_frame_ || stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
        continue;  // map frame only; malformed stamps dropped
      }
      rov_planning::SwimmerTrack s;
      s.id = track.id;
      s.stamp = Seconds(stamp);
      s.position = Eigen::Vector2d(track.position.x, track.position.y);
      s.velocity = Eigen::Vector2d(track.velocity.x, track.velocity.y);
      swimmers_->Update(s, t);  // rejects stale, future and out-of-order updates
    }
  }

  void OnMap(const nav_msgs::msg::OccupancyGrid & map)
  {
    const auto & info = map.info;
    if (map.header.frame_id != map_frame_ || info.width == 0 || info.height == 0 ||
      map.data.size() != static_cast<std::size_t>(info.width) * info.height)
    {
      grid_.reset();
      return;
    }
    rov_planning::OccupancyGrid2D grid;
    grid.resolution = info.resolution;
    grid.origin = Eigen::Vector2d(info.origin.position.x, info.origin.position.y);
    grid.width = static_cast<int>(info.width);
    grid.height = static_cast<int>(info.height);
    grid.data.assign(map.data.begin(), map.data.end());
    grid_ = std::move(grid);
  }

  void OnMissionState(const rov_interfaces::msg::MissionState & m)
  {
    using rov_interfaces::msg::MissionState;
    if (m.phase == MissionState::STANDBY) {
      mission_start_.reset();
      arrived_ = false;
    } else if (!mission_start_ && m.phase == MissionState::TRANSIT) {
      mission_start_ = now().seconds();
    }
    if (m.phase == MissionState::DIVE_AND_GRASP || m.phase == MissionState::SURFACE_HOLD) {
      arrived_ = true;
    }
    const bool dive = m.phase == MissionState::DIVE_AND_GRASP ||
      (m.phase == MissionState::SAFETY_STOP && m.preempted_phase == MissionState::DIVE_AND_GRASP);
    surface_phase_ = m.phase != MissionState::STANDBY && !dive;
  }

  void Evaluate()
  {
    SafetyInputs in;
    in.now = now().seconds();
    in.odometry_time = odom_time_;
    in.footprint = footprint_;
    in.heartbeat_stamp = heartbeat_;
    in.surface_phase = surface_phase_;
    in.saturated_since = saturated_since_;
    in.battery = battery_;
    in.leak = leak_;
    in.mission_start = mission_start_;
    in.arrived = arrived_;
    swimmers_->Prune(in.now);
    in.swimmers = swimmers_->Tracks();
    try {
      const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      const double yaw = tf2::getYaw(t.transform.rotation);
      in.pose = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y, yaw);
      const auto & v = odom_.twist.twist.linear;  // body frame
      in.velocity = Eigen::Vector2d(
        std::cos(yaw) * v.x - std::sin(yaw) * v.y, std::sin(yaw) * v.x + std::cos(yaw) * v.y);
      if (grid_) {
        const Eigen::Vector2d far =
          in.pose->head<2>() + 1.0e4 * Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
        const rov_planning::ClearanceReport c =
          rov_planning::CheckTerminalClearance(*in.pose, far, terminal_, &*grid_);
        if (std::isfinite(c.wall_distance)) {
          in.wall_distance = c.wall_distance;
        }
      }
    } catch (const tf2::TransformException &) {
      // no pose: the swimmer and wall rules cannot run (odometry rule still does)
    }
    const SafetyDecision d = EvaluateSafety(settings_, in);

    rov_interfaces::msg::SafetyStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.speed_limited = d.speed_limited;
    status.speed_limit = d.speed_limit;
    status.zero_thrust = d.zero_thrust;
    status.stop_hold = d.stop_hold;
    status.replan_now = d.replan_now;
    status.replan_slow = d.replan_slow;
    status.slow_speed_factor = d.slow_speed_factor;
    using rov_interfaces::msg::SafetyStatus;
    status.abort = d.abort != AbortKind::kNone;
    status.abort_kind = d.abort == AbortKind::kHold ? SafetyStatus::ABORT_HOLD :
      d.abort == AbortKind::kReturn ? SafetyStatus::ABORT_RETURN :
      d.abort == AbortKind::kSurfaceReturn ? SafetyStatus::ABORT_SURFACE_RETURN :
      SafetyStatus::ABORT_NONE;
    status.active_rules = d.rules;
    for (std::size_t i = 0; i < d.reasons.size(); ++i) {
      status.reason += (i ? "; " : "") + d.reasons[i];
    }
    status_publisher_->publish(status);

    // Log rule triggers and clears (edges), with time and reason.
    const std::set<std::string> active(d.rules.begin(), d.rules.end());
    for (std::size_t i = 0; i < d.rules.size(); ++i) {
      if (!active_.count(d.rules[i])) {
        Report(true, "SAFETY TRIGGER " + d.reasons[i]);
      }
    }
    for (const std::string & rule : active_) {
      if (!active.count(rule)) {
        Report(false, "SAFETY CLEAR " + rule);
      }
    }
    active_ = active;
    for (const std::string & stale : d.stale_inputs) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring stale input: %s", stale.c_str());
    }
  }

  void Report(bool trigger, const std::string & text)
  {
    char stamp[48];
    std::snprintf(stamp, sizeof(stamp), "t = %.3f s: ", now().seconds());
    const std::string line = stamp + text;
    if (trigger) {
      RCLCPP_WARN(get_logger(), "%s", line.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", line.c_str());
    }
    std_msgs::msg::String message;
    message.data = line;
    report_publisher_->publish(message);
  }

  SafetySettings settings_;
  std::string map_frame_;
  std::string base_frame_;
  rov_planning::TerminalPoseSettings terminal_;
  rov_planning::Footprint footprint_;
  std::optional<rov_planning::SwimmerTrackStore> swimmers_;
  std::optional<rov_planning::OccupancyGrid2D> grid_;
  nav_msgs::msg::Odometry odom_;
  std::optional<double> odom_time_;
  std::optional<double> heartbeat_;
  std::optional<double> saturated_since_;
  std::optional<StampedValue> battery_;
  std::optional<StampedValue> leak_;
  std::optional<double> mission_start_;
  bool arrived_ = false;
  bool surface_phase_ = false;
  std::set<std::string> active_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<rov_interfaces::msg::SafetyStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rov_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_mission::SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
