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

// mission_manager_node: runs the mission state machine (design 2, step 10)
// on typed ROS events. Every transition is logged with its time and reason
// and published on "mission_state" (rov_interfaces/MissionState, also at
// 1 Hz) and as a line on "mission_report" (std_msgs/String).
//
// Events (all typed; an event stamped more than event_max_age_s ago, or in
// the future, is ignored as stale):
//   shore_alarm   rov_interfaces/VictimTarget  STANDBY -> TRANSIT; the
//                 victim is forwarded to the planner ("victim")
//   shore_victim  rov_interfaces/VictimTarget  victim updates, forwarded
//                 to the planner during the approach
//   planner_status rov_interfaces/PlannerStatus  terminal pose (distance ->
//                 FINAL_APPROACH within d_hold), verified arrival (victim
//                 goal -> rescue phase; pose goal in RETURN -> STANDBY)
//   mission_event rov_interfaces/MissionEvent  RESCUE_COMPLETE (simulated:
//                 victim lifted / secured -> RETURN), MANUAL_TAKEOVER
//   safety_status rov_interfaces/SafetyStatus  stop_hold / zero_thrust ->
//                 SAFETY_STOP; abort hold / return / surface-and-return
// RETURN sends the shore pose (shore_pose) to the planner ("goal_pose").
// SAFETY_STOP resumes the preempted phase only when the safety monitor has
// cleared, odometry is fresh (odometry_fresh_s) and the planner reports a
// valid trajectory: a new plan dispatched after the clear (approach and
// return phases) or an active hold (rescue phases).
// DIVE_AND_GRASP is a stub: the vehicle holds bow-on; real dive and grasp
// are outside this simulation milestone.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/msg/mission_event.hpp"
#include "rov_interfaces/msg/mission_state.hpp"
#include "rov_interfaces/msg/planner_status.hpp"
#include "rov_interfaces/msg/safety_status.hpp"
#include "rov_interfaces/msg/victim_target.hpp"
#include "rov_mission/mission_state_machine.hpp"
#include "rov_planning/terminal_pose.hpp"
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

using rov_interfaces::msg::MissionState;
using rov_interfaces::msg::PlannerStatus;
using rov_interfaces::msg::SafetyStatus;
using rov_interfaces::msg::VictimTarget;

uint8_t PhaseCode(Phase phase)
{
  switch (phase) {
    case Phase::kStandby: return MissionState::STANDBY;
    case Phase::kTransit: return MissionState::TRANSIT;
    case Phase::kFinalApproach: return MissionState::FINAL_APPROACH;
    case Phase::kDiveAndGrasp: return MissionState::DIVE_AND_GRASP;
    case Phase::kSurfaceHold: return MissionState::SURFACE_HOLD;
    case Phase::kReturn: return MissionState::RETURN;
    case Phase::kSafetyStop: return MissionState::SAFETY_STOP;
  }
  return MissionState::STANDBY;
}

}  // namespace

class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode()
  : Node("mission_manager_node")
  {
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    shore_pose_ = declare_parameter("shore_pose", std::vector<double>{-20.0, -7.5, M_PI});
    event_max_age_ = declare_parameter("event_max_age_s", 2.0);
    odometry_fresh_ = declare_parameter("odometry_fresh_s", 0.2);
    std::string terminal_file = declare_parameter("terminal_pose_file", std::string(""));
    if (terminal_file.empty()) {
      terminal_file = ament_index_cpp::get_package_share_directory("rov_planning") +
        "/config/terminal_pose.yaml";
    }
    if (shore_pose_.size() != 3 || !(event_max_age_ > 0.0) || !(odometry_fresh_ > 0.0)) {
      throw std::invalid_argument(
              "shore_pose [x, y, yaw], event_max_age_s > 0, odometry_fresh_s > 0");
    }
    const double d_hold =
      rov_planning::LoadTerminalPoseSettings(terminal_file).heading_hold_distance;
    machine_ = std::make_unique<MissionStateMachine>(
      d_hold, [this](const Transition & t) {OnTransition(t);});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    victim_publisher_ = create_publisher<VictimTarget>("victim", 10);
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    state_publisher_ = create_publisher<MissionState>("mission_state", 10);
    report_publisher_ = create_publisher<std_msgs::msg::String>("mission_report", 10);
    subscriptions_.push_back(create_subscription<VictimTarget>(
        "shore_alarm", 10, [this](const VictimTarget & m) {OnAlarm(m);}));
    subscriptions_.push_back(create_subscription<VictimTarget>(
        "shore_victim", 10, [this](const VictimTarget & m) {OnVictimUpdate(m);}));
    subscriptions_.push_back(create_subscription<PlannerStatus>(
        "planner_status", 50, [this](const PlannerStatus & m) {OnPlanner(m);}));
    subscriptions_.push_back(create_subscription<rov_interfaces::msg::MissionEvent>(
        "mission_event", 10,
        [this](const rov_interfaces::msg::MissionEvent & m) {OnMissionEvent(m);}));
    subscriptions_.push_back(create_subscription<SafetyStatus>(
        "safety_status", 10, [this](const SafetyStatus & m) {OnSafety(m);}));
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry &) {odom_time_ = now();}));
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(100), [this]() {Tick();});
    RCLCPP_INFO(
      get_logger(), "mission_manager_node: d_hold %.2f m, shore pose (%.2f, %.2f, %.2f rad)",
      d_hold, shore_pose_[0], shore_pose_[1], shore_pose_[2]);
  }

private:
  double Now() {return now().seconds();}

  // True (and logs) if the event stamp is too old or in the future.
  bool Stale(const builtin_interfaces::msg::Time & stamp, const char * what)
  {
    const rclcpp::Time t(stamp, get_clock()->get_clock_type());
    if (t.nanoseconds() == 0) {
      return false;  // unstamped: taken as "now"
    }
    const double age = (now() - t).seconds();
    if (age > event_max_age_ || age < -0.5) {
      Report("ignored stale " + std::string(what) + " (" + std::to_string(age) + " s old)");
      return true;
    }
    return false;
  }

  void Report(const std::string & text)
  {
    char stamp[48];
    std::snprintf(stamp, sizeof(stamp), "t = %.3f s: ", Now());
    RCLCPP_INFO(get_logger(), "%s%s", stamp, text.c_str());
    std_msgs::msg::String m;
    m.data = stamp + text;
    report_publisher_->publish(m);
  }

  void OnTransition(const Transition & t)
  {
    const std::string line = FormatTransition(t);
    RCLCPP_WARN(get_logger(), "MISSION %s", line.c_str());
    std_msgs::msg::String m;
    m.data = "MISSION " + line;
    report_publisher_->publish(m);
    last_transition_ = line;
    PublishState();
  }

  void PublishState()
  {
    MissionState s;
    s.header.stamp = now();
    s.phase = PhaseCode(machine_->phase());
    s.preempted_phase = machine_->preempted() ? PhaseCode(*machine_->preempted()) : s.phase;
    s.victim_state = machine_->victim_state() == VictimState::kSurface ?
      VictimTarget::SURFACE : VictimTarget::SUBMERGED;
    s.aborted = machine_->aborted();
    s.last_transition = last_transition_;
    state_publisher_->publish(s);
  }

  void OnAlarm(const VictimTarget & m)
  {
    if (Stale(m.header.stamp, "shore alarm")) {
      return;
    }
    const VictimState state =
      m.victim_state == VictimTarget::SURFACE ? VictimState::kSurface : VictimState::kSubmerged;
    const EventResult r = machine_->OnShoreAlarm(Now(), state);
    if (!r.transitioned) {
      Report(r.reason);
      return;
    }
    terminal_.reset();
    rescue_goal_ = 0;
    victim_ = m;
    victim_publisher_->publish(m);  // the planner starts the approach
  }

  void OnVictimUpdate(const VictimTarget & m)
  {
    if (Stale(m.header.stamp, "victim update")) {
      return;
    }
    const Phase p = machine_->phase();
    const bool stopped = p == Phase::kSafetyStop && machine_->preempted().has_value();
    const Phase base = stopped ? *machine_->preempted() : p;
    if (base == Phase::kTransit || base == Phase::kFinalApproach) {
      victim_ = m;
      victim_publisher_->publish(m);
    }
  }

  void OnPlanner(const PlannerStatus & m)
  {
    last_status_ = m;
    last_status_time_ = Now();
    if ((m.event == PlannerStatus::PLANNED || m.event == PlannerStatus::REPLANNED) &&
      m.goal_kind == PlannerStatus::GOAL_VICTIM)
    {
      terminal_ = Eigen::Vector2d(m.terminal_x, m.terminal_y);
    }
    if (m.event == PlannerStatus::PLANNED || m.event == PlannerStatus::REPLANNED) {
      last_plan_time_ = Now();
    }
    if (m.event != PlannerStatus::ARRIVED) {
      return;
    }
    if (m.goal_kind == PlannerStatus::GOAL_VICTIM) {
      Report(machine_->OnArrival(Now(), m.verified, m.detail.substr(0, 120)).reason);
      if (machine_->phase() == Phase::kDiveAndGrasp) {
        Report("DIVE_AND_GRASP stub: holding bow-on at d_dive; waiting for the (simulated) "
          "victim-lifted event");
      }
    } else if (m.goal_kind == PlannerStatus::GOAL_POSE && machine_->phase() == Phase::kReturn) {
      if (m.verified) {
        Report(machine_->OnShoreReached(Now()).reason);
      } else {
        Report("shore pose not verified: " + m.detail.substr(0, 120));
      }
    }
  }

  void SendShoreGoal()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = now();
    goal.pose.position.x = shore_pose_[0];
    goal.pose.position.y = shore_pose_[1];
    goal.pose.orientation.z = std::sin(shore_pose_[2] / 2.0);
    goal.pose.orientation.w = std::cos(shore_pose_[2] / 2.0);
    goal_publisher_->publish(goal);
    Report("RETURN: shore pose sent to the planner");
  }

  void OnMissionEvent(const rov_interfaces::msg::MissionEvent & m)
  {
    if (Stale(m.header.stamp, "mission event")) {
      return;
    }
    if (m.type == rov_interfaces::msg::MissionEvent::RESCUE_COMPLETE) {
      const EventResult r = machine_->OnRescueComplete(Now(), m.detail);
      Report(r.reason);
      if (r.transitioned) {
        SendShoreGoal();
      }
    } else if (m.type == rov_interfaces::msg::MissionEvent::MANUAL_TAKEOVER) {
      Report(machine_->OnManualTakeover(Now()).reason);
    }
  }

  void OnSafety(const SafetyStatus & m)
  {
    if (Stale(m.header.stamp, "safety status")) {
      return;
    }
    safety_ = m;
    if (m.abort && m.abort_kind != SafetyStatus::ABORT_NONE) {
      const AbortKind kind = m.abort_kind == SafetyStatus::ABORT_HOLD ? AbortKind::kHold :
        m.abort_kind == SafetyStatus::ABORT_RETURN ? AbortKind::kReturn :
        AbortKind::kSurfaceReturn;
      const EventResult r = machine_->OnAbort(Now(), kind, m.reason);
      if (r.transitioned && machine_->phase() == Phase::kReturn) {
        if (kind == AbortKind::kSurfaceReturn) {
          Report("surface first: the planar simulation is always at the surface");
        }
        SendShoreGoal();
      }
      if (machine_->aborted() || machine_->phase() == Phase::kReturn) {
        return;
      }
    }
    if ((m.stop_hold || m.zero_thrust) && IsMotionPhase(machine_->phase())) {
      const EventResult r = machine_->OnSafetyStop(Now(), m.reason);
      if (r.transitioned) {
        stop_time_ = Now();
      }
    }
  }

  void Tick()
  {
    const double t = Now();
    if (++ticks_ % 10 == 0) {
      PublishState();
    }
    const Phase phase = machine_->phase();
    // TRANSIT -> FINAL_APPROACH: distance from base_link to the terminal point.
    if (phase == Phase::kTransit && terminal_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        const double d = (Eigen::Vector2d(tf.transform.translation.x, tf.transform.translation.y) -
          *terminal_).norm();
        machine_->OnDistanceToTerminal(t, d);
      } catch (const tf2::TransformException &) {
      }
    }
    // SAFETY_STOP -> resume: monitor clear, fresh odometry, valid trajectory.
    if (phase == Phase::kSafetyStop && !machine_->aborted() && safety_) {
      const bool clear = !safety_->stop_hold && !safety_->zero_thrust && !safety_->abort;
      if (!clear) {
        clear_since_.reset();
        return;
      }
      if (!clear_since_) {
        clear_since_ = t;
        Report("safety monitor clear: waiting for fresh state and a valid trajectory");
      }
      const bool fresh = odom_time_.nanoseconds() != 0 &&
        (now() - odom_time_).seconds() <= odometry_fresh_;
      const Phase resume = machine_->preempted().value_or(Phase::kTransit);
      const bool holding_phase = resume == Phase::kDiveAndGrasp || resume == Phase::kSurfaceHold;
      const bool valid = last_status_ && last_status_->trajectory_active &&
        (holding_phase || (last_plan_time_ && *last_plan_time_ >= *clear_since_));
      const EventResult r = machine_->OnSafetyCleared(
        t, "safety monitor clear", fresh, valid);
      if (r.transitioned) {
        clear_since_.reset();
      }
    }
  }

  std::string map_frame_;
  std::string base_frame_;
  std::vector<double> shore_pose_;
  double event_max_age_;
  double odometry_fresh_;
  std::unique_ptr<MissionStateMachine> machine_;
  std::string last_transition_;
  std::optional<VictimTarget> victim_;
  std::optional<Eigen::Vector2d> terminal_;
  std::optional<PlannerStatus> last_status_;
  double last_status_time_ = 0.0;
  std::optional<double> last_plan_time_;
  std::optional<SafetyStatus> safety_;
  std::optional<double> stop_time_;
  std::optional<double> clear_since_;
  uint64_t rescue_goal_ = 0;
  rclcpp::Time odom_time_{0, 0, RCL_ROS_TIME};
  int ticks_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<VictimTarget>::SharedPtr victim_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<MissionState>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_publisher_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rov_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_mission::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
