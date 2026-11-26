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

// trajectory_generator_node: online surface-approach planning (design 3,
// implementation steps 8B and 9).
//
// Goals (one at a time; a new goal replaces the current one):
//   - victim goal: rov_interfaces/VictimTarget on "victim" (state in the
//     message), or geometry_msgs/PointStamped on "victim_target" (state from
//     the victim_state parameter). Later messages update the victim of the
//     same goal; a different victim state starts a new goal. Terminal pose
//     from the step 8A generator (heading hold, one generator per goal).
//   - pose goal: geometry_msgs/PoseStamped on "goal_pose" (RETURN to the
//     shore): the pose itself is the terminal pose.
// Messages must be in the map frame; a victim update older than the last
// one, or older than target_max_age_s, is ignored (stale).
//
// Planning pipeline, one attempt at a time:
//   terminal pose + clearance gate -> Nav2 A* (ComputePathToPose) ->
//   FIRI corridor + MINCO on a background worker (PlanningWorker) ->
//   validation -> FollowTrajectory goal. A goal is only "active" once the
//   controller has ACCEPTED it; until then the previous one stays active, and
//   a rejected goal leaves it untouched.
// Attempts are started by:
//   - a new goal (initial: from the current pose and velocity);
//   - a timer at replan_rate_hz (5 Hz, design 3 item 6) while a planned
//     trajectory executes, warm-started from the active trajectory;
//     (none within replan_min_remaining_s of the end, nor once a surface
//     victim's final braking into the 0.3 m/s zone has begun);
//   - immediately: target shift > replan_target_shift_m, tracking error >
//     tracking_error_replan_m (controller feedback), a predicted swimmer
//     conflict on the active trajectory, a lower safety speed limit,
//     sustained-saturation "replan slow", a cleared safety hold, a retry.
// A replan continues the active reference: it starts from the active state
// at the handoff time now + replan_handoff_lead_s, and the goal sent is the
// active reference from now to the handoff followed by the replacement
// (tracking-error replans start from the extrapolated odometry state
// instead). Before dispatch every attempt must: have succeeded (every
// optimizer check incl. the deadline), use the current map snapshot, still
// serve the current goal and continue the same active trajectory, be ready
// replan_dispatch_margin_s before the handoff, respect the current speed cap,
// and pass the timestamp-aligned swimmer check over the WHOLE outgoing goal
// (retained prefix included). A swimmer conflict blocks the dispatch
// ("BLOCKED"); it is retried at the replan rate. With swimmers present (or
// seen in the last 2 s), an A* failure is also "BLOCKED" (the predicted
// discs fill the costmap), not a planning failure. A replan that is ready
// too late for its handoff is discarded (the active trajectory stays); only
// max_consecutive_failures late replans in a row count as a failure. No
// attempt starts while braking: the next one starts from rest once the
// vehicle holds.
//
// Speed: v_cruise is capped by the safety monitor's speed limit, by
// "replan slow" (slow_speed_factor * v_cruise) and, after a failure, by
// retry_speed_factor * v_cruise (design 3 item 7). Surface victims get the
// 0.3 m/s final-zone cap within 2 m (design 3 item 5) in the optimizer.
//
// Failure handling (design 3 item 7): a failed attempt keeps the active
// trajectory if it is still valid (same map, swimmer-clear, its terminal
// pose still passes clearance and faces the latest victim) and retries at
// retry_speed_factor * v_cruise; after max_consecutive_failures in a row the
// vehicle brakes to a stop along its path (a_b) and holds, retrying every
// retry_period_s. A goal that never got a trajectory is REJECTED after the
// same number of failures; gate failures (frame, terminal-pose clearance)
// reject at once.
//
// Brake and hold: braking follows the current reference's path with the
// braking deceleration a_b (BrakeAlong) and ends at rest; the controller then
// holds the final pose (hold_after_success). Braking is used after repeated
// failures, for a predicted swimmer conflict on the active trajectory (then
// replan immediately; the resumed trajectory must pass the swimmer check),
// for the safety monitor's stop / hold, and when the active trajectory is no
// longer valid. The brake goal is swimmer-checked too; it is sent even if it
// conflicts (stopping is the least motion available) and the report says so.
//
// Arrival: when the controller succeeds, the reached pose is verified against
// the LATEST target: for a victim goal, the terminal pose for the latest
// victim (heading hold applied) and the clearance check against the latest
// victim; if the victim moved more than replan_target_shift_m since the plan,
// the vehicle re-approaches instead of reporting. Acceptance limits 0.25 m /
// 0.15 rad (simulation). After arrival the controller holds the pose.
//
// Safety monitor ("safety_status"): stop_hold, zero_thrust and abort-hold
// brake and hold, and nothing is dispatched until they clear; then the goal
// resumes with a fresh plan from the current state.
//
// Dry run (dry_run): plan and check as usual, never send; the planned
// trajectory is replayed against later swimmer updates and a new conflict is
// reported as "REPLAN WOULD BE TRIGGERED".
//
// Outputs: "planner_report" (std_msgs/String, one line per event),
// "planner_status" (rov_interfaces/PlannerStatus, typed), and for RViz
// "planned_path" (A*), "trajectory" (dispatched reference), "corridor"
// (visualization_msgs/MarkerArray) and "terminal_pose".
//
// Test hooks: hold_planning_worker pauses the worker before a job;
// force_optimizer_failures (int, settable at run time) makes that many
// attempts fail as optimizer failures.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rov_control/planar_model.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_interfaces/action/follow_trajectory.hpp"
#include "rov_interfaces/msg/planner_status.hpp"
#include "rov_interfaces/msg/safety_status.hpp"
#include "rov_interfaces/msg/swimmer_track_array.hpp"
#include "rov_interfaces/msg/victim_target.hpp"
#include "rov_planning/corridor_2d.hpp"
#include "rov_planning/planning_worker.hpp"
#include "rov_planning/reference_trajectory.hpp"
#include "rov_planning/surface_traj_opt.hpp"
#include "rov_planning/swimmer_prediction.hpp"
#include "rov_planning/swimmer_tracks_ros.hpp"
#include "rov_planning/terminal_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rov_planning
{

namespace
{

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using FollowTrajectory = rov_interfaces::action::FollowTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowTrajectory>;
using PlannerStatus = rov_interfaces::msg::PlannerStatus;
using Clock = std::chrono::steady_clock;

double Milliseconds(Clock::time_point since)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - since).count();
}

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// A number with `digits` decimals.
std::string Num(double value, int digits = 2)
{
  char text[48];
  std::snprintf(text, sizeof(text), "%.*f", digits, value);
  return text;
}

// Default file in an installed package's config folder if `path` is empty.
std::string ConfigFile(
  const std::string & path, const std::string & package, const std::string & file)
{
  return path.empty() ?
         ament_index_cpp::get_package_share_directory(package) + "/config/" + file : path;
}

// nav_msgs/OccupancyGrid -> OccupancyGrid2D. Empty optional if the map is
// not usable (rotated origin, empty, or inconsistent).
std::optional<OccupancyGrid2D> ToGrid(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & info = map.info;
  const double yaw = tf2::getYaw(info.origin.orientation);
  if (info.width == 0 || info.height == 0 || !(info.resolution > 0.0f) ||
    map.data.size() != static_cast<std::size_t>(info.width) * info.height ||
    std::abs(yaw) > 1e-6)
  {
    return std::nullopt;
  }
  OccupancyGrid2D grid;
  grid.resolution = info.resolution;
  grid.origin = Eigen::Vector2d(info.origin.position.x, info.origin.position.y);
  grid.width = static_cast<int>(info.width);
  grid.height = static_cast<int>(info.height);
  grid.data.assign(map.data.begin(), map.data.end());
  return grid;
}

geometry_msgs::msg::Quaternion YawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

enum class GoalKind { kNone, kVictim, kPose };

// What the controller is doing with the active reference.
enum class Mode { kIdle, kExecuting, kBraking, kHolding };

enum class Trigger
{
  kInitial, kPeriodic, kTargetMoved, kTrackingError, kSwimmer, kSpeedLimit, kSlow, kRetry,
  kResume, kReapproach,
};

const char * TriggerName(Trigger trigger)
{
  switch (trigger) {
    case Trigger::kInitial: return "initial";
    case Trigger::kPeriodic: return "periodic";
    case Trigger::kTargetMoved: return "target moved";
    case Trigger::kTrackingError: return "tracking error";
    case Trigger::kSwimmer: return "swimmer conflict";
    case Trigger::kSpeedLimit: return "speed limit";
    case Trigger::kSlow: return "sustained saturation (slow)";
    case Trigger::kRetry: return "retry";
    case Trigger::kResume: return "resume after safety hold";
    case Trigger::kReapproach: return "re-approach";
  }
  return "unknown";
}

}  // namespace

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  TrajectoryGeneratorNode()
  : Node("trajectory_generator_node")
  {
    // Parameters (config/trajectory_generator.yaml).
    declare_parameter("victim_state", std::string("submerged"));
    declare_parameter("dry_run", false);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    planner_id_ = declare_parameter("planner_id", std::string("GridBased"));
    planner_timeout_sec_ = declare_parameter("planner_timeout_sec", 5.0);
    sample_period_ = declare_parameter("sample_period_s", 0.05);
    corridor_settings_.clearance = declare_parameter("corridor_clearance_m", 1.2);
    corridor_settings_.seed_spacing = declare_parameter("corridor_seed_spacing_m", 1.0);
    corridor_settings_.max_polygons =
      static_cast<int>(declare_parameter("corridor_max_polygons", 30));
    corridor_settings_.bounding_margin = declare_parameter("corridor_bounding_margin_m", 3.0);
    acceptance_position_ = declare_parameter("acceptance_position_m", 0.25);
    acceptance_yaw_ = declare_parameter("acceptance_yaw_rad", 0.15);
    replan_enabled_ = declare_parameter("replan_enabled", true);
    replan_rate_hz_ = declare_parameter("replan_rate_hz", 5.0);
    replan_target_shift_ = declare_parameter("replan_target_shift_m", 0.3);
    tracking_error_replan_ = declare_parameter("tracking_error_replan_m", 0.5);
    replan_handoff_lead_ = declare_parameter("replan_handoff_lead_s", 0.6);
    replan_dispatch_margin_ = declare_parameter("replan_dispatch_margin_s", 0.1);
    replan_min_remaining_ = declare_parameter("replan_min_remaining_s", 1.0);
    max_failures_ = static_cast<int>(declare_parameter("max_consecutive_failures", 3));
    retry_speed_factor_ = declare_parameter("retry_speed_factor", 0.7);
    retry_period_ = declare_parameter("retry_period_s", 1.0);
    target_max_age_ = declare_parameter("target_max_age_s", 1.0);
    swimmer_check_enabled_ = declare_parameter("swimmer_check_enabled", true);
    const std::string terminal_file = ConfigFile(
      declare_parameter("terminal_pose_file", std::string("")), "rov_planning",
      "terminal_pose.yaml");
    const std::string optimizer_file = ConfigFile(
      declare_parameter("optimizer_file", std::string("")), "rov_planning",
      "surface_trajectory_optimizer.yaml");
    const std::string allocator_file = ConfigFile(
      declare_parameter("allocator_file", std::string("")), "rov_control",
      "thrust_allocator.yaml");
    const std::string hydrodynamics_file = ConfigFile(
      declare_parameter("hydrodynamics_file", std::string("")), "rov_description",
      "hydrodynamics_v4.yaml");
    const std::string thruster_file = ConfigFile(
      declare_parameter("thruster_config_file", std::string("")), "rov_description",
      "thrusters_v4.yaml");
    const std::string swimmer_file = ConfigFile(
      declare_parameter("swimmer_prediction_file", std::string("")), "rov_planning",
      "swimmer_prediction.yaml");
    if (!(sample_period_ > 0.0) || !(planner_timeout_sec_ > 0.0) || !(replan_rate_hz_ > 0.0) ||
      max_failures_ < 1 || !(retry_speed_factor_ > 0.0) || retry_speed_factor_ > 1.0 ||
      !(retry_period_ > 0.0) || !(target_max_age_ > 0.0))
    {
      throw std::invalid_argument("invalid trajectory_generator_node parameters");
    }

    // Planning core, all from the YAML files.
    terminal_settings_ = LoadTerminalPoseSettings(terminal_file);
    optimizer_settings_ = LoadOptimizerSettings(optimizer_file);
    CheckPlanningLimits(optimizer_settings_, allocator_file);
    swimmers_.emplace(LoadSwimmerPredictionSettings(swimmer_file));
    footprint_.front = terminal_settings_.bow_offset + terminal_settings_.stowed_arm_protrusion;
    footprint_.rear = terminal_settings_.stern_offset;
    footprint_.half_width = terminal_settings_.hull_half_width;
    worker_ = std::make_unique<PlanningWorker>(std::make_unique<SurfaceTrajectoryOptimizer>(
        rov_control::PlanarModel(rov_control::LoadHydrodynamicParams(hydrodynamics_file)),
        rov_control::BuildAllocationMatrix(rov_control::LoadThrusterLayout(thruster_file)),
        optimizer_settings_));
    worker_->SetHold(declare_parameter("hold_planning_worker", false));
    forced_failures_ = static_cast<int>(declare_parameter("force_optimizer_failures", 0));
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const rclcpp::Parameter & parameter : parameters) {
          if (parameter.get_name() == "hold_planning_worker") {
            worker_->SetHold(parameter.as_bool());
          } else if (parameter.get_name() == "force_optimizer_failures") {
            forced_failures_ = static_cast<int>(parameter.as_int());
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid & map) {OnMap(map);});
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](const nav_msgs::msg::Odometry & odom) {odom_ = odom;});
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "victim_target", 10, [this](const geometry_msgs::msg::PointStamped & target) {
        const std::string state = get_parameter("victim_state").as_string();
        if (state != "submerged" && state != "surface") {
          Event(
            PlannerStatus::REJECTED, false,
            "REJECTED: victim_state must be 'submerged' or 'surface'");
          return;
        }
        OnVictim(
          Eigen::Vector2d(target.point.x, target.point.y),
          state == "surface" ? VictimState::kSurface : VictimState::kSubmerged,
          rclcpp::Time(target.header.stamp, get_clock()->get_clock_type()),
          target.header.frame_id);
      });
    victim_subscription_ = create_subscription<rov_interfaces::msg::VictimTarget>(
      "victim", 10, [this](const rov_interfaces::msg::VictimTarget & target) {
        OnVictim(
          Eigen::Vector2d(target.position.x, target.position.y),
          target.victim_state == rov_interfaces::msg::VictimTarget::SURFACE ?
          VictimState::kSurface : VictimState::kSubmerged,
          rclcpp::Time(target.header.stamp, get_clock()->get_clock_type()),
          target.header.frame_id);
      });
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped & pose) {OnPoseGoal(pose);});
    safety_subscription_ = create_subscription<rov_interfaces::msg::SafetyStatus>(
      "safety_status", 10,
      [this](const rov_interfaces::msg::SafetyStatus & status) {OnSafety(status);});
    swimmer_subscription_ = create_subscription<rov_interfaces::msg::SwimmerTrackArray>(
      "swimmer_tracks", rclcpp::SensorDataQoS(),
      [this](const rov_interfaces::msg::SwimmerTrackArray & message) {OnSwimmerTracks(message);});

    report_publisher_ = create_publisher<std_msgs::msg::String>("planner_report", 10);
    status_publisher_ = create_publisher<PlannerStatus>("planner_status", 10);
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("planned_path", latched);
    trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", latched);
    corridor_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("corridor", latched);
    terminal_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("terminal_pose", latched);
    planner_client_ =
      rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
    follow_client_ = rclcpp_action::create_client<FollowTrajectory>(this, "follow_trajectory");

    worker_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {PollWorker();});
    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / replan_rate_hz_), [this]() {Tick();});

    RCLCPP_INFO(
      get_logger(), "trajectory_generator_node ready: L_arm %.2f m (simulation value), "
      "replanning at %.1f Hz, corridor clearance %.2f m, planning thrust +%.0f / -%.0f N",
      terminal_settings_.stowed_arm_protrusion, replan_rate_hz_, corridor_settings_.clearance,
      optimizer_settings_.planning_forward_force, optimizer_settings_.planning_reverse_force);
  }

  ~TrajectoryGeneratorNode() override
  {
    worker_->Stop();  // join the worker before the rest of the node goes away
  }

private:
  // ---- State -------------------------------------------------------------

  struct Goal
  {
    GoalKind kind = GoalKind::kNone;
    uint64_t id = 0;
    VictimState state = VictimState::kSubmerged;
    Eigen::Vector2d victim = Eigen::Vector2d::Zero();  // latest victim position
    rclcpp::Time victim_stamp{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d pose = Eigen::Vector3d::Zero();    // pose goal
    bool dispatched = false;   // a trajectory for this goal was accepted
    bool arrived = false;      // verified arrival reported
    bool given_up = false;     // rejected: no more attempts
  };

  // A reference the controller executes (accepted) or is about to.
  struct Reference
  {
    uint64_t request = 0;
    GoalHandle::SharedPtr handle;
    FollowTrajectory::Goal message;
    RefTrajectory samples;                     // t from `start`
    rclcpp::Time start{0, 0, RCL_ROS_TIME};    // ROS time of t = 0
    std::optional<PolynomialTrajectory> poly;  // planned part, its t = 0 at poly_offset
    double poly_offset = 0.0;
    bool brake = false;                        // brake / hold reference
    TerminalPose terminal;                     // goal pose it ends at
    Eigen::Vector2d victim = Eigen::Vector2d::Zero();  // victim it was planned for
    std::shared_ptr<const OccupancyGrid2D> grid;
    double max_speed = 0.0;
  };

  struct Attempt
  {
    uint64_t id = 0;
    Trigger trigger = Trigger::kInitial;
    bool solving = false;                 // false: waiting for A*
    uint64_t goal_id = 0;
    TerminalPose terminal;
    Eigen::Vector2d victim = Eigen::Vector2d::Zero();
    bool prefix = false;                  // keep the active reference until the handoff
    bool continuous = false;              // must continue the active state at the handoff
    uint64_t base_request = 0;            // the active request it continues
    double handoff = 0.0;                 // s on the active reference
    StartState start;
    std::shared_ptr<const OccupancyGrid2D> grid;
    SpeedOptions speed;
    Clock::time_point began;
    Clock::time_point astar_began;
    double astar_ms = 0.0;
    std::vector<Eigen::Vector2d> path;
    rclcpp_action::ClientGoalHandle<ComputePathToPose>::SharedPtr planner_goal;
  };

  // A reference sent to the controller, waiting for its acceptance.
  struct Pending
  {
    Reference reference;
    uint8_t event = PlannerStatus::PLANNED;
    std::string text;          // report once accepted
    bool counts_as_plan = false;
    std::vector<Eigen::Vector2d> path;
    std::vector<Eigen::MatrixX4d> corridor;
  };

  // ---- Reporting ---------------------------------------------------------

  std::string Tag(uint64_t request) const
  {
    return " [goal " + std::to_string(goal_.id) + ", request " + std::to_string(request) +
           ", map version " + std::to_string(map_version_) + "]";
  }

  void Event(
    uint8_t event, bool ok, const std::string & text, uint64_t request = 0,
    const PlannerStatus * extra = nullptr)
  {
    if (ok) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", text.c_str());
    }
    std_msgs::msg::String message;
    message.data = text;
    report_publisher_->publish(message);
    PlannerStatus status = extra ? *extra : PlannerStatus();
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.event = event;
    status.request = request;
    status.goal_id = goal_.id;
    status.goal_kind = goal_.kind == GoalKind::kVictim ? PlannerStatus::GOAL_VICTIM :
      goal_.kind == GoalKind::kPose ? PlannerStatus::GOAL_POSE : PlannerStatus::GOAL_NONE;
    if (!extra) {
      status.terminal_x = terminal_.position.x();
      status.terminal_y = terminal_.position.y();
      status.terminal_yaw = terminal_.yaw;
    }
    status.trajectory_active = active_.has_value() && mode_ != Mode::kIdle;
    status.detail = text;
    status_publisher_->publish(status);
  }

  // ---- Inputs ------------------------------------------------------------

  void OnMap(const nav_msgs::msg::OccupancyGrid & map)
  {
    ++map_version_;
    std::optional<OccupancyGrid2D> grid = ToGrid(map);
    if (grid && map.header.frame_id != map_frame_) {
      grid.reset();
    }
    if (!grid) {
      grid_.reset();
      RCLCPP_ERROR(
        get_logger(), "Map version %lu rejected (empty, rotated, inconsistent or wrong frame)",
        static_cast<unsigned long>(map_version_));
      return;
    }
    grid_ = std::make_shared<const OccupancyGrid2D>(std::move(*grid));
    RCLCPP_INFO(
      get_logger(), "Map version %lu: %d x %d cells at %.3f m",
      static_cast<unsigned long>(map_version_), grid_->width, grid_->height, grid_->resolution);
  }

  std::optional<Eigen::Vector3d> RobotPose()
  {
    try {
      const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      return Eigen::Vector3d(
        t.transform.translation.x, t.transform.translation.y,
        tf2::getYaw(t.transform.rotation));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No robot pose: %s", error.what());
      return std::nullopt;
    }
  }

  // Start a new goal: forget attempts, failures and the dry-run replay.
  void NewGoal(GoalKind kind)
  {
    goal_ = Goal();
    goal_.kind = kind;
    goal_.id = ++next_goal_id_;
    attempt_.reset();
    pending_trigger_.reset();
    failures_ = 0;
    slow_ = false;
    blocked_reported_ = false;
    replay_goal_.reset();
    dry_run_ = get_parameter("dry_run").as_bool();
    dry_run_done_ = false;
  }

  void OnVictim(
    const Eigen::Vector2d & victim, VictimState state, const rclcpp::Time & stamp,
    const std::string & frame)
  {
    if (frame != map_frame_) {
      Event(
        PlannerStatus::REJECTED, false, "REJECTED: victim target must be in frame '" +
        map_frame_ + "', not '" + frame + "'");
      return;
    }
    if (!victim.allFinite()) {
      Event(PlannerStatus::REJECTED, false, "REJECTED: non-finite victim position");
      return;
    }
    const bool same_goal = goal_.kind == GoalKind::kVictim && !goal_.given_up &&
      goal_.state == state;
    // Stale updates: older than the last one, or too old. A zero stamp means
    // "now" (hand-sent targets).
    const bool stamped = stamp.nanoseconds() != 0;
    if (stamped && (now() - stamp).seconds() > target_max_age_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring a stale victim target (%.2f s old)", (now() - stamp).seconds());
      return;
    }
    if (same_goal && stamped && stamp < goal_.victim_stamp) {
      RCLCPP_WARN(get_logger(), "Ignoring an out-of-order victim target");
      return;
    }
    if (!same_goal) {
      NewGoal(GoalKind::kVictim);
      goal_.state = state;
      goal_.victim = victim;
      goal_.victim_stamp = stamped ? stamp : now();
      generator_.emplace(terminal_settings_);  // fresh heading hold for the new goal
      StartAttempt(Trigger::kInitial);
      return;
    }
    goal_.victim = victim;
    goal_.victim_stamp = stamped ? stamp : now();
    const Eigen::Vector2d planned_for = active_ && goal_.dispatched ? active_->victim :
      planned_victim_;
    const double shift = (victim - planned_for).norm();
    if (shift > replan_target_shift_) {
      if (goal_.arrived) {
        goal_.arrived = false;  // the victim moved after arrival: approach again
        RCLCPP_INFO(get_logger(), "Victim moved %.2f m after arrival: re-approaching", shift);
        pending_trigger_ = Trigger::kReapproach;
      } else if (goal_.dispatched && !pending_trigger_) {
        pending_trigger_ = Trigger::kTargetMoved;
      }
      Tick();  // immediate
    }
  }

  void OnPoseGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (pose.header.frame_id != map_frame_) {
      Event(
        PlannerStatus::REJECTED, false, "REJECTED: goal_pose must be in frame '" + map_frame_ +
        "'");
      return;
    }
    NewGoal(GoalKind::kPose);
    goal_.pose = Eigen::Vector3d(
      pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
    if (!goal_.pose.allFinite()) {
      goal_.given_up = true;
      Event(PlannerStatus::REJECTED, false, "REJECTED: non-finite goal pose");
      return;
    }
    StartAttempt(Trigger::kInitial);
  }

  void OnSafety(const rov_interfaces::msg::SafetyStatus & status)
  {
    const bool hold = status.stop_hold || status.zero_thrust ||
      (status.abort && status.abort_kind == rov_interfaces::msg::SafetyStatus::ABORT_HOLD);
    const double old_cap = SpeedCap();
    safety_ = status;
    if (hold && !safety_hold_) {
      safety_hold_ = true;
      attempt_.reset();
      pending_trigger_.reset();
      EnterBrake("safety monitor: " + status.reason);
    } else if (!hold && safety_hold_) {
      safety_hold_ = false;
      if (GoalOpen()) {
        RCLCPP_INFO(get_logger(), "Safety hold cleared: resuming with a fresh plan");
        pending_trigger_ = Trigger::kResume;
      }
    }
    if (status.replan_slow && !slow_) {
      slow_ = true;
      slow_factor_ = status.slow_speed_factor > 0.0 ? status.slow_speed_factor : 0.7;
      if (mode_ == Mode::kExecuting) {
        pending_trigger_ = Trigger::kSlow;
      }
    }
    if (mode_ == Mode::kExecuting && active_ && !active_->brake && !safety_hold_) {
      if (SpeedCap() < old_cap && active_->max_speed > SpeedCap() + 0.02) {
        pending_trigger_ = Trigger::kSpeedLimit;
      } else if (status.replan_now && !pending_trigger_) {
        pending_trigger_ = Trigger::kSwimmer;
      }
    }
  }

  void OnSwimmerTracks(const rov_interfaces::msg::SwimmerTrackArray & message)
  {
    const TrackConversion conversion = TracksFromMessage(message, map_frame_, *tf_buffer_);
    for (const std::string & error : conversion.errors) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Swimmer dropped: %s", error.c_str());
    }
    const double t_now = now().seconds();
    if (!conversion.tracks.empty()) {
      last_swimmer_seen_ = now();
    }
    for (const SwimmerTrack & track : conversion.tracks) {
      const TrackUpdate update = swimmers_->Update(track, t_now);
      if (update != TrackUpdate::kAccepted) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Swimmer track %u update rejected: %s (age %.3f s)",
          track.id, TrackUpdateToString(update), t_now - track.stamp);
      }
    }
    MonitorSwimmers();
    CheckReplay();
  }

  // ---- Timer ---------------------------------------------------------------

  bool GoalOpen() const
  {
    return goal_.kind != GoalKind::kNone && !goal_.given_up && !goal_.arrived && !dry_run_done_;
  }

  void Tick()
  {
    // A goal the controller never answered counts as rejected.
    if (pending_ && std::chrono::duration<double>(Clock::now() - pending_sent_).count() > 2.0) {
      const uint64_t id = pending_->reference.request;
      const bool plan = pending_->counts_as_plan;
      pending_.reset();
      RCLCPP_ERROR(get_logger(), "No response to goal %lu within 2 s: treated as rejected",
        static_cast<unsigned long>(id));
      if (plan) {
        Failure(id, "no response from the controller", false);
      }
    }
    MonitorSwimmers();
    if (safety_hold_ || attempt_ || pending_ || !GoalOpen()) {
      return;
    }
    if (pending_trigger_) {
      StartAttempt(*pending_trigger_);
      return;
    }
    const double since = std::chrono::duration<double>(Clock::now() - last_attempt_).count();
    if (mode_ == Mode::kExecuting && active_ && !active_->brake && replan_enabled_ &&
      since >= 1.0 / replan_rate_hz_ - 1e-3)
    {
      StartAttempt(Trigger::kPeriodic);
    } else if ((mode_ == Mode::kHolding || mode_ == Mode::kIdle) &&  // NOLINT
      since >= (failures_ >= max_failures_ ? retry_period_ : 1.0 / replan_rate_hz_) - 1e-3)
    {
      StartAttempt(Trigger::kRetry);
    }
  }

  // ---- Speed cap -----------------------------------------------------------

  double SpeedCap() const
  {
    double cap = std::numeric_limits<double>::infinity();
    if (safety_.speed_limited && safety_.speed_limit > 0.0) {
      cap = std::min(cap, safety_.speed_limit);
    }
    if (slow_) {
      cap = std::min(cap, slow_factor_ * optimizer_settings_.cruise_speed);
    }
    if (failures_ > 0) {
      cap = std::min(cap, retry_speed_factor_ * optimizer_settings_.cruise_speed);
    }
    return cap;
  }

  // ---- Attempt -------------------------------------------------------------

  // Active state at time t (s) on the active reference: exact on the planned
  // polynomial, interpolated on the samples otherwise.
  RefPoint ActiveStateAt(double t) const
  {
    if (active_->poly && t >= active_->poly_offset) {
      const double tp = std::min(t - active_->poly_offset, active_->poly->TotalDuration());
      Eigen::Vector3d pos, vel, acc;
      active_->poly->Evaluate(tp, &pos, &vel, &acc);
      RefPoint p;
      p.t = t;
      p.position = pos.head<2>();
      p.velocity = vel.head<2>();
      p.acceleration = acc.head<2>();
      p.yaw = pos(2);
      p.yaw_rate = vel(2);
      return p;
    }
    return InterpolateRef(active_->samples, t);
  }

  double ActiveDuration() const {return active_->samples.back().t;}

  void StartAttempt(Trigger trigger)
  {
    pending_trigger_.reset();
    if (!GoalOpen() || safety_hold_ || pending_) {
      return;
    }
    if (worker_->Busy() || attempt_ || mode_ == Mode::kBraking) {
      pending_trigger_ = trigger;  // after the one in flight / once stopped
      return;
    }
    if (!grid_) {
      Failure(0, "no valid map", trigger == Trigger::kInitial);
      return;
    }
    const std::optional<Eigen::Vector3d> robot = RobotPose();
    if (!robot) {
      Failure(0, "no robot pose", false);
      return;
    }
    Attempt a;
    a.id = ++next_request_id_;
    a.trigger = trigger;
    a.goal_id = goal_.id;
    a.grid = grid_;
    a.began = Clock::now();
    last_attempt_ = a.began;

    // Terminal pose of the goal (latest victim, heading hold) and its gate.
    if (goal_.kind == GoalKind::kVictim) {
      try {
        a.terminal = generator_->Update(robot->head<2>(), goal_.victim, goal_.state);
      } catch (const std::exception & error) {
        Reject(std::string("terminal pose: ") + error.what(), a.id);
        return;
      }
      a.victim = goal_.victim;
      const ClearanceReport clearance = CheckTerminalClearance(
        Eigen::Vector3d(a.terminal.position.x(), a.terminal.position.y(), a.terminal.yaw),
        goal_.victim, terminal_settings_, grid_.get());
      if (!clearance.ok) {
        char text[200];
        std::snprintf(
          text, sizeof(text), "terminal pose (%.2f, %.2f, %.2f rad) fails clearance: %s",
          a.terminal.position.x(), a.terminal.position.y(), a.terminal.yaw,
          clearance.reason.c_str());
        if (!goal_.dispatched) {
          Reject(text, a.id);
        } else {
          Failure(a.id, std::string("new ") + text, false);
        }
        return;
      }
    } else {
      a.terminal.position = goal_.pose.head<2>();
      a.terminal.yaw = goal_.pose(2);
      a.victim = goal_.pose.head<2>();
      const ClearanceReport clearance = WallClearance(goal_.pose, grid_.get());
      if (!clearance.ok) {
        Reject("goal pose fails the wall clearance check: " + clearance.reason, a.id);
        return;
      }
    }
    terminal_ = a.terminal;
    planned_victim_ = a.victim;

    // Start state.
    const bool moving = mode_ == Mode::kExecuting && active_;
    const double t_now = moving ? (now() - active_->start).seconds() : 0.0;
    if (moving && trigger != Trigger::kTrackingError) {
      a.handoff = t_now + replan_handoff_lead_;
      if (trigger == Trigger::kPeriodic &&
        a.handoff > ActiveDuration() - replan_min_remaining_)
      {
        return;  // close to the end: let it finish (arrival re-checks the target)
      }
      if (trigger == Trigger::kPeriodic && goal_.kind == GoalKind::kVictim &&
        goal_.state == VictimState::kSurface)
      {
        // Once the final braking into the surface final zone has begun, the
        // active plan rides the speed envelope and a replan from it gains
        // nothing (and lands marginally above the envelope): keep it.
        const RefPoint h = ActiveStateAt(a.handoff);
        const double braking = h.velocity.squaredNorm() /
          (2.0 * optimizer_settings_.braking_deceleration);
        if ((h.position - a.terminal.position).norm() <=
          optimizer_settings_.final_zone_distance + optimizer_settings_.final_zone_buffer +
          braking + 1.5)
        {
          return;
        }
      }
      a.handoff = std::min(a.handoff, ActiveDuration());
      const RefPoint p = ActiveStateAt(a.handoff);
      a.start.position = p.position;
      a.start.velocity = p.velocity;
      a.start.acceleration = p.acceleration;
      a.start.yaw = p.yaw;
      a.start.yaw_rate = p.yaw_rate;
      a.prefix = true;
      a.continuous = true;
      a.base_request = active_->request;
    } else {
      // From odometry; while moving (tracking error), extrapolated to the
      // handoff with constant velocity.
      const auto & twist = odom_.twist.twist;
      const double c = std::cos((*robot)(2));
      const double s = std::sin((*robot)(2));
      const Eigen::Vector2d velocity(
        c * twist.linear.x - s * twist.linear.y, s * twist.linear.x + c * twist.linear.y);
      a.start.position = robot->head<2>();
      a.start.velocity = velocity;
      a.start.yaw = (*robot)(2);
      a.start.yaw_rate = twist.angular.z;
      if (moving) {
        a.handoff = std::min(t_now + replan_handoff_lead_, ActiveDuration());
        a.start.position += velocity * (a.handoff - t_now);
        a.start.yaw += twist.angular.z * (a.handoff - t_now);
        a.prefix = true;
        a.base_request = active_->request;
      } else if (velocity.norm() < 0.05) {
        a.start.velocity.setZero();  // at rest: start exactly at rest
        a.start.yaw_rate = 0.0;
      }
    }
    a.speed.final_zone = goal_.kind == GoalKind::kVictim && goal_.state == VictimState::kSurface;
    a.speed.max_speed = std::min(SpeedCap(), 10.0 * optimizer_settings_.cruise_speed);

    if (!planner_client_->action_server_is_ready()) {
      Failure(a.id, "planner_server not available", false);
      return;
    }
    ComputePathToPose::Goal request;
    request.goal.header.frame_id = map_frame_;
    request.goal.header.stamp = now();
    request.goal.pose.position.x = a.terminal.position.x();
    request.goal.pose.position.y = a.terminal.position.y();
    request.goal.pose.orientation = YawQuaternion(a.terminal.yaw);
    request.planner_id = planner_id_;
    request.use_start = true;
    request.start.header = request.goal.header;
    request.start.pose.position.x = a.start.position.x();
    request.start.pose.position.y = a.start.position.y();
    request.start.pose.orientation = YawQuaternion(a.start.yaw);
    a.astar_began = Clock::now();
    const uint64_t id = a.id;
    attempt_ = std::move(a);
    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, id](const auto & handle) {
        if (!attempt_ || attempt_->id != id || attempt_->solving) {
          if (handle) {
            planner_client_->async_cancel_goal(handle);
          }
          return;
        }
        if (!handle) {
          attempt_.reset();
          Failure(id, "planner_server refused the path request", false);
          return;
        }
        attempt_->planner_goal = handle;
      };
    options.result_callback = [this, id](const auto & result) {OnPath(id, result);};
    planner_client_->async_send_goal(request, options);
    if (planner_timer_) {
      planner_timer_->cancel();
    }
    planner_timer_ = create_wall_timer(
      std::chrono::duration<double>(planner_timeout_sec_), [this, id]() {
        planner_timer_->cancel();
        if (!attempt_ || attempt_->id != id || attempt_->solving) {
          return;
        }
        if (attempt_->planner_goal) {
          planner_client_->async_cancel_goal(attempt_->planner_goal);
        }
        attempt_.reset();
        Failure(id, "no A* path within planner_timeout_sec", false);
      });
  }

  void OnPath(
    uint64_t id, const rclcpp_action::ClientGoalHandle<ComputePathToPose>::WrappedResult & result)
  {
    if (!attempt_ || attempt_->id != id || attempt_->solving) {
      return;  // stale: superseded, timed out, or the goal changed
    }
    if (planner_timer_) {
      planner_timer_->cancel();
    }
    Attempt & a = *attempt_;
    a.astar_ms = Milliseconds(a.astar_began);
    const auto & poses = result.result ? result.result->path.poses :
      std::vector<geometry_msgs::msg::PoseStamped>{};
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || poses.size() < 2) {
      attempt_.reset();
      swimmers_->Prune(now().seconds());
      // Swimmers now, or within the last 2 s (the costmap clears expired
      // marks on its next update, after the track age runs out).
      if (swimmer_check_enabled_ && (!swimmers_->Tracks().empty() ||
        (last_swimmer_seen_.nanoseconds() != 0 && (now() - last_swimmer_seen_).seconds() < 2.0)))
      {
        // The costmap's only moving obstacles are the predicted swimmer discs:
        // wait for them (the timed check on the active trajectory decides
        // whether it may continue), not a planning failure.
        Blocked(id, "A* found no path through the predicted swimmer positions");
        return;
      }
      Failure(id, "A* found no path to the terminal pose", false);
      return;
    }
    for (const auto & pose : poses) {
      a.path.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    a.path.front() = a.start.position;
    a.path.back() = a.terminal.position;

    PlanningJob job;
    job.request = id;
    job.grid = a.grid;
    job.corridor_settings = corridor_settings_;
    job.path = a.path;
    job.start = a.start;
    job.goal = Eigen::Vector3d(a.terminal.position.x(), a.terminal.position.y(), a.terminal.yaw);
    job.speed = a.speed;
    if (a.continuous && active_ && active_->poly && a.handoff >= active_->poly_offset) {
      job.warm = WarmStart{*active_->poly, a.handoff - active_->poly_offset};
    }
    if (forced_failures_ > 0) {
      job.force_failure = true;
      --forced_failures_;
    }
    if (!worker_->Submit(std::move(job))) {
      attempt_.reset();
      Failure(id, "planning worker busy or stopped", false);
      return;
    }
    a.solving = true;
  }

  void PollWorker()
  {
    std::optional<PlanningOutcome> outcome = worker_->TakeResult();
    if (outcome) {
      OnSolved(std::move(*outcome));
    }
    if (!attempt_ && !worker_->Busy() && pending_trigger_ && !pending_ && !safety_hold_) {
      StartAttempt(*pending_trigger_);
    }
  }

  void OnSolved(PlanningOutcome outcome)
  {
    if (!attempt_ || attempt_->id != outcome.request || !attempt_->solving) {
      return;  // stale: the attempt was superseded (brake, new goal)
    }
    Attempt a = std::move(*attempt_);
    attempt_.reset();
    if (a.goal_id != goal_.id || !GoalOpen() || safety_hold_) {
      return;  // the goal changed or ended, or a safety hold started meanwhile
    }
    if (!outcome.ok) {
      Failure(a.id, outcome.error, false);
      return;
    }
    if (!grid_ || grid_ != outcome.grid) {
      Failure(a.id, "the map changed during planning", false);
      return;
    }
    const PlanResult & plan = outcome.plan;
    const PolynomialTrajectory & poly = plan.trajectory;

    // Build the outgoing reference (with the retained prefix).
    const rclcpp::Time dispatch_time = now();
    RefTrajectory samples;
    double offset = 0.0;
    double jump_p = 0.0, jump_v = 0.0, jump_a = 0.0;
    if (a.prefix) {
      if (!active_ || active_->request != a.base_request ||
        (mode_ != Mode::kExecuting && mode_ != Mode::kBraking))
      {
        return;  // the reference it continues is gone (not a planning failure)
      }
      const double t_now = (dispatch_time - active_->start).seconds();
      if (t_now > a.handoff - replan_dispatch_margin_) {
        const std::string late = "late: ready at t = " + Num(t_now) + " s, handoff at " +
          Num(a.handoff) + " s (margin " + Num(replan_dispatch_margin_) + " s)";
        // A timing miss, not an optimization failure: the active trajectory
        // stays; only repeated misses count as a failure.
        if (++late_streak_ >= max_failures_) {
          late_streak_ = 0;
          Failure(a.id, late + ", " + std::to_string(max_failures_) + " times in a row", false);
        } else {
          Event(
            PlannerStatus::REPLAN_FAILED, false, "REPLAN DISCARDED (" + late +
            "): keeping the active trajectory" + Tag(a.id), a.id);
          if (a.trigger != Trigger::kPeriodic) {
            pending_trigger_ = a.trigger;
          }
        }
        return;
      }
      for (double t = t_now; t < a.handoff - 1e-9; t += sample_period_) {
        RefPoint p = ActiveStateAt(t);
        p.t = t - t_now;
        samples.push_back(p);
      }
      offset = a.handoff - t_now;
      if (a.continuous) {
        const RefPoint old_state = ActiveStateAt(a.handoff);
        Eigen::Vector3d pos, vel, acc;
        poly.Evaluate(0.0, &pos, &vel, &acc);
        jump_p = (pos.head<2>() - old_state.position).norm();
        jump_v = (vel.head<2>() - old_state.velocity).norm();
        jump_a = (acc.head<2>() - old_state.acceleration).norm();
        if (jump_p > 1e-6 || jump_v > 1e-6 || jump_a > 1e-6) {
          Failure(
            a.id, "the replacement does not continue the active state at the handoff", false);
          return;
        }
      }
    }
    const RefTrajectory planned =
      SamplePolynomial(poly, 0.0, poly.TotalDuration(), sample_period_, offset);
    samples.insert(samples.end(), planned.begin(), planned.end());
    const FollowTrajectory::Goal message = ToGoal(samples, dispatch_time);

    // Timestamp-aligned swimmer check over the whole outgoing goal.
    if (const std::optional<SwimmerConflict> conflict = SwimmerConflictOf(message)) {
      if (!blocked_reported_) {
        Event(
          PlannerStatus::BLOCKED, false, "BLOCKED: swimmer conflict: " +
          DescribeConflict(*conflict, message) + " (" + TriggerName(a.trigger) + " plan, T " +
          Num(poly.TotalDuration()) + " s); not sent, retrying at the replan rate" +
          (dry_run_ ? " [DRY RUN]" : "") + Tag(a.id), a.id);
        blocked_reported_ = true;
      }
      if (dry_run_) {
        dry_run_done_ = true;
      }
      return;
    }
    blocked_reported_ = false;
    // Speed cap (it may have dropped during planning).
    if (plan.report.max_speed_mps > SpeedCap() + 0.02 &&
      plan.report.max_speed_mps > a.start.velocity.norm() + 0.12)
    {
      Failure(a.id, "the speed limit dropped during planning", false);
      return;
    }

    const double total_ms = Milliseconds(a.began);
    const bool first = !goal_.dispatched;
    const char * what = goal_.kind == GoalKind::kPose ? "pose goal" :
      goal_.state == VictimState::kSurface ? "surface target" : "submerged target";
    const std::string handoff = a.prefix ? "in " + Num(offset) + " s" : "now (from rest)";
    char text[900];
    if (first) {
      std::snprintf(
        text, sizeof(text),
        "PLANNED %s (%.2f, %.2f): terminal pose (%.3f, %.3f, %.3f rad); A* %zu poses, %zu "
        "polygons, T %.2f s, max speed %.3f m/s (cap %s); time: A* %.1f ms, corridor %.1f ms "
        "(%s), MINCO %.1f ms (%s start), optimizer incl. checks %.1f ms, total %.1f ms%s%s",
        what, a.victim.x(), a.victim.y(), a.terminal.position.x(), a.terminal.position.y(),
        a.terminal.yaw, a.path.size(), outcome.polygons, poly.TotalDuration(),
        plan.report.max_speed_mps, CapText(a.speed).c_str(), a.astar_ms, outcome.corridor_ms,
        outcome.cache_hit ? "cached inflation" : "cold inflation", plan.solve_time_ms,
        plan.warm_started ? "warm" : "cold", plan.total_time_ms, total_ms,
        dry_run_ ? " [DRY RUN: not sent]" : "", Tag(a.id).c_str());
    } else {
      std::snprintf(
        text, sizeof(text),
        "REPLANNED (%s): new terminal pose (%.3f, %.3f, %.3f rad), victim (%.2f, %.2f); %s "
        "start; replan total %.1f ms (A* %.1f, corridor %.1f, MINCO %.1f, optimizer incl. "
        "checks %.1f); handoff %s; continuity |dp| %.1e m, |dv| %.1e m/s, |da| %.1e m/s^2; "
        "new T %.2f s, max speed %.3f m/s (cap %s)%s",
        TriggerName(a.trigger), a.terminal.position.x(), a.terminal.position.y(), a.terminal.yaw,
        a.victim.x(), a.victim.y(), plan.warm_started ? "warm" : "cold", total_ms, a.astar_ms,
        outcome.corridor_ms, plan.solve_time_ms, plan.total_time_ms, handoff.c_str(), jump_p,
        jump_v, jump_a, poly.TotalDuration(), plan.report.max_speed_mps,
        CapText(a.speed).c_str(), Tag(a.id).c_str());
    }
    if (dry_run_) {
      Event(PlannerStatus::PLANNED, true, text, a.id);
      replay_goal_ = message;
      replay_request_ = a.id;
      replay_conflict_ = false;
      dry_run_done_ = true;
      PublishDisplay(a.path, outcome.corridor, samples, dispatch_time);
      return;
    }
    if (!follow_client_->action_server_is_ready()) {
      Failure(a.id, "follow_trajectory server not available", false);
      return;
    }
    Pending p;
    p.reference.request = a.id;
    p.reference.message = message;
    p.reference.samples = samples;
    p.reference.start = dispatch_time;
    p.reference.poly = poly;
    p.reference.poly_offset = offset;
    p.reference.terminal = a.terminal;
    p.reference.victim = a.victim;
    p.reference.grid = a.grid;
    p.reference.max_speed = MaxSpeed(samples);
    p.event = first ? PlannerStatus::PLANNED : PlannerStatus::REPLANNED;
    p.text = text;
    p.counts_as_plan = true;
    p.path = a.path;
    p.corridor = outcome.corridor;
    Send(std::move(p));
  }

  std::string CapText(const SpeedOptions & speed) const
  {
    std::string result = "none";
    if (std::isfinite(speed.max_speed) && speed.max_speed < optimizer_settings_.cruise_speed) {
      result = Num(speed.max_speed) + " m/s";
    }
    if (speed.final_zone) {
      result += ", surface final zone";
    }
    return result;
  }

  // ---- Failures --------------------------------------------------------------

  // Gate failure: the goal cannot be planned at all.
  void Reject(const std::string & reason, uint64_t request)
  {
    goal_.given_up = true;
    attempt_.reset();
    pending_trigger_.reset();
    Event(PlannerStatus::REJECTED, false, "REJECTED: " + reason + Tag(request), request);
    if (mode_ == Mode::kExecuting) {
      EnterBrake("goal rejected: " + reason);
    }
  }

  // A retryable failure of one attempt (`gate`: reject at once if the goal
  // has no trajectory yet).
  void Failure(uint64_t request, const std::string & reason, bool gate)
  {
    if (gate && !goal_.dispatched) {
      Reject(reason, request);
      return;
    }
    ++failures_;
    const std::string count =
      " [failure " + std::to_string(failures_) + " of " + std::to_string(max_failures_) + "]";
    const std::string retry = "retrying at " + Num(retry_speed_factor_) + " v_cruise";
    if (!goal_.dispatched && mode_ != Mode::kExecuting) {
      if (failures_ >= max_failures_) {
        Reject(
          std::to_string(failures_) + " consecutive planning failures (last: " + reason + ")",
          request);
        return;
      }
      Event(
        PlannerStatus::REPLAN_FAILED, false, "PLAN FAILED (" + reason + ")" + count + ": " +
        retry + Tag(request), request);
      return;  // the timer retries at the replan rate
    }
    std::string why;
    const bool keep = mode_ == Mode::kExecuting && ActiveStillValid(&why);
    if (failures_ >= max_failures_) {
      Event(
        PlannerStatus::REPLAN_FAILED, false, "REPLAN FAILED (" + reason + ")" + count +
        ": braking to a stop and holding; retrying every " + Num(retry_period_) + " s" +
        Tag(request), request);
      if (mode_ == Mode::kExecuting) {
        EnterBrake(std::to_string(failures_) + " consecutive planning failures");
      }
      return;
    }
    if (mode_ == Mode::kExecuting && !keep) {
      Event(
        PlannerStatus::REPLAN_FAILED, false, "REPLAN FAILED (" + reason + ")" + count +
        ": active trajectory no longer valid (" + why + "): braking; " + retry + Tag(request),
        request);
      EnterBrake("active trajectory no longer valid: " + why);
      pending_trigger_ = Trigger::kRetry;
      return;
    }
    Event(
      PlannerStatus::REPLAN_FAILED, false, "REPLAN FAILED (" + reason + ")" + count + ": " +
      (keep ? "keeping the active trajectory, still valid; " : "") + retry + Tag(request),
      request);
    pending_trigger_ = Trigger::kRetry;
  }

  // Swimmer predictions block planning: not a failure. Keep a still-valid
  // active trajectory, brake if it is not, and retry at the replan rate.
  void Blocked(uint64_t request, const std::string & reason)
  {
    if (!blocked_reported_) {
      Event(
        PlannerStatus::BLOCKED, false, "BLOCKED: " + reason + "; retrying at the replan rate" +
        Tag(request), request);
      blocked_reported_ = true;
    }
    std::string why;
    if (mode_ == Mode::kExecuting && !ActiveStillValid(&why)) {
      EnterBrake("blocked by swimmers and the active trajectory is no longer valid: " + why);
    }
  }

  // Is the active trajectory still acceptable to keep executing?
  bool ActiveStillValid(std::string * why)
  {
    if (!active_ || active_->brake) {
      *why = "no planned trajectory";
      return false;
    }
    if (!grid_ || grid_ != active_->grid) {
      *why = "the map changed since it was planned";
      return false;
    }
    if (SwimmerConflictOf(active_->message)) {
      *why = "predicted swimmer conflict";
      return false;
    }
    if (goal_.kind == GoalKind::kVictim) {
      const TerminalPose & t = active_->terminal;
      const ClearanceReport clearance = CheckTerminalClearance(
        Eigen::Vector3d(t.position.x(), t.position.y(), t.yaw), goal_.victim, terminal_settings_,
        grid_.get());
      if (!clearance.ok) {
        *why = "its terminal pose fails clearance: " + clearance.reason;
        return false;
      }
      const Eigen::Vector2d to_victim = goal_.victim - t.position;
      const double bearing_error =
        std::abs(NormalizeAngle(std::atan2(to_victim.y(), to_victim.x()) - t.yaw));
      if (bearing_error > terminal_settings_.heading_reaim_angle) {
        *why = "the bow would miss the victim by " + Num(bearing_error, 3) + " rad";
        return false;
      }
    }
    return true;
  }

  // ---- Brake and hold -------------------------------------------------------

  void EnterBrake(const std::string & reason)
  {
    if (mode_ != Mode::kExecuting || !active_) {
      // Idle (nothing commanded) or already braking / holding: nothing to stop.
      RCLCPP_INFO(get_logger(), "Stop requested (%s): nothing is moving", reason.c_str());
      return;
    }
    const rclcpp::Time t0 = now();
    const RefTrajectory samples = BrakeAlong(
      active_->samples, (t0 - active_->start).seconds(),
      optimizer_settings_.braking_deceleration, sample_period_);
    const FollowTrajectory::Goal message = ToGoal(samples, t0);
    std::string note;
    if (const auto conflict = SwimmerConflictOf(message)) {
      note = "; the brake path also conflicts with swimmer " + std::to_string(conflict->id) +
        " (braking anyway: least motion available)";
    }
    Pending p;
    p.reference.request = ++next_request_id_;
    p.reference.message = message;
    p.reference.samples = samples;
    p.reference.start = t0;
    p.reference.brake = true;
    p.reference.terminal = terminal_;
    p.reference.victim = planned_victim_;
    p.reference.grid = grid_;
    p.event = PlannerStatus::BRAKING;
    p.text = "BRAKING: " + reason + "; stopping in " + Num(samples.back().t) + " s and " +
      Num((samples.back().position - samples.front().position).norm()) +
      " m along the path at " + Num(optimizer_settings_.braking_deceleration) +
      " m/s^2, then holding" + note + Tag(p.reference.request);
    Send(std::move(p));
  }

  // ---- Dispatch --------------------------------------------------------------

  FollowTrajectory::Goal ToGoal(const RefTrajectory & samples, const rclcpp::Time & start) const
  {
    FollowTrajectory::Goal goal;
    goal.trajectory.header.frame_id = map_frame_;
    goal.trajectory.header.stamp = start;
    for (const RefPoint & p : samples) {
      rov_interfaces::msg::TrajectoryPoint point;
      point.time_from_start = rclcpp::Duration::from_seconds(p.t);
      point.x = p.position.x();
      point.y = p.position.y();
      point.vx = p.velocity.x();
      point.vy = p.velocity.y();
      point.ax = p.acceleration.x();
      point.ay = p.acceleration.y();
      point.yaw = p.yaw;
      point.yaw_rate = p.yaw_rate;
      goal.trajectory.points.push_back(point);
    }
    return goal;
  }

  void Send(Pending p)
  {
    const uint64_t id = p.reference.request;
    if (!follow_client_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "follow_trajectory server not available: goal %lu not sent",
        static_cast<unsigned long>(id));
      if (p.counts_as_plan) {
        Failure(id, "follow_trajectory server not available", false);
      }
      return;
    }
    pending_ = std::move(p);
    pending_sent_ = Clock::now();
    rclcpp_action::Client<FollowTrajectory>::SendGoalOptions options;
    options.goal_response_callback = [this, id](const GoalHandle::SharedPtr & handle) {
        OnGoalResponse(id, handle);
      };
    options.feedback_callback = [this](
      GoalHandle::SharedPtr handle, const std::shared_ptr<const FollowTrajectory::Feedback> fb) {
        OnFeedback(handle, *fb);
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        OnResult(result);
      };
    follow_client_->async_send_goal(pending_->reference.message, options);
  }

  void OnGoalResponse(uint64_t id, const GoalHandle::SharedPtr & handle)
  {
    if (!pending_ || pending_->reference.request != id) {
      return;  // superseded by a later goal, which preempts this one at the controller
    }
    Pending p = std::move(*pending_);
    pending_.reset();
    if (!handle) {
      // Rejected: the previous goal (if any) keeps running untouched.
      if (p.counts_as_plan) {
        Failure(id, "the controller rejected the trajectory", false);
      } else {
        Event(
          PlannerStatus::REPLAN_FAILED, false, "BRAKE NOT SENT: the controller rejected it" +
          Tag(id), id);
      }
      return;
    }
    p.reference.handle = handle;
    const bool hold_only = p.reference.brake && p.reference.samples.size() == 2 &&
      p.reference.samples.front().velocity.isZero();
    active_ = std::move(p.reference);
    mode_ = active_->brake ? (hold_only ? Mode::kHolding : Mode::kBraking) : Mode::kExecuting;
    swimmer_conflict_episode_ = false;
    if (p.counts_as_plan) {
      goal_.dispatched = true;
      failures_ = 0;
      late_streak_ = 0;
      PublishDisplay(p.path, p.corridor, active_->samples, active_->start);
    } else {
      PublishDisplay({}, {}, active_->samples, active_->start);
    }
    PlannerStatus extra;
    extra.terminal_x = active_->terminal.position.x();
    extra.terminal_y = active_->terminal.position.y();
    extra.terminal_yaw = active_->terminal.yaw;
    extra.max_speed = active_->max_speed;
    Event(p.event, true, p.text, id, &extra);
  }

  void OnFeedback(const GoalHandle::SharedPtr & handle, const FollowTrajectory::Feedback & fb)
  {
    if (!active_ || active_->handle != handle || mode_ != Mode::kExecuting) {
      return;
    }
    if (fb.position_error > tracking_error_replan_ && !attempt_ && !pending_trigger_) {
      RCLCPP_WARN(
        get_logger(), "Tracking error %.2f m > %.2f m: replanning from the current state",
        fb.position_error, tracking_error_replan_);
      pending_trigger_ = Trigger::kTrackingError;
    }
  }

  void OnResult(const GoalHandle::WrappedResult & result)
  {
    if (!active_ || !active_->handle || active_->handle->get_goal_id() != result.goal_id) {
      return;  // an older goal (preempted or superseded): nothing to do
    }
    const auto & r = result.result;
    const uint8_t code = r ? r->error_code : FollowTrajectory::Result::TRACKING_ERROR_EXCEEDED;
    if (code == FollowTrajectory::Result::PREEMPTED) {
      return;  // a replacement took over
    }
    const uint64_t request = active_->request;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && r && r->success) {
      mode_ = Mode::kHolding;  // the controller holds the final pose
      if (active_->brake) {
        Event(
          PlannerStatus::HOLDING, true, "HOLDING: stopped, holding position" + Tag(request),
          request);
        return;
      }
      OnArrived(request);
      return;
    }
    // The controller gave up (tracking error, odometry, invalid, canceled).
    mode_ = Mode::kIdle;
    const std::string message = r ? r->message : "no result";
    if (!GoalOpen()) {
      return;
    }
    Failure(request, "controller failed: " + message, false);
  }

  // ---- Arrival ---------------------------------------------------------------

  void OnArrived(uint64_t request)
  {
    const std::optional<Eigen::Vector3d> robot = RobotPose();
    if (!robot || !grid_) {
      goal_.arrived = true;
      Event(
        PlannerStatus::ARRIVED, false, std::string("NOT ACCEPTED: verification failed: ") +
        (robot ? "no valid current map" : "no robot pose") + Tag(request), request);
      return;
    }
    TerminalPose expected = active_->terminal;
    ClearanceReport clearance;
    std::string against;
    if (goal_.kind == GoalKind::kVictim) {
      const double shift = (goal_.victim - active_->victim).norm();
      if (shift > replan_target_shift_ && GoalOpen()) {
        RCLCPP_INFO(
          get_logger(), "Arrived, but the victim has moved %.2f m since the plan: re-approaching",
          shift);
        pending_trigger_ = Trigger::kReapproach;
        return;
      }
      if (shift > 1e-9) {
        try {
          expected = generator_->Update(robot->head<2>(), goal_.victim, goal_.state);
        } catch (const std::exception &) {
          // keep the planned terminal pose
        }
      }
      clearance = CheckTerminalClearance(*robot, goal_.victim, terminal_settings_, grid_.get());
      against = "latest victim (" + Num(goal_.victim.x()) + ", " + Num(goal_.victim.y()) + ")";
    } else {
      expected.position = goal_.pose.head<2>();
      expected.yaw = goal_.pose(2);
      clearance = WallClearance(*robot, grid_.get());
      against = "goal pose";
    }
    const double position_error = (robot->head<2>() - expected.position).norm();
    const double yaw_error = std::abs(NormalizeAngle((*robot)(2) - expected.yaw));
    const bool accepted = position_error <= acceptance_position_ &&
      yaw_error <= acceptance_yaw_ && clearance.ok;
    goal_.arrived = true;
    const std::string wall = std::isfinite(clearance.wall_distance) ?
      Num(clearance.wall_distance, 3) + " m" :
      "none within " + Num(terminal_settings_.wall_clearance + grid_->resolution) + " m";
    const std::string gap = goal_.kind == GoalKind::kVictim ?
      Num(clearance.victim_gap, 3) + " m" : "n/a";
    char text[700];
    std::snprintf(
      text, sizeof(text),
      "%s: controller succeeded; reached (%.3f, %.3f, %.3f rad), error %.3f m / %.3f rad "
      "against the %s terminal pose (%.3f, %.3f, %.3f rad) (acceptance %.2f m / %.2f rad); "
      "clearance %s: victim gap %s, wall %s%s%s; holding position%s",
      accepted ? "ACCEPTED" : "NOT ACCEPTED", robot->x(), robot->y(), (*robot)(2),
      position_error, yaw_error, against.c_str(), expected.position.x(), expected.position.y(),
      expected.yaw, acceptance_position_, acceptance_yaw_, clearance.ok ? "ok" : "FAILED",
      gap.c_str(), wall.c_str(), clearance.ok ? "" : " - ", clearance.reason.c_str(),
      Tag(request).c_str());
    PlannerStatus extra;
    extra.terminal_x = expected.position.x();
    extra.terminal_y = expected.position.y();
    extra.terminal_yaw = expected.yaw;
    extra.verified = accepted;
    extra.position_error = position_error;
    extra.yaw_error = yaw_error;
    Event(PlannerStatus::ARRIVED, accepted, text, request, &extra);
  }

  // Wall clearance of the stowed outline at `pose` (no victim): the victim
  // is put far ahead of the bow so only the wall and map checks matter.
  ClearanceReport WallClearance(const Eigen::Vector3d & pose, const OccupancyGrid2D * grid) const
  {
    const Eigen::Vector2d far_ahead =
      pose.head<2>() + 1.0e4 * Eigen::Vector2d(std::cos(pose(2)), std::sin(pose(2)));
    return CheckTerminalClearance(pose, far_ahead, terminal_settings_, grid);
  }

  // ---- Swimmers ----------------------------------------------------------------

  std::optional<SwimmerConflict> SwimmerConflictOf(const FollowTrajectory::Goal & goal)
  {
    if (!swimmer_check_enabled_) {
      return std::nullopt;
    }
    const double t_now = now().seconds();
    swimmers_->Prune(t_now);
    const double start = rclcpp::Time(goal.trajectory.header.stamp).seconds();
    std::vector<TimedPose> samples;
    samples.reserve(goal.trajectory.points.size());
    for (const auto & point : goal.trajectory.points) {
      samples.push_back(
        {start + rclcpp::Duration(point.time_from_start).seconds(),
          Eigen::Vector3d(point.x, point.y, point.yaw)});
    }
    return FindSwimmerConflict(
      samples, footprint_, swimmers_->Tracks(), t_now, swimmers_->settings());
  }

  std::string DescribeConflict(
    const SwimmerConflict & conflict, const FollowTrajectory::Goal & goal)
  {
    const double start = rclcpp::Time(goal.trajectory.header.stamp).seconds();
    const double t_now = now().seconds();
    char text[400];
    std::snprintf(
      text, sizeof(text),
      "track %u: in %.2f s (t = %.2f s on the trajectory) the hull at (%.2f, %.2f) would be "
      "%.2f m from the swimmer's predicted position (%.2f, %.2f), inside its %.2f m disc "
      "(r_swim %.2f m + growth over %.2f s since the measurement)",
      conflict.id, conflict.t - t_now, conflict.t - start, conflict.robot.x(),
      conflict.robot.y(), conflict.gap, conflict.swimmer.x(), conflict.swimmer.y(),
      conflict.radius, swimmers_->settings().clearance, conflict.age);
    return text;
  }

  // Predicted conflict on the active (planned) trajectory: brake, hold and
  // replan immediately (design 5).
  void MonitorSwimmers()
  {
    if (dry_run_ || mode_ != Mode::kExecuting || !active_ || active_->brake || pending_) {
      return;
    }
    const std::optional<SwimmerConflict> conflict = SwimmerConflictOf(active_->message);
    if (!conflict) {
      swimmer_conflict_episode_ = false;
      return;
    }
    if (swimmer_conflict_episode_) {
      return;
    }
    swimmer_conflict_episode_ = true;
    Event(
      PlannerStatus::SWIMMER_CONFLICT, false, "SWIMMER CONFLICT on the active trajectory: " +
      DescribeConflict(*conflict, active_->message) + "; braking, holding and replanning" +
      Tag(active_->request), active_->request);
    attempt_.reset();
    EnterBrake("predicted swimmer conflict (track " + std::to_string(conflict->id) + ")");
    pending_trigger_ = Trigger::kSwimmer;
  }

  // Dry run: check the replayed trajectory's remaining part.
  void CheckReplay()
  {
    if (!replay_goal_) {
      return;
    }
    const double start = rclcpp::Time(replay_goal_->trajectory.header.stamp).seconds();
    const double end =
      start + rclcpp::Duration(replay_goal_->trajectory.points.back().time_from_start).seconds();
    if (now().seconds() > end) {
      Event(
        PlannerStatus::HOLDING, true, "DRY-RUN REPLAY ENDED: trajectory finished" +
        Tag(replay_request_), replay_request_);
      replay_goal_.reset();
      return;
    }
    const std::optional<SwimmerConflict> conflict = SwimmerConflictOf(*replay_goal_);
    if (conflict && !replay_conflict_) {
      Event(
        PlannerStatus::SWIMMER_CONFLICT, false, "REPLAN WOULD BE TRIGGERED (dry run): swimmer "
        "conflict on the replayed trajectory: " + DescribeConflict(*conflict, *replay_goal_) +
        Tag(replay_request_), replay_request_);
    } else if (!conflict && replay_conflict_) {
      RCLCPP_INFO(get_logger(), "Dry-run replay: swimmer conflict cleared");
    }
    replay_conflict_ = conflict.has_value();
  }

  // ---- Display (RViz) ------------------------------------------------------------

  void PublishDisplay(
    const std::vector<Eigen::Vector2d> & path, const std::vector<Eigen::MatrixX4d> & corridor,
    const RefTrajectory & samples, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = map_frame_;
    trajectory.header.stamp = stamp;
    for (const RefPoint & p : samples) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = trajectory.header;
      pose.pose.position.x = p.position.x();
      pose.pose.position.y = p.position.y();
      pose.pose.orientation = YawQuaternion(p.yaw);
      trajectory.poses.push_back(pose);
    }
    trajectory_publisher_->publish(trajectory);
    geometry_msgs::msg::PoseStamped terminal;
    terminal.header = trajectory.header;
    terminal.pose.position.x = terminal_.position.x();
    terminal.pose.position.y = terminal_.position.y();
    terminal.pose.orientation = YawQuaternion(terminal_.yaw);
    terminal_publisher_->publish(terminal);
    if (path.empty()) {
      return;
    }
    nav_msgs::msg::Path astar;
    astar.header = trajectory.header;
    for (const Eigen::Vector2d & q : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = astar.header;
      pose.pose.position.x = q.x();
      pose.pose.position.y = q.y();
      pose.pose.orientation.w = 1.0;
      astar.poses.push_back(pose);
    }
    path_publisher_->publish(astar);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (std::size_t i = 0; i < corridor.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header = trajectory.header;
      m.ns = "corridor";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.04;
      m.color.r = 0.1f;
      m.color.g = 0.8f;
      m.color.b = 0.3f;
      m.color.a = 0.8f;
      const std::vector<Eigen::Vector2d> vertices = PolygonVertices(corridor[i]);
      for (std::size_t k = 0; !vertices.empty() && k <= vertices.size(); ++k) {
        geometry_msgs::msg::Point q;
        q.x = vertices[k % vertices.size()].x();
        q.y = vertices[k % vertices.size()].y();
        m.points.push_back(q);
      }
      markers.markers.push_back(m);
    }
    corridor_publisher_->publish(markers);
  }

  // ---- Members ---------------------------------------------------------------------

  // Settings.
  std::string map_frame_;
  std::string base_frame_;
  std::string planner_id_;
  double planner_timeout_sec_;
  double sample_period_;
  double acceptance_position_;
  double acceptance_yaw_;
  bool replan_enabled_;
  double replan_rate_hz_;
  double replan_target_shift_;
  double tracking_error_replan_;
  double replan_handoff_lead_;
  double replan_dispatch_margin_;
  double replan_min_remaining_;
  int max_failures_;
  double retry_speed_factor_;
  double retry_period_;
  double target_max_age_;
  bool swimmer_check_enabled_ = true;
  CorridorSettings corridor_settings_;
  TerminalPoseSettings terminal_settings_;
  OptimizerSettings optimizer_settings_;
  Footprint footprint_;
  std::optional<SwimmerTrackStore> swimmers_;
  int forced_failures_ = 0;  // test hook

  // Goal and execution state.
  Goal goal_;
  uint64_t next_goal_id_ = 0;
  uint64_t next_request_id_ = 0;
  std::optional<TerminalPoseGenerator> generator_;
  TerminalPose terminal_;
  Eigen::Vector2d planned_victim_ = Eigen::Vector2d::Zero();
  Mode mode_ = Mode::kIdle;
  std::optional<Reference> active_;
  std::optional<Pending> pending_;
  Clock::time_point pending_sent_;
  std::optional<Attempt> attempt_;
  std::optional<Trigger> pending_trigger_;
  Clock::time_point last_attempt_;
  int failures_ = 0;
  int late_streak_ = 0;  // consecutive late replans
  rclcpp::Time last_swimmer_seen_{0, 0, RCL_ROS_TIME};
  bool slow_ = false;
  double slow_factor_ = 0.7;
  bool safety_hold_ = false;
  rov_interfaces::msg::SafetyStatus safety_;
  bool blocked_reported_ = false;
  bool swimmer_conflict_episode_ = false;
  std::shared_ptr<const OccupancyGrid2D> grid_;
  uint64_t map_version_ = 0;
  nav_msgs::msg::Odometry odom_;
  // Dry run.
  bool dry_run_ = false;
  bool dry_run_done_ = false;
  std::optional<FollowTrajectory::Goal> replay_goal_;
  uint64_t replay_request_ = 0;
  bool replay_conflict_ = false;

  // ROS interfaces.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<rov_interfaces::msg::VictimTarget>::SharedPtr victim_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<rov_interfaces::msg::SafetyStatus>::SharedPtr safety_subscription_;
  rclcpp::Subscription<rov_interfaces::msg::SwimmerTrackArray>::SharedPtr swimmer_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_publisher_;
  rclcpp::Publisher<PlannerStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr terminal_publisher_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr follow_client_;
  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  // Declared last, so it is destroyed (and its thread joined) first.
  std::unique_ptr<PlanningWorker> worker_;
};

}  // namespace rov_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_planning::TrajectoryGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
