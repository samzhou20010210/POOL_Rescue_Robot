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

// mission_node: listens for goals sent from RViz2's "2D Goal Pose" tool
// and logs the goal's frame ID and x/y position. What happens next depends
// on the "mode" parameter:
//   - "drive" (default): send the pose to the controller as a single-pose
//     FollowPath goal, so the robot drives straight there.
//   - "planning_preview": ask Nav2's planner for a route
//     (ComputePathToPose) and only log the result; the robot does not move.
//   - "planned_drive": ask the planner for a route, then send that route to
//     the controller (FollowPath). One plan-and-drive task runs at a time;
//     clicks that arrive while a task is running are ignored.
// All action calls are asynchronous: no callback waits for an answer.

#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rov_mission
{

using FollowPath = nav2_msgs::action::FollowPath;
using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

// Readable text for a ComputePathToPose error code.
std::string PlanErrorToString(uint16_t error_code)
{
  using Result = ComputePath::Result;
  switch (error_code) {
    case Result::INVALID_PLANNER: return "invalid planner";
    case Result::TF_ERROR: return "TF error (is map -> odom -> base_link available?)";
    case Result::START_OUTSIDE_MAP: return "robot is outside the map";
    case Result::GOAL_OUTSIDE_MAP: return "goal is outside the map";
    case Result::START_OCCUPIED: return "robot is inside an obstacle";
    case Result::GOAL_OCCUPIED: return "goal is inside an obstacle";
    case Result::TIMEOUT: return "planner timed out";
    case Result::NO_VALID_PATH: return "no valid path to the goal";
    default: return "unknown error";
  }
}

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission_node")
  {
    const std::string mode = declare_parameter("mode", std::string("drive"));
    if (mode == "drive") {
      mode_ = Mode::kDrive;
    } else if (mode == "planning_preview") {
      mode_ = Mode::kPlanningPreview;
    } else if (mode == "planned_drive") {
      mode_ = Mode::kPlannedDrive;
    } else {
      throw std::invalid_argument(
              "mode must be 'drive', 'planning_preview' or 'planned_drive', got '" + mode + "'");
    }

    // "goal_pose" is a relative topic name (no leading "/"), so it is
    // resolved inside this node's namespace. With no namespace it becomes
    // "/goal_pose", which is where RViz2 publishes 2D Goal Pose clicks.
    // The queue depth of 10 keeps up to 10 goals waiting if we fall behind.
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & goal) {OnGoalReceived(goal);});

    // Action clients: the controller's "follow_path" and Nav2's planner.
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
    compute_path_client_ =
      rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");

    RCLCPP_INFO(
      get_logger(), "mission_node started (mode '%s'), waiting for goals on 'goal_pose'",
      mode.c_str());
  }

private:
  enum class Mode
  {
    kDrive,
    kPlanningPreview,
    kPlannedDrive,
  };

  // Called once for every goal message that arrives.
  void OnGoalReceived(const geometry_msgs::msg::PoseStamped & goal)
  {
    RCLCPP_INFO(
      get_logger(), "Received goal: frame_id='%s', x=%.2f, y=%.2f",
      goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);

    switch (mode_) {
      case Mode::kDrive: {
          // A path with just the clicked pose: drive straight there.
          nav_msgs::msg::Path path;
          path.header = goal.header;
          path.poses.push_back(goal);
          SendFollowPathGoal(path);
          break;
        }
      case Mode::kPlanningPreview:
        SendComputePathGoal(goal);
        break;
      case Mode::kPlannedDrive:
        if (task_active_) {
          RCLCPP_WARN(get_logger(), "Ignoring goal: a plan-and-drive task is still running");
          return;
        }
        if (SendComputePathGoal(goal)) {
          task_active_ = true;  // Cleared when planning fails or driving ends.
        }
        break;
    }
  }

  // ---- Driving: FollowPath ----

  // Send a path to the controller. Returns false if it could not be sent.
  // This returns immediately; the callbacks below are called later.
  bool SendFollowPathGoal(const nav_msgs::msg::Path & path)
  {
    if (!follow_path_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "follow_path action server is not available, goal not sent");
      return false;
    }

    FollowPath::Goal goal;
    goal.path = path;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const FollowPathGoalHandle::SharedPtr & goal_handle) {
        OnGoalResponse(goal_handle);
      };
    options.result_callback =
      [this](const FollowPathGoalHandle::WrappedResult & result) {OnGoalResult(result);};
    follow_path_client_->async_send_goal(goal, options);
    return true;
  }

  // Called once the controller has accepted or rejected the goal.
  void OnGoalResponse(const FollowPathGoalHandle::SharedPtr & goal_handle)
  {
    if (goal_handle) {
      RCLCPP_INFO(get_logger(), "FollowPath goal accepted");
    } else {
      RCLCPP_WARN(get_logger(), "FollowPath goal rejected");
      task_active_ = false;
    }
  }

  // Called once the accepted goal has finished.
  void OnGoalResult(const FollowPathGoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "FollowPath goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(
          get_logger(), "FollowPath goal aborted (error %u: %s)",
          result.result->error_code, result.result->error_msg.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "FollowPath goal canceled");
        break;
      default:
        RCLCPP_WARN(get_logger(), "FollowPath goal ended with an unknown result");
        break;
    }
    task_active_ = false;
  }

  // ---- Planning: ComputePathToPose ----

  // Ask the planner for a route from the robot's current pose to the goal.
  // Returns false if the request could not be sent.
  // This returns immediately; the callbacks below are called later.
  bool SendComputePathGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!compute_path_client_->action_server_is_ready()) {
      RCLCPP_WARN(
        get_logger(), "compute_path_to_pose action server is not available, goal not sent");
      return false;
    }

    ComputePath::Goal goal;
    goal.goal = pose;
    goal.use_start = false;  // Start from the robot's current pose (from TF).

    rclcpp_action::Client<ComputePath>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const ComputePathGoalHandle::SharedPtr & goal_handle) {
        OnPlanResponse(goal_handle);
      };
    options.result_callback =
      [this](const ComputePathGoalHandle::WrappedResult & result) {OnPlanResult(result);};
    compute_path_client_->async_send_goal(goal, options);
    return true;
  }

  void OnPlanResponse(const ComputePathGoalHandle::SharedPtr & goal_handle)
  {
    if (goal_handle) {
      RCLCPP_INFO(get_logger(), "ComputePathToPose goal accepted");
    } else {
      RCLCPP_WARN(get_logger(), "ComputePathToPose goal rejected");
      task_active_ = false;
    }
  }

  void OnPlanResult(const ComputePathGoalHandle::WrappedResult & result)
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_WARN(
          get_logger(), "Planning failed: %s (error %u%s%s)",
          PlanErrorToString(result.result->error_code).c_str(), result.result->error_code,
          result.result->error_msg.empty() ? "" : ": ", result.result->error_msg.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Planning did not finish (canceled or unknown result)");
      }
      task_active_ = false;  // The robot was never sent anywhere; it stays stopped.
      return;
    }

    const nav_msgs::msg::Path & path = result.result->path;
    if (mode_ == Mode::kPlanningPreview) {
      RCLCPP_INFO(
        get_logger(), "Path planned: %zu poses (robot stays still in preview mode)",
        path.poses.size());
      return;
    }

    // Planned drive: hand this exact path to the controller.
    if (path.poses.empty()) {
      RCLCPP_WARN(get_logger(), "Planner returned an empty path; not driving");
      task_active_ = false;
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Path planned: %zu poses, sending it to the controller", path.poses.size());
    if (!SendFollowPathGoal(path)) {
      task_active_ = false;
    }
  }

  Mode mode_ = Mode::kDrive;
  // True while a planned_drive task (planning, then driving) is running.
  bool task_active_ = false;

  // The subscription must be kept alive as a member; if it were a local
  // variable it would be destroyed and no messages would be received.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
};

}  // namespace rov_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_mission::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
