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

// mission_node: listens for goals sent from RViz2's "2D Goal Pose" tool,
// logs the goal's frame ID and x/y position, and forwards the pose to the
// controller as a single-pose FollowPath action goal.

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rov_mission
{

using FollowPath = nav2_msgs::action::FollowPath;
using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission_node")
  {
    // "goal_pose" is a relative topic name (no leading "/"), so it is
    // resolved inside this node's namespace. With no namespace it becomes
    // "/goal_pose", which is where RViz2 publishes 2D Goal Pose clicks.
    // The queue depth of 10 keeps up to 10 goals waiting if we fall behind.
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & goal) {OnGoalReceived(goal);});

    // Action client for the controller's "follow_path" action server.
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    RCLCPP_INFO(get_logger(), "mission_node started, waiting for goals on 'goal_pose'");
  }

private:
  // Called once for every goal message that arrives.
  void OnGoalReceived(const geometry_msgs::msg::PoseStamped & goal)
  {
    RCLCPP_INFO(
      get_logger(), "Received goal: frame_id='%s', x=%.2f, y=%.2f",
      goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
    SendFollowPathGoal(goal);
  }

  // Send the pose as a one-pose path. This returns immediately; the
  // callbacks below are called later when the controller answers.
  void SendFollowPathGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!follow_path_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "follow_path action server is not available, goal not sent");
      return;
    }

    FollowPath::Goal goal;
    goal.path.header = pose.header;
    goal.path.poses.push_back(pose);

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const FollowPathGoalHandle::SharedPtr & goal_handle) {
        OnGoalResponse(goal_handle);
      };
    options.result_callback =
      [this](const FollowPathGoalHandle::WrappedResult & result) {OnGoalResult(result);};
    follow_path_client_->async_send_goal(goal, options);
  }

  // Called once the controller has accepted or rejected the goal.
  void OnGoalResponse(const FollowPathGoalHandle::SharedPtr & goal_handle)
  {
    if (goal_handle) {
      RCLCPP_INFO(get_logger(), "FollowPath goal accepted");
    } else {
      RCLCPP_WARN(get_logger(), "FollowPath goal rejected");
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
  }

  // The subscription must be kept alive as a member; if it were a local
  // variable it would be destroyed and no messages would be received.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
};

}  // namespace rov_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_mission::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
