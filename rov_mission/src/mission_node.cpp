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
// and logs the goal's frame ID and x/y position.

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rov_mission
{

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

    RCLCPP_INFO(get_logger(), "mission_node started, waiting for goals on 'goal_pose'");
  }

private:
  // Called once for every goal message that arrives.
  void OnGoalReceived(const geometry_msgs::msg::PoseStamped & goal)
  {
    RCLCPP_INFO(
      get_logger(), "Received goal: frame_id='%s', x=%.2f, y=%.2f",
      goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
  }

  // The subscription must be kept alive as a member; if it were a local
  // variable it would be destroyed and no messages would be received.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
};

}  // namespace rov_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_mission::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
