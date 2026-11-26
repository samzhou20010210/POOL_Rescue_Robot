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

// gazebo_thruster_relay: connects the ROS thruster interface to the Gazebo
// PlanarThrusterSystem plugin. ros_gz_bridge cannot carry the custom
// ThrusterCommand message, so this node publishes on Gazebo Transport itself.
//
// Subscribes:
//   thruster_command   rov_interfaces/ThrusterCommand (frame_id base_link).
//                      Each valid message is forwarded at once as a
//                      gz.msgs.Double_V with the 8 forces in the same order.
//                      Messages in another frame or with NaN/inf are dropped.
// Service:
//   reset_vehicle      std_srvs/Trigger: asks the plugin to put the model
//                      back at its spawn pose, at rest, with zero thrust.
//
// Odometry: the bridge publishes Gazebo's odometry and TF on gz_odom /
// gz_tf; this node republishes them on odom / tf while publish_odometry is
// true (test hook, settable at run time: false simulates lost odometry, the
// vehicle keeps moving).
//
// The command timeout is applied by the plugin on simulation time, so the
// thrust is cut even if this relay stops.

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gz/msgs/double_v.pb.h"
#include "gz/msgs/empty.pb.h"
#include "gz/transport/Node.hh"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rov_interfaces/msg/thruster_command.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rov_simulation
{

class GazeboThrusterRelay : public rclcpp::Node
{
public:
  GazeboThrusterRelay()
  : Node("gazebo_thruster_relay")
  {
    const std::string command_topic =
      declare_parameter("gz_command_topic", std::string("/model/rov_v4/thruster_forces"));
    const std::string reset_topic =
      declare_parameter("gz_reset_topic", std::string("/model/rov_v4/reset_vehicle"));
    base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));

    gz_command_publisher_ = gz_node_.Advertise<gz::msgs::Double_V>(command_topic);
    gz_reset_publisher_ = gz_node_.Advertise<gz::msgs::Empty>(reset_topic);
    if (!gz_command_publisher_ || !gz_reset_publisher_) {
      throw std::runtime_error("cannot advertise the Gazebo topics");
    }

    // Relative names, so they follow the node's namespace.
    command_subscription_ = create_subscription<rov_interfaces::msg::ThrusterCommand>(
      "thruster_command", 10,
      [this](const rov_interfaces::msg::ThrusterCommand & message) {OnThrusterCommand(message);});
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "reset_vehicle",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {OnReset(*response);});

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
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "gz_odom", 10, [this](const nav_msgs::msg::Odometry & m) {
        if (publish_odometry_) {
          odom_publisher_->publish(m);
        }
      });
    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "gz_tf", 100, [this](const tf2_msgs::msg::TFMessage & m) {
        if (publish_odometry_) {
          tf_publisher_->publish(m);
        }
      });

    RCLCPP_INFO(
      get_logger(), "Relaying thruster_command to gz %s, reset_vehicle to gz %s",
      command_topic.c_str(), reset_topic.c_str());
  }

private:
  void OnThrusterCommand(const rov_interfaces::msg::ThrusterCommand & message)
  {
    if (message.header.frame_id != base_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Dropping thruster_command in frame '%s'; expected '%s'",
        message.header.frame_id.c_str(), base_frame_id_.c_str());
      return;
    }
    gz::msgs::Double_V forces;
    for (const double force : message.force) {
      if (!std::isfinite(force)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Dropping thruster_command with NaN or infinite force");
        return;
      }
      forces.add_data(force);
    }
    gz_command_publisher_.Publish(forces);
  }

  void OnReset(std_srvs::srv::Trigger::Response & response)
  {
    response.success = gz_reset_publisher_.Publish(gz::msgs::Empty());
    response.message = response.success ? "reset sent to Gazebo" : "Gazebo publish failed";
  }

  std::string base_frame_id_;
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_command_publisher_;
  gz::transport::Node::Publisher gz_reset_publisher_;
  rclcpp::Subscription<rov_interfaces::msg::ThrusterCommand>::SharedPtr command_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  bool publish_odometry_ = true;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace rov_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_simulation::GazeboThrusterRelay>());
  rclcpp::shutdown();
  return 0;
}
