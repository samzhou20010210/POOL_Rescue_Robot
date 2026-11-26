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

// fake_shore_node: simulated shore tracker for non-target swimmers
// (implementation step 9).
//
// Publishes rov_interfaces/SwimmerTrackArray on "swimmer_tracks" at
// publish_rate_hz with every swimmer that currently exists. Each track has a
// stable ID, the measurement stamp, the frame, a position and a velocity;
// Gaussian noise (position_noise_m, velocity_noise_mps) is added if > 0.
// The same tracks without noise go to "swimmer_truth" (for metrics only: the
// planner must never use it).
// Swimmers are configured as parallel arrays (one entry per swimmer), see
// config/fake_shore.yaml. Time counts from the node's start, on the node's
// clock (simulation time if use_sim_time), or, with active = false at start,
// from the moment the parameter `active` is set to true (scripted tests);
// nothing is published while inactive.
// Shore heartbeat: std_msgs/Header on "shore_heartbeat" at heartbeat_rate_hz
// while heartbeat_enabled (settable at run time: false simulates a lost shore
// link). It does not depend on `active`.

#include <chrono>
#include <cstdint>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/msg/swimmer_track_array.hpp"
#include "rov_simulation/swimmer_motion.hpp"
#include "std_msgs/msg/header.hpp"

namespace rov_simulation
{

class FakeShoreNode : public rclcpp::Node
{
public:
  FakeShoreNode()
  : Node("fake_shore_node")
  {
    const double rate_hz = declare_parameter("publish_rate_hz", 10.0);
    frame_id_ = declare_parameter("frame_id", std::string("map"));
    position_noise_ = declare_parameter("position_noise_m", 0.05);
    velocity_noise_ = declare_parameter("velocity_noise_mps", 0.05);
    random_.seed(static_cast<uint32_t>(declare_parameter("random_seed", 42)));
    active_ = declare_parameter("active", true);
    heartbeat_enabled_ = declare_parameter("heartbeat_enabled", true);
    const double heartbeat_rate_hz = declare_parameter("heartbeat_rate_hz", 10.0);
    if (!(heartbeat_rate_hz > 0.0)) {
      throw std::invalid_argument("heartbeat_rate_hz must be > 0");
    }
    const auto ids = declare_parameter("swimmer_ids", std::vector<int64_t>{});
    const auto motions = declare_parameter("swimmer_motions", std::vector<std::string>{});
    const auto start_x = declare_parameter("swimmer_start_x", std::vector<double>{});
    const auto start_y = declare_parameter("swimmer_start_y", std::vector<double>{});
    const auto velocity_x = declare_parameter("swimmer_velocity_x", std::vector<double>{});
    const auto velocity_y = declare_parameter("swimmer_velocity_y", std::vector<double>{});
    const auto travel = declare_parameter("swimmer_travel_m", std::vector<double>{});
    const auto start_time = declare_parameter("swimmer_start_time_s", std::vector<double>{});
    const auto lifetime = declare_parameter("swimmer_lifetime_s", std::vector<double>{});
    if (!(rate_hz > 0.0) || position_noise_ < 0.0 || velocity_noise_ < 0.0) {
      throw std::invalid_argument("publish_rate_hz must be > 0 and noise >= 0");
    }
    const std::size_t n = ids.size();
    if (motions.size() != n || start_x.size() != n || start_y.size() != n ||
      velocity_x.size() != n || velocity_y.size() != n || travel.size() != n ||
      start_time.size() != n || lifetime.size() != n)
    {
      throw std::invalid_argument("all swimmer_* arrays must have one entry per swimmer");
    }
    for (std::size_t i = 0; i < n; ++i) {
      SwimmerConfig swimmer;
      if (ids[i] < 0) {
        throw std::invalid_argument("swimmer IDs must be >= 0");
      }
      swimmer.id = static_cast<uint32_t>(ids[i]);
      swimmer.motion = ParseSwimmerMotion(motions[i]);
      swimmer.start = Eigen::Vector2d(start_x[i], start_y[i]);
      swimmer.velocity = Eigen::Vector2d(velocity_x[i], velocity_y[i]);
      swimmer.travel = travel[i];
      swimmer.start_time = start_time[i];
      swimmer.lifetime = lifetime[i];
      if (swimmer.motion == SwimmerMotion::kBackAndForth && !(swimmer.travel > 0.0)) {
        throw std::invalid_argument("back_and_forth swimmers need swimmer_travel_m > 0");
      }
      swimmers_.push_back(swimmer);
    }

    // Relative topic name, so it follows the node's namespace.
    publisher_ = create_publisher<rov_interfaces::msg::SwimmerTrackArray>("swimmer_tracks", 10);
    truth_publisher_ =
      create_publisher<rov_interfaces::msg::SwimmerTrackArray>("swimmer_truth", 10);
    start_ = now();
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const rclcpp::Parameter & parameter : parameters) {
          if (parameter.get_name() == "heartbeat_enabled") {
            heartbeat_enabled_ = parameter.as_bool();
            RCLCPP_WARN(
              get_logger(), "Shore heartbeat %s", heartbeat_enabled_ ? "resumed" : "stopped");
          }
          if (parameter.get_name() == "active") {
            const bool active = parameter.as_bool();
            if (active && !active_) {
              start_ = now();  // swimmer times count from activation
              RCLCPP_INFO(get_logger(), "fake_shore_node activated");
            }
            active_ = active;
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
    heartbeat_publisher_ = create_publisher<std_msgs::msg::Header>("shore_heartbeat", 10);
    heartbeat_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / heartbeat_rate_hz), [this]() {
        if (heartbeat_enabled_) {
          std_msgs::msg::Header beat;
          beat.stamp = now();
          beat.frame_id = frame_id_;
          heartbeat_publisher_->publish(beat);
        }
      });
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_hz), [this]() {Publish();});
    RCLCPP_INFO(
      get_logger(), "fake_shore_node: %zu swimmer(s), %.1f Hz, noise %.3f m / %.3f m/s, frame %s",
      swimmers_.size(), rate_hz, position_noise_, velocity_noise_, frame_id_.c_str());
  }

private:
  double Noise(double sigma)
  {
    if (sigma <= 0.0) {
      return 0.0;
    }
    return std::normal_distribution<double>(0.0, sigma)(random_);
  }

  void Publish()
  {
    if (!active_) {
      return;
    }
    const rclcpp::Time stamp = now();
    const double t = (stamp - start_).seconds();
    rov_interfaces::msg::SwimmerTrackArray message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    rov_interfaces::msg::SwimmerTrackArray truth = message;
    for (const SwimmerConfig & swimmer : swimmers_) {
      const std::optional<SwimmerState> state = SwimmerStateAt(swimmer, t);
      if (!state) {
        continue;  // not started yet, or its lifetime is over: the track ends
      }
      rov_interfaces::msg::SwimmerTrack track;
      track.header = message.header;
      track.id = swimmer.id;
      track.position.x = state->position.x();
      track.position.y = state->position.y();
      track.velocity.x = state->velocity.x();
      track.velocity.y = state->velocity.y();
      truth.tracks.push_back(track);
      track.position.x = state->position.x() + Noise(position_noise_);
      track.position.y = state->position.y() + Noise(position_noise_);
      track.velocity.x = state->velocity.x() + Noise(velocity_noise_);
      track.velocity.y = state->velocity.y() + Noise(velocity_noise_);
      message.tracks.push_back(track);
    }
    publisher_->publish(message);
    truth_publisher_->publish(truth);
  }

  std::string frame_id_;
  bool active_ = true;
  bool heartbeat_enabled_ = true;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  double position_noise_;
  double velocity_noise_;
  std::mt19937 random_;
  std::vector<SwimmerConfig> swimmers_;
  rclcpp::Time start_;
  rclcpp::Publisher<rov_interfaces::msg::SwimmerTrackArray>::SharedPtr publisher_;
  rclcpp::Publisher<rov_interfaces::msg::SwimmerTrackArray>::SharedPtr truth_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rov_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_simulation::FakeShoreNode>());
  rclcpp::shutdown();
  return 0;
}
