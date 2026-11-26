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

// Motion of simulated non-target swimmers for the fake shore tracker
// (implementation step 9). No ROS.
//
// A swimmer starts at `start` at time `start_time` (s since the shore
// started) and moves with `velocity` (m/s):
//   kStraight      p(t) = start + velocity * (t - start_time);
//   kBackAndForth  goes `travel` metres along velocity, turns back to the
//                  start, and repeats (velocity flips sign at each turn).
// It exists from start_time for `lifetime` seconds (lifetime <= 0: forever);
// outside that window it has no state, so its track ends.

#ifndef ROV_SIMULATION__SWIMMER_MOTION_HPP_
#define ROV_SIMULATION__SWIMMER_MOTION_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "Eigen/Dense"

namespace rov_simulation
{

enum class SwimmerMotion { kStraight, kBackAndForth };

// "straight" or "back_and_forth"; throws std::invalid_argument otherwise.
SwimmerMotion ParseSwimmerMotion(const std::string & name);

struct SwimmerConfig
{
  uint32_t id = 0;
  SwimmerMotion motion = SwimmerMotion::kStraight;
  Eigen::Vector2d start = Eigen::Vector2d::Zero();     // m
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();  // m/s
  double travel = 0.0;      // m, back-and-forth leg length (> 0 for kBackAndForth)
  double start_time = 0.0;  // s
  double lifetime = 0.0;    // s, <= 0: forever
};

struct SwimmerState
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  // m
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();  // m/s
};

// State at time t (s since the shore started), or empty if the swimmer
// does not exist at t.
std::optional<SwimmerState> SwimmerStateAt(const SwimmerConfig & config, double t);

}  // namespace rov_simulation

#endif  // ROV_SIMULATION__SWIMMER_MOTION_HPP_
