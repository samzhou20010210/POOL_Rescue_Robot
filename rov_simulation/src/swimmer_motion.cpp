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

#include "rov_simulation/swimmer_motion.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace rov_simulation
{

SwimmerMotion ParseSwimmerMotion(const std::string & name)
{
  if (name == "straight") {
    return SwimmerMotion::kStraight;
  }
  if (name == "back_and_forth") {
    return SwimmerMotion::kBackAndForth;
  }
  throw std::invalid_argument("swimmer motion must be 'straight' or 'back_and_forth', not '" +
          name + "'");
}

std::optional<SwimmerState> SwimmerStateAt(const SwimmerConfig & config, double t)
{
  const double elapsed = t - config.start_time;
  if (elapsed < 0.0 || (config.lifetime > 0.0 && elapsed > config.lifetime)) {
    return std::nullopt;
  }
  SwimmerState state;
  const double speed = config.velocity.norm();
  if (config.motion == SwimmerMotion::kStraight || speed == 0.0 || !(config.travel > 0.0)) {
    state.position = config.start + config.velocity * elapsed;
    state.velocity = config.velocity;
    return state;
  }
  // Back and forth on the segment [start, start + travel * direction].
  const Eigen::Vector2d direction = config.velocity / speed;
  const double distance = std::fmod(speed * elapsed, 2.0 * config.travel);
  const bool outbound = distance <= config.travel;
  const double along = outbound ? distance : 2.0 * config.travel - distance;
  state.position = config.start + direction * along;
  state.velocity = outbound ? config.velocity : Eigen::Vector2d(-config.velocity);
  return state;
}

}  // namespace rov_simulation
