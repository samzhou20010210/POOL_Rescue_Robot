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

// Simulated swimmer motion (fake shore, implementation step 9).

#include <gtest/gtest.h>

#include <stdexcept>

#include "rov_simulation/swimmer_motion.hpp"

namespace rov_simulation
{
namespace
{

TEST(SwimmerMotionTest, StraightMovesWithConstantVelocity)
{
  SwimmerConfig s;
  s.start = Eigen::Vector2d(1.0, 2.0);
  s.velocity = Eigen::Vector2d(0.5, -0.25);
  s.start_time = 2.0;
  const auto state = SwimmerStateAt(s, 6.0);  // 4 s after its start
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(state->position.isApprox(Eigen::Vector2d(3.0, 1.0)));
  EXPECT_TRUE(state->velocity.isApprox(s.velocity));
}

TEST(SwimmerMotionTest, BackAndForthTurnsAtTheEndsOfItsLeg)
{
  SwimmerConfig s;
  s.motion = SwimmerMotion::kBackAndForth;
  s.velocity = Eigen::Vector2d(1.0, 0.0);
  s.travel = 4.0;
  auto at = [&s](double t) {return *SwimmerStateAt(s, t);};
  EXPECT_NEAR(at(3.0).position.x(), 3.0, 1e-12);   // outbound
  EXPECT_NEAR(at(3.0).velocity.x(), 1.0, 1e-12);
  EXPECT_NEAR(at(6.0).position.x(), 2.0, 1e-12);   // on the way back
  EXPECT_NEAR(at(6.0).velocity.x(), -1.0, 1e-12);
  EXPECT_NEAR(at(8.0).position.x(), 0.0, 1e-12);   // back at the start
  EXPECT_NEAR(at(9.0).position.x(), 1.0, 1e-12);   // and out again
}

TEST(SwimmerMotionTest, TrackExistsOnlyDuringItsLifetime)
{
  SwimmerConfig s;
  s.start_time = 1.0;
  s.lifetime = 5.0;
  EXPECT_FALSE(SwimmerStateAt(s, 0.5).has_value());
  EXPECT_TRUE(SwimmerStateAt(s, 3.0).has_value());
  EXPECT_FALSE(SwimmerStateAt(s, 6.5).has_value());
  s.lifetime = 0.0;  // forever
  EXPECT_TRUE(SwimmerStateAt(s, 1e6).has_value());
  EXPECT_THROW(ParseSwimmerMotion("zigzag"), std::invalid_argument);
}

}  // namespace
}  // namespace rov_simulation
