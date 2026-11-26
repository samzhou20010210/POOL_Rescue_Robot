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

// Sampled references and braking (implementation steps 8B and 9).

#include <gtest/gtest.h>

#include <cmath>

#include "rov_planning/reference_trajectory.hpp"

namespace rov_planning
{
namespace
{

// 1.5 m/s along +x for 10 s, sampled every 0.05 s.
RefTrajectory Cruise()
{
  RefTrajectory r;
  for (double t = 0.0; t <= 10.0 + 1e-9; t += 0.05) {
    RefPoint p;
    p.t = t;
    p.position = Eigen::Vector2d(1.5 * t, 0.0);
    p.velocity = Eigen::Vector2d(1.5, 0.0);
    r.push_back(p);
  }
  return r;
}

TEST(ReferenceTrajectoryTest, BrakeStopsAtTheBrakingDistanceAlongThePath)
{
  const RefTrajectory brake = BrakeAlong(Cruise(), 2.0, 1.0, 0.05);
  // Starts where the reference is at t0 = 2 s, moving at 1.5 m/s.
  EXPECT_NEAR(brake.front().position.x(), 3.0, 1e-9);
  EXPECT_NEAR(brake.front().velocity.x(), 1.5, 1e-9);
  // Stops after v^2 / (2 a) = 1.125 m and 1.5 s, and stays there.
  EXPECT_NEAR(brake.back().position.x(), 3.0 + 1.125, 1e-9);
  EXPECT_NEAR(brake.back().t, 1.5 + 0.05, 1e-6);
  EXPECT_TRUE(brake.back().velocity.isZero());
  double max_deceleration = 0.0;
  for (std::size_t k = 1; k < brake.size(); ++k) {
    EXPECT_GE(brake[k].position.x(), brake[k - 1].position.x() - 1e-12);  // never backwards
    EXPECT_NEAR(brake[k].position.y(), 0.0, 1e-12);                      // on the path
    max_deceleration = std::max(
      max_deceleration,
      (brake[k - 1].velocity.x() - brake[k].velocity.x()) / (brake[k].t - brake[k - 1].t));
  }
  EXPECT_LE(max_deceleration, 1.0 + 1e-5);
}

TEST(ReferenceTrajectoryTest, BrakeFollowsACurvedPath)
{
  // Quarter circle of radius 2 m at 1 m/s.
  RefTrajectory arc;
  for (double t = 0.0; t <= M_PI + 1e-9; t += 0.02) {
    RefPoint p;
    p.t = t;
    const double angle = t / 2.0;
    p.position = Eigen::Vector2d(2.0 * std::sin(angle), 2.0 - 2.0 * std::cos(angle));
    p.velocity = Eigen::Vector2d(std::cos(angle), std::sin(angle));
    p.yaw = angle;
    arc.push_back(p);
  }
  const RefTrajectory brake = BrakeAlong(arc, 0.0, 1.0, 0.02);
  for (const RefPoint & p : brake) {
    EXPECT_NEAR((p.position - Eigen::Vector2d(0.0, 2.0)).norm(), 2.0, 2e-3);  // on the circle
  }
  EXPECT_NEAR(brake.back().yaw, 0.25, 0.01);  // 0.5 m of arc at radius 2
}

TEST(ReferenceTrajectoryTest, BrakeStopsAtACuspInsteadOfDoublingBack)
{
  // A replan that turns back: 1.4 m/s along +x, decelerating at 2 m/s^2 to
  // rest at x = 0.49 m (t = 0.7 s), then back along -x. Braking at 1 m/s^2
  // would need 0.98 m, so the brake follows the reference to its cusp.
  RefTrajectory turn_back;
  for (double t = 0.0; t <= 2.0 + 1e-9; t += 0.02) {
    RefPoint p;
    p.t = t;
    p.position = Eigen::Vector2d(1.4 * t - t * t, 0.0);
    p.velocity = Eigen::Vector2d(1.4 - 2.0 * t, 0.0);
    p.acceleration = Eigen::Vector2d(-2.0, 0.0);
    turn_back.push_back(p);
  }
  const RefTrajectory brake = BrakeAlong(turn_back, 0.0, 1.0, 0.02);
  EXPECT_NEAR(brake.back().position.x(), 0.49, 1e-3);
  EXPECT_NEAR(brake.back().t, 0.7 + 0.02, 0.01);
  EXPECT_TRUE(brake.back().velocity.isZero());
  for (std::size_t k = 1; k < brake.size(); ++k) {
    EXPECT_GE(brake[k].position.x(), brake[k - 1].position.x() - 1e-12);  // never backwards
    EXPECT_GE(brake[k].velocity.x(), -1e-12);
  }
}

TEST(ReferenceTrajectoryTest, BrakeIsNeverFasterThanTheReferenceOnThePath)
{
  // The reference slows from 1.4 to 0.4 m/s at 2 m/s^2 (x = 0.45 m), then
  // cruises at 0.4 m/s (as through a tight turn). Braking at 1 m/s^2 alone
  // would still be at 1.06 m/s there; the brake takes the reference's speed,
  // holds 0.4 m/s until the braking curve falls below it (x = 0.9 m), and
  // stops at v0^2 / (2 a) = 0.98 m.
  RefTrajectory slow_turn;
  for (double t = 0.0; t <= 4.0 + 1e-9; t += 0.01) {
    RefPoint p;
    p.t = t;
    const double tb = std::min(t, 0.5);
    const double x = 1.4 * tb - tb * tb + 0.4 * (t - tb);
    p.position = Eigen::Vector2d(x, 0.0);
    p.velocity = Eigen::Vector2d(t < 0.5 ? 1.4 - 2.0 * t : 0.4, 0.0);
    slow_turn.push_back(p);
  }
  const RefTrajectory brake = BrakeAlong(slow_turn, 0.0, 1.0, 0.01);
  for (const RefPoint & p : brake) {
    const double x = p.position.x();
    const double ref = x < 0.45 ? std::sqrt(std::max(0.0, 1.96 - 4.0 * x)) : 0.4;
    EXPECT_LE(p.velocity.x(), ref + 0.01) << "at x = " << x;
    EXPECT_LE(p.velocity.x(), std::sqrt(std::max(0.0, 1.96 - 2.0 * x)) + 0.01) << "at x = " << x;
  }
  EXPECT_NEAR(brake.back().position.x(), 0.98, 1e-3);
  EXPECT_TRUE(brake.back().velocity.isZero());
}

TEST(ReferenceTrajectoryTest, AtRestGivesAHoldAndInterpolationClamps)
{
  RefTrajectory still = HoldAt(Eigen::Vector3d(1.0, 2.0, 0.5), 0.05);
  const RefTrajectory brake = BrakeAlong(still, 0.0, 1.0, 0.05);
  ASSERT_EQ(brake.size(), 2u);
  EXPECT_TRUE(brake.front().position.isApprox(Eigen::Vector2d(1.0, 2.0)));
  const RefTrajectory cruise = Cruise();
  EXPECT_NEAR(InterpolateRef(cruise, 1.025).position.x(), 1.5375, 1e-9);
  EXPECT_NEAR(InterpolateRef(cruise, 99.0).position.x(), 15.0, 1e-9);
  EXPECT_NEAR(MaxSpeed(cruise), 1.5, 1e-12);
}

}  // namespace
}  // namespace rov_planning
