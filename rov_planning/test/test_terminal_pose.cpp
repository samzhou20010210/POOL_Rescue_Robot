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

// Offline tests of terminal-pose generation (implementation step 8A).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

#include "rov_planning/corridor_2d.hpp"
#include "rov_planning/terminal_pose.hpp"

namespace rov_planning
{
namespace
{

class TerminalPoseTest : public ::testing::Test
{
protected:
  TerminalPoseTest()
  : settings_(LoadTerminalPoseSettings(TERMINAL_POSE_YAML)) {}

  TerminalPoseSettings settings_;
};

TEST_F(TerminalPoseTest, LoadsTheExplicitSimulationValues)
{
  EXPECT_DOUBLE_EQ(settings_.stowed_arm_protrusion, 0.10);
  EXPECT_DOUBLE_EQ(Standoff(settings_, VictimState::kSurface), 1.00);
  EXPECT_DOUBLE_EQ(Standoff(settings_, VictimState::kSubmerged), 1.15);
}

// Both victim states: one bow-facing pose, only the standoff differs.
TEST_F(TerminalPoseTest, BothVictimStatesStopBowOnAtTheirStandoff)
{
  const Eigen::Vector2d robot(0.0, 0.0);
  const Eigen::Vector2d victim(3.0, 4.0);  // 5 m away, bearing atan2(4, 3)
  const double bearing = std::atan2(4.0, 3.0);
  const Eigen::Vector2d direction(0.6, 0.8);
  for (const VictimState state : {VictimState::kSurface, VictimState::kSubmerged}) {
    TerminalPoseGenerator generator(settings_);
    const TerminalPose pose = generator.Update(robot, victim, state);
    const double d = Standoff(settings_, state);
    const double gap = state == VictimState::kSurface ? 0.15 : 0.30;
    EXPECT_NEAR(pose.standoff, d, 1e-12);
    EXPECT_NEAR(pose.yaw, bearing, 1e-12);  // bow faces the victim
    EXPECT_TRUE(pose.position.isApprox(victim - d * direction, 1e-12));
    EXPECT_FALSE(pose.heading_held);  // 5 m > d + d_hold: still recomputing

    // At the nominal pose, the stowed arm tips are exactly g from the body.
    const ClearanceReport clearance = CheckTerminalClearance(
      Eigen::Vector3d(pose.position.x(), pose.position.y(), pose.yaw), victim, settings_,
      nullptr);
    EXPECT_NEAR(clearance.victim_gap, gap, 1e-12);
    EXPECT_TRUE(clearance.ok) << clearance.reason;
  }
  EXPECT_NEAR(
    Standoff(settings_, VictimState::kSubmerged) - Standoff(settings_, VictimState::kSurface),
    0.15, 1e-12);
}

TEST_F(TerminalPoseTest, HeadingIsHeldDuringFinalApproach)
{
  TerminalPoseGenerator generator(settings_);
  const Eigen::Vector2d victim(10.0, 0.0);
  const double zone = Standoff(settings_, VictimState::kSurface) + 2.0;  // 3.0 m

  // Transit: the heading follows the bearing.
  TerminalPose pose = generator.Update({0.0, 1.0}, victim, VictimState::kSurface);
  EXPECT_FALSE(pose.heading_held);
  EXPECT_NEAR(pose.yaw, std::atan2(-1.0, 10.0), 1e-12);
  pose = generator.Update({2.0, -1.0}, victim, VictimState::kSurface);
  EXPECT_NEAR(pose.yaw, std::atan2(1.0, 8.0), 1e-12);

  // Enter the zone on the x axis: the heading is frozen at 0.
  pose = generator.Update({10.0 - zone + 0.1, 0.0}, victim, VictimState::kSurface);
  ASSERT_TRUE(pose.heading_held);
  EXPECT_NEAR(pose.yaw, 0.0, 1e-12);

  // Noisy robot (+-0.15 m sideways) and victim (+-0.05 m) positions while
  // closing in: the naive bearing swings, the held heading does not.
  double largest_bearing_swing = 0.0;
  for (int k = 0; k < 40; ++k) {
    const Eigen::Vector2d robot(7.2 + 0.03 * k, 0.15 * std::sin(1.3 * k));
    const Eigen::Vector2d noisy_victim =
      victim + Eigen::Vector2d(0.05 * std::cos(2.1 * k), 0.05 * std::sin(0.7 * k));
    pose = generator.Update(robot, noisy_victim, VictimState::kSurface);
    EXPECT_TRUE(pose.heading_held);
    EXPECT_EQ(pose.yaw, 0.0) << "update " << k;
    // The goal still follows the victim along the held heading.
    EXPECT_TRUE(pose.position.isApprox(noisy_victim - Eigen::Vector2d(1.0, 0.0), 1e-12));
    const Eigen::Vector2d to = noisy_victim - robot;
    largest_bearing_swing = std::max(largest_bearing_swing, std::abs(std::atan2(to.y(), to.x())));
  }
  std::printf("[ heading    ] naive bearing swings up to %.3f rad; held heading 0\n",
    largest_bearing_swing);
  EXPECT_GT(largest_bearing_swing, 0.05);

  // The victim drifts 1 m sideways: bearing off by > 0.15 rad, re-aim once.
  const Eigen::Vector2d robot(8.0, 0.0);
  const Eigen::Vector2d moved(10.0, 1.0);
  pose = generator.Update(robot, moved, VictimState::kSurface);
  EXPECT_TRUE(pose.heading_held);
  EXPECT_NEAR(pose.yaw, std::atan2(1.0, 2.0), 1e-12);
  const double reaimed = pose.yaw;
  pose = generator.Update(robot + Eigen::Vector2d(0.0, 0.05), moved, VictimState::kSurface);
  EXPECT_EQ(pose.yaw, reaimed);  // held again

  // Back out beyond the zone + hysteresis: released, follows the bearing.
  pose = generator.Update({10.0 - zone - 0.6, 1.0}, moved, VictimState::kSurface);
  EXPECT_FALSE(pose.heading_held);
  // Inside the hysteresis band it would still hold:
  TerminalPoseGenerator other(settings_);
  other.Update({10.0 - zone + 0.1, 0.0}, victim, VictimState::kSurface);
  EXPECT_TRUE(other.Update({10.0 - zone - 0.3, 0.0}, victim, VictimState::kSurface).heading_held);
  other.Reset();
  EXPECT_FALSE(other.holding());
}

TEST_F(TerminalPoseTest, InvalidInputsAreRejected)
{
  TerminalPoseGenerator generator(settings_);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    generator.Update({nan, 0.0}, {5.0, 0.0}, VictimState::kSurface), std::invalid_argument);
  EXPECT_THROW(
    generator.Update({0.0, 0.0}, {5.0, std::numeric_limits<double>::infinity()},
      VictimState::kSubmerged), std::invalid_argument);
  // Robot at the victim, nothing held: no approach direction.
  EXPECT_THROW(
    generator.Update({5.0, 0.0}, {5.0, 0.0}, VictimState::kSurface), std::invalid_argument);
  // Once a heading is held, the same position keeps it instead of failing.
  generator.Update({4.0, 0.0}, {5.0, 0.0}, VictimState::kSurface);
  EXPECT_NO_THROW(generator.Update({5.0, 0.0}, {5.0, 0.0}, VictimState::kSurface));

  // Settings: L_arm never set, or nonsense values.
  TerminalPoseSettings unset;  // stowed_arm_protrusion stays NaN
  EXPECT_THROW(TerminalPoseGenerator{unset}, std::invalid_argument);
  TerminalPoseSettings bad = settings_;
  bad.surface_gap = -0.1;
  EXPECT_THROW(TerminalPoseGenerator{bad}, std::invalid_argument);
  bad = settings_;
  bad.heading_reaim_angle = 0.0;
  EXPECT_THROW(TerminalPoseGenerator{bad}, std::invalid_argument);
}

TEST_F(TerminalPoseTest, LoaderRequiresAnExplicitArmProtrusion)
{
  const std::string path = ::testing::TempDir() + "terminal_pose_null_arm.yaml";
  {
    std::ifstream in(TERMINAL_POSE_YAML);
    std::ofstream out(path);
    std::string line;
    while (std::getline(in, line)) {
      out << (line.rfind("stowed_arm_protrusion:", 0) == 0 ? "stowed_arm_protrusion: null" : line)
          << "\n";
    }
  }
  EXPECT_THROW(LoadTerminalPoseSettings(path), std::runtime_error);
  EXPECT_THROW(LoadTerminalPoseSettings("/nonexistent/terminal_pose.yaml"), std::runtime_error);
}

// Clearance is separate from the acceptance limits: a stop 0.20 m short of
// the surface goal towards the victim is within the 0.25 m limit but puts
// the arm tips into the victim (gap 0.15 - 0.20 < 0).
TEST_F(TerminalPoseTest, ClearanceCheckIsIndependentOfTheAcceptanceLimits)
{
  const Eigen::Vector2d victim(10.0, 5.0);
  const TerminalPose goal = BowFacingPose(victim, 0.0, Standoff(settings_, VictimState::kSurface));
  const Eigen::Vector3d too_close(goal.position.x() + 0.20, goal.position.y(), 0.0);
  EXPECT_LT((too_close.head<2>() - goal.position).norm(), 0.25);  // inside acceptance
  const ClearanceReport close = CheckTerminalClearance(too_close, victim, settings_, nullptr);
  EXPECT_FALSE(close.ok);
  EXPECT_NEAR(close.victim_gap, -0.05, 1e-12);

  // Wall: a wall 0.3 m beside the hull (needs 0.5 m).
  OccupancyGrid2D grid;
  grid.width = 400;
  grid.height = 200;
  grid.data.assign(grid.width * grid.height, 0);
  const double wall_y = goal.position.y() + settings_.hull_half_width + 0.3;
  for (int x = 0; x < grid.width; ++x) {
    for (int y = static_cast<int>(wall_y / grid.resolution); y < grid.height; ++y) {
      grid.data[y * grid.width + x] = 100;
    }
  }
  const Eigen::Vector3d at_goal(goal.position.x(), goal.position.y(), goal.yaw);
  const ClearanceReport walled = CheckTerminalClearance(at_goal, victim, settings_, &grid);
  EXPECT_FALSE(walled.ok);
  EXPECT_NEAR(walled.wall_distance, 0.3, grid.resolution);
  EXPECT_NE(walled.reason.find("wall distance"), std::string::npos);
  EXPECT_TRUE(CheckTerminalClearance(at_goal, victim, settings_, nullptr).ok);
}

// Regression: the robot sweeping sideways in front of a stationary victim
// changes the bearing by much more than psi_reaim, but must not re-aim.
TEST_F(TerminalPoseTest, RobotLateralMotionAloneDoesNotReaim)
{
  TerminalPoseGenerator generator(settings_);
  const Eigen::Vector2d victim(10.0, 0.0);
  ASSERT_TRUE(generator.Update({7.5, 0.0}, victim, VictimState::kSurface).heading_held);
  double largest_bearing_change = 0.0;
  for (int k = 0; k <= 20; ++k) {
    const Eigen::Vector2d robot(7.5, -1.0 + 0.1 * k);  // 2 m sweep across
    const TerminalPose pose = generator.Update(robot, victim, VictimState::kSurface);
    EXPECT_TRUE(pose.heading_held);
    EXPECT_EQ(pose.yaw, 0.0) << "robot y " << robot.y();
    largest_bearing_change = std::max(
      largest_bearing_change, std::abs(std::atan2(victim.y() - robot.y(), victim.x() - robot.x())));
  }
  EXPECT_GT(largest_bearing_change, 0.35);  // well above psi_reaim = 0.15 rad
}

// Genuine victim motion still re-aims, measured from the position at the
// last lock or re-aim; small victim moves (below the shift) do not.
TEST_F(TerminalPoseTest, GenuineVictimMotionStillReaims)
{
  TerminalPoseGenerator generator(settings_);
  const Eigen::Vector2d robot(8.0, 0.0);
  ASSERT_TRUE(generator.Update(robot, {10.0, 0.0}, VictimState::kSurface).heading_held);
  // 0.25 m sideways: bearing off by 0.124 rad, below both thresholds.
  EXPECT_EQ(generator.Update(robot, {10.0, 0.25}, VictimState::kSurface).yaw, 0.0);
  // 0.6 m sideways from the lock position: both thresholds exceeded.
  const TerminalPose reaimed = generator.Update(robot, {10.0, 0.6}, VictimState::kSurface);
  EXPECT_NEAR(reaimed.yaw, std::atan2(0.6, 2.0), 1e-12);
  // The new lock position is (10, 0.6): another 0.2 m is not enough.
  EXPECT_EQ(generator.Update(robot, {10.0, 0.8}, VictimState::kSurface).yaw, reaimed.yaw);
}

TEST_F(TerminalPoseTest, OutlineOutsideTheMapFailsClearance)
{
  OccupancyGrid2D grid;  // 10 m x 10 m, all free
  grid.width = 200;
  grid.height = 200;
  grid.data.assign(grid.width * grid.height, 0);
  settings_.wall_clearance = 0.0;  // no walls on this map: only the map edge matters
  // Facing +x with base_link 0.3 m from the left edge: the stern sticks out.
  const ClearanceReport outside =
    CheckTerminalClearance({0.3, 5.0, 0.0}, {1.6, 5.0}, settings_, &grid);
  EXPECT_FALSE(outside.ok);
  EXPECT_NE(outside.reason.find("outside the map"), std::string::npos) << outside.reason;
  // The same pose 2 m in is fine.
  EXPECT_TRUE(CheckTerminalClearance({2.3, 5.0, 0.0}, {3.6, 5.0}, settings_, &grid).ok);
}

TEST_F(TerminalPoseTest, MalformedGridFailsClearance)
{
  OccupancyGrid2D grid;
  grid.width = 100;
  grid.height = 100;
  grid.data.assign(50, 0);  // too short for 100 x 100
  const Eigen::Vector3d pose(2.5, 2.5, 0.0);
  const Eigen::Vector2d victim(3.8, 2.5);
  ClearanceReport report = CheckTerminalClearance(pose, victim, settings_, &grid);
  EXPECT_FALSE(report.ok);
  EXPECT_NE(report.reason.find("invalid occupancy grid"), std::string::npos);
  grid.data.assign(grid.width * grid.height, 0);
  grid.resolution = 0.0;
  EXPECT_FALSE(CheckTerminalClearance(pose, victim, settings_, &grid).ok);
  grid.resolution = 0.05;
  grid.origin.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(CheckTerminalClearance(pose, victim, settings_, &grid).ok);
}

TEST_F(TerminalPoseTest, NegativeMinimumVictimGapIsRejected)
{
  TerminalPoseSettings bad = settings_;
  bad.min_victim_gap = -0.1;
  EXPECT_NE(ValidateTerminalPoseSettings(bad), "");
  EXPECT_THROW(TerminalPoseGenerator{bad}, std::invalid_argument);
  const ClearanceReport report =
    CheckTerminalClearance({0.0, 0.0, 0.0}, {1.6, 0.0}, bad, nullptr);
  EXPECT_FALSE(report.ok);
  EXPECT_NE(report.reason.find("min_victim_gap"), std::string::npos) << report.reason;
}

}  // namespace
}  // namespace rov_planning
