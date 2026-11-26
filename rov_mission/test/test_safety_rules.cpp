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

// Safety rules (design 5, step 10): each rule triggered alone gives its
// specified action; simultaneous rules combine; stale inputs are ignored.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "rov_mission/safety_rules.hpp"

namespace rov_mission
{
namespace
{

// A healthy snapshot: fresh odometry and heartbeat, surface phase, nothing
// near, battery full, dry.
SafetyInputs Healthy()
{
  SafetyInputs in;
  in.now = 100.0;
  in.odometry_time = 99.95;
  in.pose = Eigen::Vector3d(0.0, 0.0, 0.0);
  in.velocity = Eigen::Vector2d(1.0, 0.0);
  in.footprint = rov_planning::Footprint{0.6, 0.5, 0.52};
  in.heartbeat_stamp = 99.9;
  in.surface_phase = true;
  in.wall_distance = 4.0;
  in.battery = StampedValue{99.5, 0.9};
  in.leak = StampedValue{99.5, 0.0};
  in.mission_start = 90.0;
  return in;
}

bool Has(const SafetyDecision & d, const std::string & rule)
{
  return std::find(d.rules.begin(), d.rules.end(), rule) != d.rules.end();
}

rov_planning::SwimmerTrack Swimmer(double x, double y, double vx, double vy, double stamp)
{
  rov_planning::SwimmerTrack t;
  t.stamp = stamp;
  t.position = Eigen::Vector2d(x, y);
  t.velocity = Eigen::Vector2d(vx, vy);
  return t;
}

const SafetySettings kSettings;

TEST(SafetyRulesTest, HealthyGivesNoAction)
{
  const SafetyDecision d = EvaluateSafety(kSettings, Healthy());
  EXPECT_TRUE(d.rules.empty());
  EXPECT_FALSE(d.speed_limited || d.zero_thrust || d.stop_hold || d.replan_now || d.replan_slow);
  EXPECT_EQ(d.abort, AbortKind::kNone);
  EXPECT_TRUE(ValidateSafetySettings(kSettings).empty());
}

TEST(SafetyRulesTest, SwimmerNearbyLimitsTheSpeed)
{
  SafetyInputs in = Healthy();
  in.velocity.setZero();
  in.swimmers = {Swimmer(0.0, 3.3, 0.0, 0.0, 100.0)};  // 2.78 m from the hull side
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "swimmer_nearby"));
  EXPECT_TRUE(d.speed_limited);
  EXPECT_DOUBLE_EQ(d.speed_limit, 0.5);
  EXPECT_FALSE(d.stop_hold);
}

TEST(SafetyRulesTest, PredictedSwimmerStopsHoldsAndReplans)
{
  SafetyInputs in = Healthy();
  // 5 m ahead, swimming towards the robot: within 1 m of the hull in < 2 s.
  in.swimmers = {Swimmer(5.0, 0.0, -1.5, 0.0, 100.0)};
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "swimmer_predicted"));
  EXPECT_TRUE(d.stop_hold);
  EXPECT_TRUE(d.replan_now);
}

TEST(SafetyRulesTest, NearWallLimitsTheSpeed)
{
  SafetyInputs in = Healthy();
  in.wall_distance = 0.4;
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "near_wall"));
  EXPECT_DOUBLE_EQ(d.speed_limit, 0.3);
}

TEST(SafetyRulesTest, OdometryTimeoutZeroesTheThrust)
{
  SafetyInputs in = Healthy();
  in.odometry_time = 99.7;  // 0.3 s
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "odometry_timeout"));
  EXPECT_TRUE(d.zero_thrust);
}

TEST(SafetyRulesTest, HeartbeatLossStopsThenAbortsHolding)
{
  SafetyInputs in = Healthy();
  in.heartbeat_stamp = 98.5;  // 1.5 s
  SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "heartbeat_lost"));
  EXPECT_TRUE(d.stop_hold);
  EXPECT_EQ(d.abort, AbortKind::kNone);
  in.heartbeat_stamp = 94.0;  // 6 s
  d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "heartbeat_abort"));
  EXPECT_EQ(d.abort, AbortKind::kHold);
  // Not a surface phase (STANDBY, DIVE_AND_GRASP): no heartbeat rule.
  in.surface_phase = false;
  EXPECT_FALSE(Has(EvaluateSafety(kSettings, in), "heartbeat_abort"));
}

TEST(SafetyRulesTest, SustainedSaturationReplansSlower)
{
  SafetyInputs in = Healthy();
  in.saturated_since = 98.5;  // 1.5 s: not yet
  EXPECT_FALSE(EvaluateSafety(kSettings, in).replan_slow);
  in.saturated_since = 97.9;  // 2.1 s
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "thrust_saturation"));
  EXPECT_TRUE(d.replan_slow);
  EXPECT_DOUBLE_EQ(d.slow_speed_factor, 0.7);
}

TEST(SafetyRulesTest, LowBatteryAbortsAndReturns)
{
  SafetyInputs in = Healthy();
  in.battery = StampedValue{99.5, 0.19};
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "low_battery"));
  EXPECT_EQ(d.abort, AbortKind::kReturn);
  EXPECT_FALSE(d.stop_hold);
}

TEST(SafetyRulesTest, LeakAbortsSurfacesAndReturns)
{
  SafetyInputs in = Healthy();
  in.leak = StampedValue{99.5, 1.0};
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "leak"));
  EXPECT_EQ(d.abort, AbortKind::kSurfaceReturn);
}

TEST(SafetyRulesTest, MissionTimeoutAbortsHolding)
{
  SafetyInputs in = Healthy();
  in.mission_start = -21.0;  // 121 s ago
  SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_TRUE(Has(d, "mission_timeout"));
  EXPECT_EQ(d.abort, AbortKind::kHold);
  EXPECT_TRUE(d.stop_hold);
  in.arrived = true;  // arrival stops the clock
  EXPECT_FALSE(Has(EvaluateSafety(kSettings, in), "mission_timeout"));
}

TEST(SafetyRulesTest, SimultaneousRulesCombine)
{
  SafetyInputs in = Healthy();
  in.velocity.setZero();
  in.swimmers = {Swimmer(0.0, 3.3, 0.0, 0.0, 100.0)};  // 0.5 m/s
  in.wall_distance = 0.4;                              // 0.3 m/s
  in.battery = StampedValue{99.5, 0.1};                // return
  in.leak = StampedValue{99.5, 1.0};                   // surface + return (wins)
  in.odometry_time = 99.0;                             // zero thrust
  in.heartbeat_stamp = 98.0;                           // stop and hold
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_EQ(d.rules.size(), 6u);
  EXPECT_DOUBLE_EQ(d.speed_limit, 0.3);  // the lowest limit
  EXPECT_TRUE(d.zero_thrust);
  EXPECT_TRUE(d.stop_hold);
  EXPECT_EQ(d.abort, AbortKind::kSurfaceReturn);
  // Hold beats return.
  in.leak = StampedValue{99.5, 0.0};
  in.heartbeat_stamp = 90.0;
  EXPECT_EQ(EvaluateSafety(kSettings, in).abort, AbortKind::kHold);
}

TEST(SafetyRulesTest, StaleOrFutureReadingsAreIgnored)
{
  SafetyInputs in = Healthy();
  in.battery = StampedValue{90.0, 0.1};  // 10 s old
  in.leak = StampedValue{105.0, 1.0};    // 5 s in the future
  const SafetyDecision d = EvaluateSafety(kSettings, in);
  EXPECT_FALSE(Has(d, "low_battery"));
  EXPECT_FALSE(Has(d, "leak"));
  EXPECT_EQ(d.stale_inputs.size(), 2u);
  // A heartbeat stamped in the future is not a fresh heartbeat.
  in = Healthy();
  in.heartbeat_stamp = 110.0;
  EXPECT_TRUE(Has(EvaluateSafety(kSettings, in), "heartbeat_abort"));
}

}  // namespace
}  // namespace rov_mission
