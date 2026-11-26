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

// Mission state machine (implementation step 10): both victim-state
// branches through RETURN, verified-arrival gate, SAFETY_STOP preemption.
// Times in s; d_hold = 2.0 m (design 6.5).

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "rov_mission/mission_state_machine.hpp"

namespace rov_mission
{
namespace
{

constexpr double kHoldDistance = 2.0;  // d_hold, m

std::vector<Phase> PhasesOf(const MissionStateMachine & machine)
{
  std::vector<Phase> phases = {Phase::kStandby};
  for (const Transition & t : machine.history()) {
    phases.push_back(t.to);
  }
  return phases;
}

// Drives a fresh machine from STANDBY to `target` (a motion phase).
void DriveTo(MissionStateMachine & m, Phase target, VictimState state, double * t)
{
  if (target == Phase::kStandby) {
    return;
  }
  ASSERT_TRUE(m.OnShoreAlarm(*t += 1.0, state).transitioned);
  if (target == Phase::kTransit) {
    return;
  }
  ASSERT_TRUE(m.OnDistanceToTerminal(*t += 1.0, 1.5).transitioned);
  if (target == Phase::kFinalApproach) {
    return;
  }
  ASSERT_TRUE(m.OnArrival(*t += 1.0, true, "error 0.02 m").transitioned);
  if (target == Phase::kDiveAndGrasp || target == Phase::kSurfaceHold) {
    return;
  }
  ASSERT_TRUE(m.OnRescueComplete(*t += 1.0, "simulated").transitioned);
}

TEST(MissionStateMachineTest, SubmergedVictimGoesThroughDiveAndGraspToReturn)
{
  std::vector<std::string> log;
  MissionStateMachine m(
    kHoldDistance, [&log](const Transition & t) {log.push_back(FormatTransition(t));});
  EXPECT_TRUE(m.OnShoreAlarm(10.0, VictimState::kSubmerged).transitioned);
  EXPECT_FALSE(m.OnDistanceToTerminal(12.0, 8.4).transitioned);  // still in transit
  EXPECT_TRUE(m.OnDistanceToTerminal(18.5, 1.9).transitioned);
  EXPECT_TRUE(m.OnArrival(21.0, true, "error 0.016 m / 0.003 rad, clearance ok").transitioned);
  EXPECT_EQ(m.phase(), Phase::kDiveAndGrasp);
  EXPECT_NE(m.history().back().reason.find("stub"), std::string::npos);
  EXPECT_TRUE(m.OnRescueComplete(40.0, "simulated").transitioned);
  EXPECT_EQ(m.phase(), Phase::kReturn);
  EXPECT_TRUE(m.OnShoreReached(70.0).transitioned);

  const std::vector<Phase> expected = {
    Phase::kStandby, Phase::kTransit, Phase::kFinalApproach, Phase::kDiveAndGrasp,
    Phase::kReturn, Phase::kStandby};
  EXPECT_EQ(PhasesOf(m), expected);
  // Each transition is logged with its time and reason.
  ASSERT_EQ(log.size(), 5u);
  EXPECT_EQ(log[0], "t = 10.000 s: STANDBY -> TRANSIT (shore alarm, victim submerged)");
  EXPECT_EQ(
    log[1], "t = 18.500 s: TRANSIT -> FINAL_APPROACH (1.90 m from the terminal point, "
    "within d_hold 2.00 m)");
  EXPECT_EQ(log[3], "t = 40.000 s: DIVE_AND_GRASP -> RETURN (victim lifted: simulated)");
  EXPECT_DOUBLE_EQ(m.history()[2].time, 21.0);
  EXPECT_FALSE(m.victim_state().has_value());  // mission over
}

TEST(MissionStateMachineTest, SurfaceVictimGoesThroughSurfaceHoldToReturn)
{
  MissionStateMachine m(kHoldDistance);
  EXPECT_TRUE(m.OnShoreAlarm(0.0, VictimState::kSurface).transitioned);
  EXPECT_TRUE(m.OnDistanceToTerminal(9.0, 2.0).transitioned);  // d_hold itself counts
  EXPECT_TRUE(m.OnArrival(12.0, true, "error 0.05 m").transitioned);
  EXPECT_EQ(m.phase(), Phase::kSurfaceHold);
  EXPECT_TRUE(m.OnRescueComplete(20.0, "victim holds on").transitioned);
  EXPECT_EQ(m.history().back().reason, "victim secured: victim holds on");
  EXPECT_TRUE(m.OnShoreReached(45.0).transitioned);
  const std::vector<Phase> expected = {
    Phase::kStandby, Phase::kTransit, Phase::kFinalApproach, Phase::kSurfaceHold,
    Phase::kReturn, Phase::kStandby};
  EXPECT_EQ(PhasesOf(m), expected);
}

TEST(MissionStateMachineTest, RescuePhaseOnlyAfterVerifiedArrival)
{
  MissionStateMachine m(kHoldDistance);
  EXPECT_FALSE(m.OnArrival(0.5, true, "x").transitioned);  // STANDBY
  m.OnShoreAlarm(1.0, VictimState::kSubmerged);
  // An arrival during TRANSIT does not skip FINAL_APPROACH.
  EXPECT_FALSE(m.OnArrival(2.0, true, "early").transitioned);
  EXPECT_FALSE(m.OnRescueComplete(2.5, "early").transitioned);
  m.OnDistanceToTerminal(3.0, 1.0);
  // Failed verification: stay in FINAL_APPROACH, with the reason.
  const EventResult refused = m.OnArrival(4.0, false, "error 0.40 m > 0.25 m");
  EXPECT_FALSE(refused.transitioned);
  EXPECT_NE(refused.reason.find("not verified: error 0.40 m"), std::string::npos);
  EXPECT_EQ(m.phase(), Phase::kFinalApproach);
  EXPECT_FALSE(m.OnRescueComplete(4.5, "x").transitioned);
  EXPECT_TRUE(m.OnArrival(5.0, true, "error 0.10 m").transitioned);
  EXPECT_EQ(m.phase(), Phase::kDiveAndGrasp);
}

TEST(MissionStateMachineTest, SafetyStopPreemptsEveryMotionPhaseAndHolds)
{
  const std::vector<std::pair<Phase, VictimState>> cases = {
    {Phase::kTransit, VictimState::kSubmerged},
    {Phase::kFinalApproach, VictimState::kSubmerged},
    {Phase::kDiveAndGrasp, VictimState::kSubmerged},
    {Phase::kSurfaceHold, VictimState::kSurface},
    {Phase::kReturn, VictimState::kSurface},
  };
  for (const auto & [phase, state] : cases) {
    SCOPED_TRACE(PhaseName(phase));
    MissionStateMachine m(kHoldDistance);
    double t = 0.0;
    DriveTo(m, phase, state, &t);
    ASSERT_EQ(m.phase(), phase);
    ASSERT_TRUE(IsMotionPhase(phase));
    const EventResult stop = m.OnSafetyStop(t += 0.1, "odometry timeout (simulated)");
    EXPECT_TRUE(stop.transitioned);
    EXPECT_EQ(m.phase(), Phase::kSafetyStop);
    EXPECT_EQ(m.preempted(), phase);
    EXPECT_NE(stop.reason.find("decelerate to 0 and hold"), std::string::npos);
    // While stopped, no mission event moves it on.
    EXPECT_FALSE(m.OnDistanceToTerminal(t += 0.1, 0.5).transitioned);
    EXPECT_FALSE(m.OnArrival(t += 0.1, true, "x").transitioned);
    EXPECT_FALSE(m.OnRescueComplete(t += 0.1, "x").transitioned);
    EXPECT_FALSE(m.OnShoreReached(t += 0.1).transitioned);
    EXPECT_FALSE(m.OnSafetyStop(t += 0.1, "again").transitioned);
    EXPECT_EQ(m.phase(), Phase::kSafetyStop);
    // Cleared: the preempted phase resumes.
    EXPECT_TRUE(m.OnSafetyCleared(t += 1.0, "odometry back").transitioned);
    EXPECT_EQ(m.phase(), phase);
    EXPECT_FALSE(m.preempted().has_value());
  }
}

TEST(MissionStateMachineTest, SafetyStopDoesNotApplyWithoutMotion)
{
  MissionStateMachine m(kHoldDistance);
  EXPECT_FALSE(IsMotionPhase(Phase::kStandby));
  EXPECT_FALSE(m.OnSafetyStop(1.0, "leak").transitioned);
  EXPECT_FALSE(m.OnSafetyCleared(2.0, "x").transitioned);
  EXPECT_EQ(m.phase(), Phase::kStandby);
  EXPECT_TRUE(m.history().empty());
}

TEST(MissionStateMachineTest, ResumeNeedsFreshStateAndAValidTrajectory)
{
  MissionStateMachine m(kHoldDistance);
  double t = 0.0;
  DriveTo(m, Phase::kTransit, VictimState::kSubmerged, &t);
  ASSERT_TRUE(m.OnSafetyStop(t += 1.0, "heartbeat lost").transitioned);
  const EventResult stale = m.OnSafetyCleared(t += 1.0, "heartbeat back", false, true);
  EXPECT_FALSE(stale.transitioned);
  EXPECT_NE(stale.reason.find("fresh state"), std::string::npos);
  EXPECT_FALSE(m.OnSafetyCleared(t += 0.1, "heartbeat back", true, false).transitioned);
  EXPECT_EQ(m.phase(), Phase::kSafetyStop);
  EXPECT_TRUE(m.OnSafetyCleared(t += 0.1, "heartbeat back", true, true).transitioned);
  EXPECT_EQ(m.phase(), Phase::kTransit);
}

TEST(MissionStateMachineTest, AbortKinds)
{
  // Hold: SAFETY_STOP, no resume, manual takeover -> STANDBY.
  MissionStateMachine hold(kHoldDistance);
  double t = 0.0;
  DriveTo(hold, Phase::kFinalApproach, VictimState::kSubmerged, &t);
  ASSERT_TRUE(hold.OnAbort(t += 1.0, AbortKind::kHold, "heartbeat lost 5 s").transitioned);
  EXPECT_EQ(hold.phase(), Phase::kSafetyStop);
  EXPECT_TRUE(hold.aborted());
  EXPECT_FALSE(hold.OnSafetyCleared(t += 1.0, "heartbeat back").transitioned);
  EXPECT_FALSE(hold.OnAbort(t += 0.1, AbortKind::kReturn, "battery").transitioned);
  EXPECT_TRUE(hold.OnManualTakeover(t += 1.0).transitioned);
  EXPECT_EQ(hold.phase(), Phase::kStandby);
  EXPECT_FALSE(hold.aborted());
  // Return: from any mission phase to RETURN, then the shore ends it.
  for (Phase from : {Phase::kTransit, Phase::kDiveAndGrasp, Phase::kSurfaceHold}) {
    MissionStateMachine m(kHoldDistance);
    double tt = 0.0;
    DriveTo(
      m, from, from == Phase::kSurfaceHold ? VictimState::kSurface : VictimState::kSubmerged,
      &tt);
    ASSERT_TRUE(m.OnAbort(tt += 1.0, AbortKind::kSurfaceReturn, "leak").transitioned);
    EXPECT_EQ(m.phase(), Phase::kReturn);
    EXPECT_FALSE(m.OnAbort(tt += 0.1, AbortKind::kReturn, "battery").transitioned);
    EXPECT_TRUE(m.OnShoreReached(tt += 10.0).transitioned);
  }
  // Also from SAFETY_STOP (not aborted), and never in STANDBY.
  MissionStateMachine m(kHoldDistance);
  EXPECT_FALSE(m.OnAbort(1.0, AbortKind::kReturn, "battery").transitioned);
  double tt = 1.0;
  DriveTo(m, Phase::kTransit, VictimState::kSubmerged, &tt);
  m.OnSafetyStop(tt += 1.0, "heartbeat lost");
  EXPECT_TRUE(m.OnAbort(tt += 1.0, AbortKind::kReturn, "battery").transitioned);
  EXPECT_EQ(m.phase(), Phase::kReturn);
}

TEST(MissionStateMachineTest, InvalidEventsChangeNothing)
{
  MissionStateMachine m(kHoldDistance);
  m.OnShoreAlarm(10.0, VictimState::kSurface);
  EXPECT_FALSE(m.OnShoreAlarm(11.0, VictimState::kSubmerged).transitioned);  // already active
  EXPECT_EQ(m.victim_state(), VictimState::kSurface);
  EXPECT_FALSE(m.OnDistanceToTerminal(9.0, 1.0).transitioned);  // back in time
  EXPECT_FALSE(m.OnDistanceToTerminal(12.0, -1.0).transitioned);
  EXPECT_EQ(m.phase(), Phase::kTransit);
  EXPECT_EQ(m.history().size(), 1u);
  EXPECT_THROW(MissionStateMachine(0.0), std::invalid_argument);
}

}  // namespace
}  // namespace rov_mission
