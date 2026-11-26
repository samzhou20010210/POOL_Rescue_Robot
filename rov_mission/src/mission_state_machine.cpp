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

#include "rov_mission/mission_state_machine.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

namespace rov_mission
{

const char * PhaseName(Phase phase)
{
  switch (phase) {
    case Phase::kStandby: return "STANDBY";
    case Phase::kTransit: return "TRANSIT";
    case Phase::kFinalApproach: return "FINAL_APPROACH";
    case Phase::kDiveAndGrasp: return "DIVE_AND_GRASP";
    case Phase::kSurfaceHold: return "SURFACE_HOLD";
    case Phase::kReturn: return "RETURN";
    case Phase::kSafetyStop: return "SAFETY_STOP";
  }
  return "UNKNOWN";
}

bool IsMotionPhase(Phase phase)
{
  return phase != Phase::kStandby && phase != Phase::kSafetyStop;
}

const char * AbortKindName(AbortKind kind)
{
  switch (kind) {
    case AbortKind::kHold: return "hold and wait for manual takeover";
    case AbortKind::kReturn: return "return to the shore";
    case AbortKind::kSurfaceReturn: return "surface, then return to the shore";
  }
  return "unknown";
}

const char * VictimStateName(VictimState state)
{
  return state == VictimState::kSurface ? "surface" : "submerged";
}

std::string FormatTransition(const Transition & transition)
{
  char head[96];
  std::snprintf(
    head, sizeof(head), "t = %.3f s: %s -> %s (", transition.time, PhaseName(transition.from),
    PhaseName(transition.to));
  return head + transition.reason + ")";
}

MissionStateMachine::MissionStateMachine(double heading_hold_distance, Logger logger)
: heading_hold_distance_(heading_hold_distance), logger_(std::move(logger))
{
  if (!(heading_hold_distance_ > 0.0) || !std::isfinite(heading_hold_distance_)) {
    throw std::invalid_argument("heading_hold_distance (d_hold) must be finite and > 0");
  }
}

EventResult MissionStateMachine::OnShoreAlarm(double time, VictimState victim_state)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("shore alarm", bad);
  }
  if (phase_ != Phase::kStandby) {
    return Refuse("shore alarm", "a mission is already active");
  }
  victim_state_ = victim_state;
  return Go(
    time, Phase::kTransit,
    std::string("shore alarm, victim ") + VictimStateName(victim_state));
}

EventResult MissionStateMachine::OnDistanceToTerminal(double time, double distance)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("distance update", bad);
  }
  if (!(distance >= 0.0) || !std::isfinite(distance)) {
    return Refuse("distance update", "distance must be finite and >= 0");
  }
  if (phase_ != Phase::kTransit) {
    return Refuse("distance update", "only used in TRANSIT");
  }
  if (distance > heading_hold_distance_) {
    return Refuse("distance update", "still outside d_hold");
  }
  char reason[96];
  std::snprintf(
    reason, sizeof(reason), "%.2f m from the terminal point, within d_hold %.2f m", distance,
    heading_hold_distance_);
  return Go(time, Phase::kFinalApproach, reason);
}

EventResult MissionStateMachine::OnArrival(
  double time, bool verified, const std::string & detail)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("arrival", bad);
  }
  if (phase_ != Phase::kFinalApproach) {
    return Refuse("arrival", "the rescue phase is entered only from FINAL_APPROACH");
  }
  if (!verified) {
    return Refuse("arrival", "arrival not verified: " + detail);
  }
  if (*victim_state_ == VictimState::kSubmerged) {
    return Go(
      time, Phase::kDiveAndGrasp,
      "verified arrival (" + detail + "), victim submerged; dive and grasp is a stub: hold");
  }
  return Go(time, Phase::kSurfaceHold, "verified arrival (" + detail + "), victim at surface");
}

EventResult MissionStateMachine::OnRescueComplete(double time, const std::string & detail)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("rescue complete", bad);
  }
  if (phase_ == Phase::kDiveAndGrasp) {
    return Go(time, Phase::kReturn, "victim lifted: " + detail);
  }
  if (phase_ == Phase::kSurfaceHold) {
    return Go(time, Phase::kReturn, "victim secured: " + detail);
  }
  return Refuse("rescue complete", "not in DIVE_AND_GRASP or SURFACE_HOLD");
}

EventResult MissionStateMachine::OnShoreReached(double time)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("shore reached", bad);
  }
  if (phase_ != Phase::kReturn) {
    return Refuse("shore reached", "only ends RETURN");
  }
  EventResult result = Go(time, Phase::kStandby, "shore reached, mission complete");
  victim_state_.reset();
  return result;
}

EventResult MissionStateMachine::OnSafetyStop(double time, const std::string & cause)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("safety stop", bad);
  }
  if (!IsMotionPhase(phase_)) {
    return Refuse("safety stop", std::string("no motion to stop in ") + PhaseName(phase_));
  }
  preempted_ = phase_;
  return Go(
    time, Phase::kSafetyStop,
    cause + "; decelerate to 0 and hold (preempted " + PhaseName(phase_) + ")");
}

EventResult MissionStateMachine::OnSafetyCleared(
  double time, const std::string & detail, bool state_fresh, bool trajectory_valid)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("safety cleared", bad);
  }
  if (phase_ != Phase::kSafetyStop) {
    return Refuse("safety cleared", "not in SAFETY_STOP");
  }
  if (aborted_) {
    return Refuse("safety cleared", "the mission was aborted: manual takeover required");
  }
  if (!state_fresh) {
    return Refuse("safety cleared", "no fresh state (odometry) yet");
  }
  if (!trajectory_valid) {
    return Refuse("safety cleared", "no valid trajectory from the planner yet");
  }
  const Phase resume = *preempted_;
  preempted_.reset();
  return Go(
    time, resume, "safety stop cleared (" + detail + "), resume " + PhaseName(resume));
}

EventResult MissionStateMachine::OnAbort(double time, AbortKind kind, const std::string & cause)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("abort", bad);
  }
  if (phase_ == Phase::kStandby) {
    return Refuse("abort", "no mission to abort");
  }
  const std::string reason = "abort (" + cause + "): " + AbortKindName(kind);
  if (kind == AbortKind::kHold) {
    if (aborted_) {
      return Refuse("abort", "already aborted and holding");
    }
    if (phase_ != Phase::kSafetyStop) {
      preempted_ = phase_;
    }
    aborted_ = true;
    return Go(time, Phase::kSafetyStop, reason);  // logged even from SAFETY_STOP
  }
  if (aborted_) {
    return Refuse("abort", "aborted and holding: manual takeover required");
  }
  if (phase_ == Phase::kReturn) {
    return Refuse("abort", "already returning");
  }
  preempted_.reset();
  return Go(time, Phase::kReturn, reason);
}

EventResult MissionStateMachine::OnManualTakeover(double time)
{
  if (const std::string bad = CheckTime(time); !bad.empty()) {
    return Refuse("manual takeover", bad);
  }
  if (!aborted_) {
    return Refuse("manual takeover", "only after an abort-and-hold");
  }
  aborted_ = false;
  preempted_.reset();
  EventResult result = Go(time, Phase::kStandby, "manual takeover");
  victim_state_.reset();
  return result;
}

std::string MissionStateMachine::CheckTime(double time) const
{
  if (!std::isfinite(time)) {
    return "timestamp is not finite";
  }
  if (time < last_time_) {
    return "timestamp is earlier than the last accepted event";
  }
  return "";
}

EventResult MissionStateMachine::Refuse(const std::string & event, const std::string & why) const
{
  return {false, event + " ignored in " + PhaseName(phase_) + ": " + why};
}

EventResult MissionStateMachine::Go(double time, Phase to, const std::string & reason)
{
  const Transition transition{time, phase_, to, reason};
  phase_ = to;
  last_time_ = time;
  history_.push_back(transition);
  if (logger_) {
    logger_(transition);
  }
  return {true, reason};
}

}  // namespace rov_mission
