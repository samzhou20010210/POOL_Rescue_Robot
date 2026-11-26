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

// Mission state machine of the surface approach (design section 2,
// implementation step 10). No ROS: events are method calls with a timestamp
// (s, any monotonic clock; ROS time in a node), so every transition can be
// tested on its own.
//
//   STANDBY --shore alarm--> TRANSIT --within d_hold of the terminal point-->
//   FINAL_APPROACH --verified arrival--> DIVE_AND_GRASP (victim submerged)
//                                    \-> SURFACE_HOLD   (victim at surface)
//   DIVE_AND_GRASP --victim lifted-->  RETURN
//   SURFACE_HOLD   --victim secured--> RETURN
//   RETURN --shore reached--> STANDBY
//
//   SAFETY_STOP preempts every motion phase (all phases except STANDBY and
//   SAFETY_STOP): decelerate to 0 and hold. When the interruption is cleared
//   the preempted phase resumes, but only with fresh state (odometry) and a
//   valid trajectory (or hold) from the planner.
//   Aborts (design 5): hold -> SAFETY_STOP, aborted, no resume until a
//   manual takeover (-> STANDBY); return / surface-and-return -> RETURN from
//   any phase except STANDBY and RETURN (refused while aborted-and-holding).
//
// Rules:
//   - the rescue phase (DIVE_AND_GRASP / SURFACE_HOLD) is entered only from
//     FINAL_APPROACH and only on a VERIFIED arrival; an arrival that failed
//     verification is refused and the phase stays FINAL_APPROACH;
//   - DIVE_AND_GRASP is a stub: it holds position and only ends on a
//     (simulated) "victim lifted" event;
//   - an event that does not apply to the current phase changes nothing and
//     is answered with the reason; events must not go back in time;
//   - every transition is recorded (time, from, to, reason) and passed to
//     the optional logger.
// Assumption (not in the design): RETURN ends with a "shore reached" event
// that brings the mission back to STANDBY.

#ifndef ROV_MISSION__MISSION_STATE_MACHINE_HPP_
#define ROV_MISSION__MISSION_STATE_MACHINE_HPP_

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace rov_mission
{

enum class Phase
{
  kStandby,
  kTransit,
  kFinalApproach,
  kDiveAndGrasp,
  kSurfaceHold,
  kReturn,
  kSafetyStop,
};

// "STANDBY", "TRANSIT", ... as in the design.
const char * PhaseName(Phase phase);

// Motion phases are the ones SAFETY_STOP may preempt.
bool IsMotionPhase(Phase phase);

enum class AbortKind { kHold, kReturn, kSurfaceReturn };

const char * AbortKindName(AbortKind kind);

enum class VictimState { kSurface, kSubmerged };

const char * VictimStateName(VictimState state);

struct Transition
{
  double time = 0.0;  // s
  Phase from = Phase::kStandby;
  Phase to = Phase::kStandby;
  std::string reason;
};

// "t = 12.300 s: TRANSIT -> FINAL_APPROACH (reason)".
std::string FormatTransition(const Transition & transition);

// Answer to one event.
struct EventResult
{
  bool transitioned = false;
  std::string reason;  // why it did or did not change the phase
};

class MissionStateMachine
{
public:
  using Logger = std::function<void (const Transition &)>;

  // heading_hold_distance: d_hold (m, > 0), the FINAL_APPROACH zone around
  // the terminal point (terminal_pose.yaml, design 6.5). Throws
  // std::invalid_argument otherwise.
  explicit MissionStateMachine(double heading_hold_distance, Logger logger = nullptr);

  // ---- Events ---------------------------------------------------------------

  // Shore alarm with the victim's state: STANDBY -> TRANSIT.
  EventResult OnShoreAlarm(double time, VictimState victim_state);

  // Planar distance (m) from the robot to the terminal point:
  // TRANSIT -> FINAL_APPROACH once it is <= d_hold.
  EventResult OnDistanceToTerminal(double time, double distance);

  // Arrival check at the terminal pose (`verified` true only if the reached
  // pose passed the acceptance and clearance checks; `detail` says why).
  // FINAL_APPROACH -> DIVE_AND_GRASP / SURFACE_HOLD only if verified.
  EventResult OnArrival(double time, bool verified, const std::string & detail);

  // The rescue finished: victim lifted (DIVE_AND_GRASP) or secured
  // (SURFACE_HOLD) -> RETURN.
  EventResult OnRescueComplete(double time, const std::string & detail);

  // RETURN -> STANDBY.
  EventResult OnShoreReached(double time);

  // Safety interruption: any motion phase -> SAFETY_STOP.
  EventResult OnSafetyStop(double time, const std::string & cause);

  // The interruption is over: SAFETY_STOP -> the preempted phase, only if
  // the state is fresh and the planner has a valid trajectory (or hold).
  EventResult OnSafetyCleared(
    double time, const std::string & detail, bool state_fresh = true,
    bool trajectory_valid = true);

  // Abort the mission (see the header comment).
  EventResult OnAbort(double time, AbortKind kind, const std::string & cause);

  // Operator takes over after an abort-and-hold: SAFETY_STOP -> STANDBY.
  EventResult OnManualTakeover(double time);

  // ---- State ----------------------------------------------------------------

  Phase phase() const {return phase_;}
  std::optional<VictimState> victim_state() const {return victim_state_;}
  // The phase SAFETY_STOP preempted (only while in SAFETY_STOP).
  std::optional<Phase> preempted() const {return preempted_;}
  const std::vector<Transition> & history() const {return history_;}
  bool aborted() const {return aborted_;}

private:
  // Refuses events that go back in time; empty string if `time` is usable.
  std::string CheckTime(double time) const;
  EventResult Refuse(const std::string & event, const std::string & why) const;
  EventResult Go(double time, Phase to, const std::string & reason);

  double heading_hold_distance_;
  Logger logger_;
  Phase phase_ = Phase::kStandby;
  std::optional<VictimState> victim_state_;
  std::optional<Phase> preempted_;
  bool aborted_ = false;
  double last_time_ = -1e300;  // s, time of the last accepted event
  std::vector<Transition> history_;
};

}  // namespace rov_mission

#endif  // ROV_MISSION__MISSION_STATE_MACHINE_HPP_
