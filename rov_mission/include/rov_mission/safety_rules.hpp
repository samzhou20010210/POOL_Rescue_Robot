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

// Safety rules of the surface approach (design 5, thresholds design 6.7),
// evaluated on one snapshot of the inputs. No ROS. The safety monitor node
// fills SafetyInputs at a fixed rate and publishes the SafetyDecision.
//
// | Rule                    | Trigger                                  | Action                       |
// |-------------------------|------------------------------------------|------------------------------|
// | swimmer_nearby          | hull within r_slow (3.0 m) of a swimmer  | speed limit v_slow 0.5 m/s   |
// | swimmer_predicted       | predicted swimmer within r_swim (1.0 m)  | stop and hold, replan now    |
// |                         | of the hull within t_pred (2.0 s)        |                              |
// | near_wall               | hull edge within 0.5 m of a wall         | speed limit v_wall 0.3 m/s   |
// | odometry_timeout        | no odometry for t_odom (0.2 s)           | all thruster forces 0        |
// | heartbeat_lost          | shore heartbeat older than 1.0 s, during | decelerate along the         |
// |                         | a surface phase                          | trajectory, hold             |
// | heartbeat_abort         | ... older than 5.0 s                     | abort, hold, manual takeover |
// | thrust_saturation       | saturated for t_sat (2.0 s) without a    | replan at 0.7 v_cruise       |
// |                         | break                                    |                              |
// | low_battery             | battery <= 20 %                          | abort, return                |
// | leak                    | leak sensor wet                          | abort, surface, return       |
// | mission_timeout         | 120 s since the alarm without arrival    | abort, hold                  |
//
// Simultaneous rules combine: the lowest speed limit; zero thrust, stop and
// replan flags OR'ed; one abort, by priority surface-and-return (leak) >
// hold (heartbeat 5 s, mission timeout) > return (battery). Battery and
// leak readings older than sensor_max_age, or stamped in the future, are
// ignored (listed in stale_inputs); a heartbeat stamped in the future does
// not count as fresh. "Surface phase" = a mission phase other than STANDBY
// and DIVE_AND_GRASP (the vehicle is at the surface).

#ifndef ROV_MISSION__SAFETY_RULES_HPP_
#define ROV_MISSION__SAFETY_RULES_HPP_

#include <optional>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "rov_planning/swimmer_prediction.hpp"

namespace rov_mission
{

struct SafetySettings
{
  double swimmer_slow_radius = 3.0;   // r_slow, m
  double swimmer_slow_speed = 0.5;    // v_slow, m/s
  double swimmer_clearance = 1.0;     // r_swim, m
  double prediction_horizon = 2.0;    // t_pred, s
  double prediction_step = 0.1;       // s
  double wall_distance = 0.5;         // m, hull edge to wall
  double wall_speed = 0.3;            // v_wall, m/s
  double odometry_timeout = 0.2;      // t_odom, s
  double heartbeat_stop = 1.0;        // t_hb_stop, s
  double heartbeat_abort = 5.0;       // t_hb_abort, s
  double saturation_duration = 2.0;   // t_sat, s
  double slow_speed_factor = 0.7;     // of v_cruise
  double battery_minimum = 0.20;      // SOC_min, fraction
  double mission_timeout = 120.0;     // t_mission, s
  double sensor_max_age = 3.0;        // s: older battery / leak readings are ignored
  double future_tolerance = 0.5;      // s: stamps further ahead are ignored
};

// Empty if usable, otherwise the reason.
std::string ValidateSafetySettings(const SafetySettings & settings);

struct StampedValue
{
  double stamp = 0.0;   // s, same clock as SafetyInputs::now
  double value = 0.0;
};

struct SafetyInputs
{
  double now = 0.0;                                   // s
  std::optional<double> odometry_time;                // receive time of the last odometry
  std::optional<Eigen::Vector3d> pose;                // map frame (x, y, yaw)
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();  // map frame, m/s
  rov_planning::Footprint footprint;                  // hull outline
  std::optional<double> heartbeat_stamp;              // stamp of the last heartbeat
  bool surface_phase = false;                         // heartbeat rules apply
  std::vector<rov_planning::SwimmerTrack> swimmers;   // current (not stale) tracks
  std::optional<double> wall_distance;                // hull edge to nearest wall, m
  std::optional<double> saturated_since;              // start of the current saturation
  std::optional<StampedValue> battery;                // fraction 0..1
  std::optional<StampedValue> leak;                   // 1 = wet
  std::optional<double> mission_start;                // time of the alarm (TRANSIT entry)
  bool arrived = false;                               // rescue phase reached
};

enum class AbortKind { kNone, kHold, kReturn, kSurfaceReturn };

struct SafetyDecision
{
  bool speed_limited = false;
  double speed_limit = 0.0;      // m/s
  bool zero_thrust = false;
  bool stop_hold = false;
  bool replan_now = false;
  bool replan_slow = false;
  double slow_speed_factor = 0.7;
  AbortKind abort = AbortKind::kNone;
  std::vector<std::string> rules;    // triggered rule names
  std::vector<std::string> reasons;  // one line per triggered rule
  std::vector<std::string> stale_inputs;
};

SafetyDecision EvaluateSafety(const SafetySettings & settings, const SafetyInputs & inputs);

}  // namespace rov_mission

#endif  // ROV_MISSION__SAFETY_RULES_HPP_
