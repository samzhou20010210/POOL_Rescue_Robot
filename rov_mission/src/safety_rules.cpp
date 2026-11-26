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

#include "rov_mission/safety_rules.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace rov_mission
{

namespace
{

std::string Format(const char * format, double a, double b = 0.0, double c = 0.0)
{
  char text[200];
  std::snprintf(text, sizeof(text), format, a, b, c);
  return text;
}

}  // namespace

std::string ValidateSafetySettings(const SafetySettings & s)
{
  const double positive[] = {
    s.swimmer_slow_radius, s.swimmer_slow_speed, s.prediction_horizon, s.prediction_step,
    s.wall_speed, s.odometry_timeout, s.heartbeat_stop, s.heartbeat_abort,
    s.saturation_duration, s.slow_speed_factor, s.mission_timeout, s.sensor_max_age};
  for (const double value : positive) {
    if (!(value > 0.0) || !std::isfinite(value)) {
      return "safety settings: every time, speed, radius and factor must be finite and > 0";
    }
  }
  if (!(s.swimmer_clearance >= 0.0) || !(s.wall_distance >= 0.0) ||
    !(s.battery_minimum >= 0.0) || s.battery_minimum > 1.0 || !(s.future_tolerance >= 0.0) ||
    s.heartbeat_abort < s.heartbeat_stop || s.slow_speed_factor > 1.0)
  {
    return "safety settings: invalid clearance, wall distance, battery minimum, tolerance, "
           "heartbeat order or slow factor";
  }
  return "";
}

SafetyDecision EvaluateSafety(const SafetySettings & s, const SafetyInputs & in)
{
  SafetyDecision d;
  d.slow_speed_factor = s.slow_speed_factor;
  auto limit = [&d](double speed) {
      d.speed_limit = d.speed_limited ? std::min(d.speed_limit, speed) : speed;
      d.speed_limited = true;
    };
  auto trigger = [&d](const std::string & rule, const std::string & reason) {
      d.rules.push_back(rule);
      d.reasons.push_back(rule + ": " + reason);
    };
  auto abort = [&d](AbortKind kind) {
      auto rank = [](AbortKind k) {
          return k == AbortKind::kSurfaceReturn ? 3 : k == AbortKind::kHold ? 2 :
                 k == AbortKind::kReturn ? 1 : 0;
        };
      if (rank(kind) > rank(d.abort)) {
        d.abort = kind;
      }
    };
  auto fresh = [&](const std::optional<StampedValue> & value, const char * name) {
      if (!value) {
        return false;
      }
      if (value->stamp > in.now + s.future_tolerance ||
        in.now - value->stamp > s.sensor_max_age)
      {
        d.stale_inputs.push_back(name);
        return false;
      }
      return true;
    };

  // Swimmers.
  if (in.pose) {
    double nearest = std::numeric_limits<double>::infinity();
    bool predicted = false;
    double predicted_gap = 0.0, predicted_t = 0.0;
    for (const rov_planning::SwimmerTrack & track : in.swimmers) {
      const double age = std::max(0.0, in.now - track.stamp);
      nearest = std::min(
        nearest, rov_planning::FootprintDistance(
          *in.pose, in.footprint, track.position + track.velocity * age));
      for (double t = 0.0; t <= s.prediction_horizon + 1e-9 && !predicted;
        t += s.prediction_step)
      {
        Eigen::Vector3d robot = *in.pose;
        robot.head<2>() += in.velocity * t;
        const double gap = rov_planning::FootprintDistance(
          robot, in.footprint, track.position + track.velocity * (age + t));
        if (gap < s.swimmer_clearance) {
          predicted = true;
          predicted_gap = gap;
          predicted_t = t;
        }
      }
    }
    if (nearest <= s.swimmer_slow_radius) {
      limit(s.swimmer_slow_speed);
      trigger(
        "swimmer_nearby", Format(
          "a swimmer %.2f m from the hull (<= %.1f m): speed limit %.2f m/s", nearest,
          s.swimmer_slow_radius, s.swimmer_slow_speed));
    }
    if (predicted) {
      d.stop_hold = true;
      d.replan_now = true;
      trigger(
        "swimmer_predicted", Format(
          "a swimmer predicted %.2f m from the hull in %.1f s (< r_swim %.1f m): stop, hold, "
          "replan", predicted_gap, predicted_t, s.swimmer_clearance));
    }
  }
  // Walls.
  if (in.wall_distance && *in.wall_distance <= s.wall_distance) {
    limit(s.wall_speed);
    trigger(
      "near_wall", Format(
        "hull edge %.2f m from a wall (<= %.2f m): speed limit %.2f m/s", *in.wall_distance,
        s.wall_distance, s.wall_speed));
  }
  // Odometry.
  if (!in.odometry_time || in.now - *in.odometry_time > s.odometry_timeout) {
    d.zero_thrust = true;
    trigger(
      "odometry_timeout", in.odometry_time ?
      Format("no odometry for %.2f s (> %.2f s): all thruster forces 0",
      in.now - *in.odometry_time, s.odometry_timeout) :
      std::string("no odometry yet: all thruster forces 0"));
  }
  // Shore heartbeat (surface phases only).
  if (in.surface_phase) {
    const bool future = in.heartbeat_stamp && *in.heartbeat_stamp > in.now + s.future_tolerance;
    if (future) {
      d.stale_inputs.push_back("heartbeat (stamped in the future)");
    }
    const double age = in.heartbeat_stamp && !future ? in.now - *in.heartbeat_stamp :
      std::numeric_limits<double>::infinity();
    if (age > s.heartbeat_abort) {
      d.stop_hold = true;
      abort(AbortKind::kHold);
      trigger(
        "heartbeat_abort", std::isfinite(age) ?
        Format("shore heartbeat lost for %.1f s (> %.1f s): abort, hold, wait for manual "
        "takeover", age, s.heartbeat_abort) :
        std::string("no shore heartbeat: abort, hold, wait for manual takeover"));
    } else if (age > s.heartbeat_stop) {
      d.stop_hold = true;
      trigger(
        "heartbeat_lost", Format(
          "shore heartbeat lost for %.1f s (> %.1f s): decelerate along the trajectory, hold",
          age, s.heartbeat_stop));
    }
  }
  // Thrust saturation.
  if (in.saturated_since && in.now - *in.saturated_since >= s.saturation_duration) {
    d.replan_slow = true;
    trigger(
      "thrust_saturation", Format(
        "thrust saturated for %.1f s (>= %.1f s): replan at %.2f v_cruise",
        in.now - *in.saturated_since, s.saturation_duration, s.slow_speed_factor));
  }
  // Battery and leak.
  if (fresh(in.battery, "battery") && in.battery->value <= s.battery_minimum) {
    abort(AbortKind::kReturn);
    trigger(
      "low_battery", Format(
        "battery %.0f %% (<= %.0f %%): abort, return", 100.0 * in.battery->value,
        100.0 * s.battery_minimum));
  }
  if (fresh(in.leak, "leak") && in.leak->value > 0.5) {
    abort(AbortKind::kSurfaceReturn);
    trigger("leak", "leak sensor alarm: abort, surface, return");
  }
  // Mission timeout.
  if (in.mission_start && !in.arrived && in.now - *in.mission_start >= s.mission_timeout) {
    d.stop_hold = true;
    abort(AbortKind::kHold);
    trigger(
      "mission_timeout", Format(
        "%.0f s since the alarm without arrival (>= %.0f s): abort, hold",
        in.now - *in.mission_start, s.mission_timeout));
  }
  if (d.abort == AbortKind::kHold) {
    d.stop_hold = true;
  }
  return d;
}

}  // namespace rov_mission
