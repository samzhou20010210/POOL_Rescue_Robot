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

#include "rov_planning/reference_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace rov_planning
{

namespace
{

double Unwrap(double previous, double angle)
{
  return previous + std::remainder(angle - previous, 2.0 * M_PI);
}

}  // namespace

RefTrajectory SamplePolynomial(
  const PolynomialTrajectory & trajectory, double t_begin, double t_end, double dt,
  double offset)
{
  if (!(dt > 0.0) || t_end < t_begin) {
    throw std::invalid_argument("SamplePolynomial: need dt > 0 and t_end >= t_begin");
  }
  RefTrajectory samples;
  for (double t = t_begin; ; t += dt) {
    const double clamped = std::min(t, t_end);
    Eigen::Vector3d pos, vel, acc;
    trajectory.Evaluate(clamped, &pos, &vel, &acc);
    RefPoint point;
    point.t = clamped - t_begin + offset;
    point.position = pos.head<2>();
    point.velocity = vel.head<2>();
    point.acceleration = acc.head<2>();
    point.yaw = pos(2);
    point.yaw_rate = vel(2);
    samples.push_back(point);
    if (clamped >= t_end) {
      break;
    }
  }
  return samples;
}

RefPoint InterpolateRef(const RefTrajectory & reference, double t)
{
  if (reference.empty()) {
    throw std::invalid_argument("InterpolateRef: empty reference");
  }
  if (t <= reference.front().t) {
    RefPoint p = reference.front();
    p.t = t;
    return p;
  }
  if (t >= reference.back().t) {
    RefPoint p = reference.back();
    p.t = t;
    return p;
  }
  const auto upper = std::upper_bound(
    reference.begin(), reference.end(), t,
    [](double value, const RefPoint & point) {return value < point.t;});
  const RefPoint & b = *upper;
  const RefPoint & a = *(upper - 1);
  const double w = (t - a.t) / (b.t - a.t);
  RefPoint p;
  p.t = t;
  p.position = (1.0 - w) * a.position + w * b.position;
  p.velocity = (1.0 - w) * a.velocity + w * b.velocity;
  p.acceleration = (1.0 - w) * a.acceleration + w * b.acceleration;
  p.yaw = a.yaw + w * std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
  p.yaw_rate = (1.0 - w) * a.yaw_rate + w * b.yaw_rate;
  return p;
}

double MaxSpeed(const RefTrajectory & reference)
{
  double speed = 0.0;
  for (const RefPoint & p : reference) {
    speed = std::max(speed, p.velocity.norm());
  }
  return speed;
}

RefTrajectory HoldAt(const Eigen::Vector3d & pose, double dt)
{
  RefPoint a;
  a.position = pose.head<2>();
  a.yaw = pose(2);
  RefPoint b = a;
  b.t = dt;
  return {a, b};
}

RefTrajectory BrakeAlong(
  const RefTrajectory & reference, double t0, double deceleration, double dt)
{
  if (!(deceleration > 0.0) || !(dt > 0.0)) {
    throw std::invalid_argument("BrakeAlong: deceleration and dt must be > 0");
  }
  const RefPoint start = InterpolateRef(reference, t0);
  const double v0 = start.velocity.norm();
  if (v0 < 1e-3) {
    return HoldAt(Eigen::Vector3d(start.position.x(), start.position.y(), start.yaw), dt);
  }
  // Remaining path up to where the reference itself stops: its first sample
  // at rest or the first reversal of direction (a cusp, e.g. a replan that
  // turns back), else its end. Past a cusp the path doubles back.
  std::vector<Eigen::Vector2d> points = {start.position};
  std::vector<double> yaws = {start.yaw};
  std::vector<double> arc = {0.0};
  std::vector<double> ref_speed = {v0};
  bool ends_at_rest = false;
  Eigen::Vector2d previous_velocity = start.velocity;
  for (const RefPoint & p : reference) {
    if (p.t <= t0) {
      continue;
    }
    const bool stop = p.velocity.norm() < 1e-3 || p.velocity.dot(previous_velocity) < 0.0;
    previous_velocity = p.velocity;
    const double step = (p.position - points.back()).norm();
    if (step >= 1e-9) {
      points.push_back(p.position);
      yaws.push_back(Unwrap(yaws.back(), p.yaw));
      arc.push_back(arc.back() + step);
      ref_speed.push_back(stop ? 0.0 : p.velocity.norm());
    }
    if (stop) {
      ends_at_rest = true;
      break;
    }
  }
  if (!ends_at_rest && !reference.empty() && reference.back().velocity.norm() < 1e-3) {
    ref_speed.back() = 0.0;
    ends_at_rest = true;
  }
  auto at_arc = [&](double s, Eigen::Vector2d * position, Eigen::Vector2d * tangent,
    double * yaw) {
      if (points.size() == 1) {
        *position = points.front();
        *tangent = start.velocity / v0;
        *yaw = yaws.front();
        return;
      }
      s = std::clamp(s, 0.0, arc.back());
      std::size_t i = std::upper_bound(arc.begin(), arc.end(), s) - arc.begin();
      i = std::clamp<std::size_t>(i, 1, arc.size() - 1);
      const double w = (s - arc[i - 1]) / (arc[i] - arc[i - 1]);
      *position = (1.0 - w) * points[i - 1] + w * points[i];
      *tangent = (points[i] - points[i - 1]).normalized();
      *yaw = (1.0 - w) * yaws[i - 1] + w * yaws[i];
    };
  auto ref_speed_at = [&](double s) {
      if (points.size() == 1) {
        return v0;
      }
      std::size_t i = std::upper_bound(arc.begin(), arc.end(), s) - arc.begin();
      i = std::clamp<std::size_t>(i, 1, arc.size() - 1);
      const double w = std::clamp((s - arc[i - 1]) / (arc[i] - arc[i - 1]), 0.0, 1.0);
      return (1.0 - w) * ref_speed[i - 1] + w * ref_speed[i];
    };

  // Speed profile over arc length: v(s) = min(sqrt(v0^2 - 2 a s), v_ref(s)),
  // never increasing. The brake never decelerates more gently than a_b, and is
  // never faster than the reference at the same point of the path (the
  // reference is feasible on that geometry: braking at a_b through a tight
  // turn the reference takes slowly would need more lateral thrust).
  // Constant deceleration between grid points (exact for a constant a).
  const double brake_distance = 0.5 * v0 * v0 / deceleration;
  const double path_length = points.size() == 1 ? brake_distance : arc.back();
  const double s_end = std::min(brake_distance, path_length);
  constexpr double kArcStep = 0.005;  // m
  std::vector<double> grid_s = {0.0};
  std::vector<double> grid_v = {v0};
  while (grid_s.back() < s_end) {
    const double s = std::min(grid_s.back() + kArcStep, s_end);
    double v = std::sqrt(std::max(0.0, v0 * v0 - 2.0 * deceleration * s));
    if (s >= brake_distance) {
      v = 0.0;
    }
    v = std::min({v, ref_speed_at(s), grid_v.back()});
    if (s >= s_end && (s >= brake_distance || ends_at_rest)) {
      v = 0.0;
    }
    grid_s.push_back(s);
    grid_v.push_back(v);
    if (v < 1e-6) {
      break;
    }
  }
  // Times at the grid points (trapezoid: exact for constant deceleration).
  std::vector<double> grid_t = {0.0};
  for (std::size_t i = 1; i < grid_s.size(); ++i) {
    const double mean = 0.5 * (grid_v[i - 1] + grid_v[i]);
    grid_t.push_back(grid_t.back() + (grid_s[i] - grid_s[i - 1]) / std::max(mean, 1e-9));
  }
  const double t_stop = grid_t.back();

  RefTrajectory brake;
  std::size_t segment = 1;
  for (double t = 0.0; ; t += dt) {
    const double tc = std::min(t, t_stop);
    while (segment + 1 < grid_t.size() && grid_t[segment] < tc) {
      ++segment;
    }
    double distance = grid_s.back();
    double speed = grid_v.back();
    if (grid_s.size() > 1) {
      const std::size_t i = segment;
      const double span = grid_t[i] - grid_t[i - 1];
      const double tau = std::clamp(tc - grid_t[i - 1], 0.0, span);
      const double a = span > 1e-12 ? (grid_v[i] - grid_v[i - 1]) / span : 0.0;
      distance = grid_s[i - 1] + grid_v[i - 1] * tau + 0.5 * a * tau * tau;
      speed = std::max(grid_v[i - 1] + a * tau, 0.0);
    }
    RefPoint p;
    p.t = tc;
    Eigen::Vector2d tangent;
    at_arc(distance, &p.position, &tangent, &p.yaw);
    p.velocity = Eigen::Vector2d(speed * tangent);
    brake.push_back(p);
    if (tc >= t_stop) {
      break;
    }
  }
  // Accelerations and yaw rates by finite differences; the start keeps the
  // reference's yaw rate, the end is at rest.
  for (std::size_t k = 0; k < brake.size(); ++k) {
    const std::size_t a = k == 0 ? 0 : k - 1;
    const std::size_t b = std::min(k + 1, brake.size() - 1);
    const double span = brake[b].t - brake[a].t;
    if (span > 1e-9) {
      brake[k].acceleration = (brake[b].velocity - brake[a].velocity) / span;
      brake[k].yaw_rate = (brake[b].yaw - brake[a].yaw) / span;
    }
  }
  brake.front().yaw_rate = start.yaw_rate;
  brake.back().velocity.setZero();
  brake.back().acceleration.setZero();
  brake.back().yaw_rate = 0.0;
  RefPoint rest = brake.back();
  rest.t += dt;
  brake.push_back(rest);
  return brake;
}

}  // namespace rov_planning
