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

// Sampled reference trajectories for dispatch, handoffs and braking
// (implementation steps 8B and 9). No ROS.
//
// A RefTrajectory is a list of RefPoints with increasing time t (s, from the
// trajectory's own start). Velocities and accelerations are in the map
// frame. The trajectory controller interpolates the same samples.

#ifndef ROV_PLANNING__REFERENCE_TRAJECTORY_HPP_
#define ROV_PLANNING__REFERENCE_TRAJECTORY_HPP_

#include <vector>

#include "Eigen/Dense"
#include "rov_planning/surface_traj_opt.hpp"

namespace rov_planning
{

struct RefPoint
{
  double t = 0.0;                                          // s
  Eigen::Vector2d position = Eigen::Vector2d::Zero();      // m
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();      // m/s
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();  // m/s^2
  double yaw = 0.0;                                        // rad
  double yaw_rate = 0.0;                                   // rad/s
};

using RefTrajectory = std::vector<RefPoint>;

// Samples of `trajectory` from t_begin to t_end (both included, every dt),
// with sample times shifted by `offset` (sample time = t - t_begin + offset).
RefTrajectory SamplePolynomial(
  const PolynomialTrajectory & trajectory, double t_begin, double t_end, double dt,
  double offset);

// Linear interpolation at t, clamped to the ends (yaw unwrapped).
RefPoint InterpolateRef(const RefTrajectory & reference, double t);

// Highest planar speed of the samples (m/s).
double MaxSpeed(const RefTrajectory & reference);

// Stationary reference at `pose` (x, y, yaw): two samples, 0 and dt.
RefTrajectory HoldAt(const Eigen::Vector3d & pose, double dt);

// Brake to rest along the path of `reference`, starting from its state at
// time t0: the position follows the remaining path geometry (arc length) and
// the yaw follows the path's yaw at the same arc length. The speed at arc
// length s is min(sqrt(v0^2 - 2 a s), v_ref(s)), never increasing: at least
// `deceleration` (m/s^2, > 0), and never faster than the reference at the
// same point (a tight turn the reference takes slowly stays feasible). The
// brake stops where the reference itself comes to rest (a stop, a reversal
// of direction or its end at rest) if that comes first.
// Times start at 0; the last sample is at rest, and one more rest sample
// follows. Starting at rest gives HoldAt(start pose).
RefTrajectory BrakeAlong(
  const RefTrajectory & reference, double t0, double deceleration, double dt);

}  // namespace rov_planning

#endif  // ROV_PLANNING__REFERENCE_TRAJECTORY_HPP_
