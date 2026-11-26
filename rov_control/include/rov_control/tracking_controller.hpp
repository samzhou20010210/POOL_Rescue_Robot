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

// Trajectory tracking for the planar ROV (design section 4, controller v1).
// This library does not use ROS.
//
// Control law, everything in the body frame of the current pose:
//   tau = tau_ff + K_p * e_p + K_d * e_v
//   e_p = [x error, y error, yaw error],  e_v = [u error, v error, r error]
//   tau_ff = M nu_dot_ref + C(nu_ref) nu_ref + D(nu_ref) nu_ref
// tau_ff comes from the shared PlanarModel (inverse dynamics). The design's
// tau_ff = M_eff a_ref + D(v_ref) v_ref is the same without the Coriolis
// term; the full model is used so the feedforward matches the simulator.

#ifndef ROV_CONTROL__TRACKING_CONTROLLER_HPP_
#define ROV_CONTROL__TRACKING_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "Eigen/Dense"
#include "rov_control/planar_model.hpp"

namespace rov_control
{

// One trajectory sample, in the trajectory frame (for example map).
// Mirrors rov_interfaces/msg/TrajectoryPoint.
struct TrajectorySample
{
  double time = 0.0;                                        // s since the start
  Eigen::Vector2d position = Eigen::Vector2d::Zero();       // m
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();       // m/s
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();   // m/s^2
  double yaw = 0.0;                                         // rad, unwrapped
  double yaw_rate = 0.0;                                    // rad/s
};

// The reference at one moment.
struct Reference
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();       // m
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();       // m/s
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();   // m/s^2
  double yaw = 0.0;                                         // rad
  double yaw_rate = 0.0;                                    // rad/s
  double yaw_acceleration = 0.0;                            // rad/s^2
};

// Empty string if the samples form a valid trajectory, otherwise the reason:
// no samples, first time not 0, times not strictly increasing, or NaN/inf.
std::string ValidateTrajectory(const std::vector<TrajectorySample> & samples);

// Reference at time t (s since the start), by linear interpolation between
// samples. The yaw acceleration is the slope of yaw_rate in that interval.
// Before the start: the first sample. After the end: the last sample held
// still (velocity and acceleration zero), as the interface specifies.
// The samples must be valid (see ValidateTrajectory).
Reference SampleTrajectory(const std::vector<TrajectorySample> & samples, double t);

// Gains (design section 6.6).
struct TrackingGains
{
  double position = 200.0;   // K_p x, y, N/m
  double velocity = 150.0;   // K_d x, y, N s/m
  double yaw = 50.0;         // K_p yaw, N m/rad
  double yaw_rate = 30.0;    // K_d yaw, N m s/rad
  bool use_feedforward = true;
};

struct TrackingOutput
{
  Eigen::Vector3d wrench = Eigen::Vector3d::Zero();          // [Fx, Fy, Mz], base_link
  Eigen::Vector3d feedforward = Eigen::Vector3d::Zero();     // tau_ff part of wrench
  Eigen::Vector2d position_error = Eigen::Vector2d::Zero();  // reference - robot, body frame, m
  double yaw_error = 0.0;                                    // reference - robot, [-pi, pi]
};

class TrackingController
{
public:
  TrackingController(const PlanarModel & model, const TrackingGains & gains);

  // pose: [x, y, yaw] of base_link in the trajectory frame.
  // body_velocity: [u, v, r] in base_link (from odometry).
  TrackingOutput Compute(
    const Eigen::Vector3d & pose, const Eigen::Vector3d & body_velocity,
    const Reference & reference) const;

  const TrackingGains & gains() const {return gains_;}

private:
  PlanarModel model_;
  TrackingGains gains_;
};

// Wrap an angle into [-pi, pi].
double NormalizeAngle(double angle);

}  // namespace rov_control

#endif  // ROV_CONTROL__TRACKING_CONTROLLER_HPP_
