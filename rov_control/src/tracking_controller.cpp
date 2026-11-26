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

#include "rov_control/tracking_controller.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace rov_control
{

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::string ValidateTrajectory(const std::vector<TrajectorySample> & samples)
{
  if (samples.empty()) {
    return "the trajectory has no points";
  }
  if (samples.front().time != 0.0) {
    return "the first point must have time_from_start = 0";
  }
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const TrajectorySample & s = samples[i];
    if (!std::isfinite(s.time) || !s.position.allFinite() || !s.velocity.allFinite() ||
      !s.acceleration.allFinite() || !std::isfinite(s.yaw) || !std::isfinite(s.yaw_rate))
    {
      return "point " + std::to_string(i) + " contains NaN or infinity";
    }
    if (i > 0 && !(s.time > samples[i - 1].time)) {
      return "time_from_start must strictly increase (point " + std::to_string(i) + ")";
    }
  }
  return "";
}

Reference SampleTrajectory(const std::vector<TrajectorySample> & samples, double t)
{
  Reference reference;
  const TrajectorySample & first = samples.front();
  const TrajectorySample & last = samples.back();

  if (t <= first.time || samples.size() == 1) {
    const TrajectorySample & s = (t <= first.time) ? first : last;
    reference.position = s.position;
    reference.yaw = s.yaw;
    if (t <= first.time && samples.size() > 1) {
      reference.velocity = s.velocity;
      reference.acceleration = s.acceleration;
      reference.yaw_rate = s.yaw_rate;
    }
    return reference;
  }
  if (t >= last.time) {
    // Hold the last point still.
    reference.position = last.position;
    reference.yaw = last.yaw;
    return reference;
  }

  // Find the interval [k, k + 1] that contains t (samples are sorted).
  std::size_t k = 0;
  while (samples[k + 1].time < t) {
    ++k;
  }
  const TrajectorySample & a = samples[k];
  const TrajectorySample & b = samples[k + 1];
  const double dt = b.time - a.time;
  const double w = (t - a.time) / dt;  // 0 at a, 1 at b

  reference.position = (1.0 - w) * a.position + w * b.position;
  reference.velocity = (1.0 - w) * a.velocity + w * b.velocity;
  reference.acceleration = (1.0 - w) * a.acceleration + w * b.acceleration;
  reference.yaw = (1.0 - w) * a.yaw + w * b.yaw;
  reference.yaw_rate = (1.0 - w) * a.yaw_rate + w * b.yaw_rate;
  reference.yaw_acceleration = (b.yaw_rate - a.yaw_rate) / dt;
  return reference;
}

TrackingController::TrackingController(const PlanarModel & model, const TrackingGains & gains)
: model_(model), gains_(gains)
{
}

TrackingOutput TrackingController::Compute(
  const Eigen::Vector3d & pose, const Eigen::Vector3d & body_velocity,
  const Reference & reference) const
{
  // Rotation from the trajectory frame into the current body frame.
  const double c = std::cos(pose(2));
  const double s = std::sin(pose(2));
  auto to_body = [c, s](const Eigen::Vector2d & w) {
      return Eigen::Vector2d(c * w.x() + s * w.y(), -s * w.x() + c * w.y());
    };

  TrackingOutput output;
  output.position_error = to_body(reference.position - pose.head<2>());
  output.yaw_error = NormalizeAngle(reference.yaw - pose(2));

  // Reference velocity in the body frame: nu_ref = [u_ref, v_ref, r_ref].
  const Eigen::Vector2d velocity_ref = to_body(reference.velocity);
  const Eigen::Vector3d nu_ref(velocity_ref.x(), velocity_ref.y(), reference.yaw_rate);
  const Eigen::Vector3d velocity_error = nu_ref - body_velocity;

  if (gains_.use_feedforward) {
    // Body-frame acceleration of the reference. A world acceleration a
    // seen from a frame turning at r is [u_dot, v_dot] = R^T a + r [v, -u].
    const Eigen::Vector2d acceleration_ref = to_body(reference.acceleration);
    const Eigen::Vector3d nu_dot_ref(
      acceleration_ref.x() + reference.yaw_rate * velocity_ref.y(),
      acceleration_ref.y() - reference.yaw_rate * velocity_ref.x(),
      reference.yaw_acceleration);
    output.feedforward = model_.RequiredWrench(nu_ref, nu_dot_ref);
  }

  const Eigen::Vector3d proportional(
    gains_.position * output.position_error.x(),
    gains_.position * output.position_error.y(),
    gains_.yaw * output.yaw_error);
  const Eigen::Vector3d derivative(
    gains_.velocity * velocity_error(0),
    gains_.velocity * velocity_error(1),
    gains_.yaw_rate * velocity_error(2));
  output.wrench = output.feedforward + proportional + derivative;
  return output;
}

}  // namespace rov_control
