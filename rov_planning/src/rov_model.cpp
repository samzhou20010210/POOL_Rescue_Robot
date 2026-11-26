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

#include "rov_planning/rov_model.hpp"

#include <cmath>

namespace rov_planning
{

RovModel::RovModel(const rov_control::PlanarModel & model)
: model_(model)
{
}

Eigen::Vector3d RovModel::Wrench(const TrajectoryState & state) const
{
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double r = state.yaw_rate;
  const Eigen::Vector2d velocity_body(
    c * state.velocity.x() + s * state.velocity.y(),
    -s * state.velocity.x() + c * state.velocity.y());
  // to_body * a + r * J * v_body, with J [u, v] = [v, -u].
  const Eigen::Vector2d acceleration_body(
    c * state.acceleration.x() + s * state.acceleration.y() + r * velocity_body.y(),
    -s * state.acceleration.x() + c * state.acceleration.y() - r * velocity_body.x());
  return model_.RequiredWrench(
    Eigen::Vector3d(velocity_body.x(), velocity_body.y(), r),
    Eigen::Vector3d(acceleration_body.x(), acceleration_body.y(), state.yaw_acceleration));
}

RequiredWrench RovModel::Compute(const TrajectoryState & state) const
{
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double r = state.yaw_rate;

  // World -> body rotation R^T and its derivative with respect to yaw.
  Eigen::Matrix2d to_body;
  to_body << c, s, -s, c;
  Eigen::Matrix2d to_body_d_yaw;
  to_body_d_yaw << -s, c, -c, -s;
  // J [u, v] = [v, -u]: the turning-frame term of the body acceleration.
  Eigen::Matrix2d turn;
  turn << 0.0, 1.0, -1.0, 0.0;

  // Body velocity and acceleration.
  const Eigen::Vector2d velocity_body = to_body * state.velocity;
  const Eigen::Vector2d acceleration_body =
    to_body * state.acceleration + r * turn * velocity_body;
  const Eigen::Vector3d nu(velocity_body.x(), velocity_body.y(), r);
  const Eigen::Vector3d nu_dot(
    acceleration_body.x(), acceleration_body.y(), state.yaw_acceleration);

  RequiredWrench result;
  result.wrench = model_.RequiredWrench(nu, nu_dot);

  // Chain rule: tau = M nu_dot + g(nu), so
  // d tau / dx = M d nu_dot / dx + G d nu / dx, G = d g / d nu.
  const Eigen::Matrix3d mass = model_.effective_mass().asDiagonal();
  const Eigen::Matrix3d g_jacobian = model_.VelocityJacobian(nu);

  // Velocity v (world, 2 columns).
  Eigen::Matrix<double, 3, 2> d_nu_d_velocity = Eigen::Matrix<double, 3, 2>::Zero();
  d_nu_d_velocity.topRows<2>() = to_body;
  Eigen::Matrix<double, 3, 2> d_nu_dot_d_velocity = Eigen::Matrix<double, 3, 2>::Zero();
  d_nu_dot_d_velocity.topRows<2>() = r * turn * to_body;
  result.d_velocity = mass * d_nu_dot_d_velocity + g_jacobian * d_nu_d_velocity;

  // Acceleration a (world): only nu_dot depends on it.
  Eigen::Matrix<double, 3, 2> d_nu_dot_d_acceleration = Eigen::Matrix<double, 3, 2>::Zero();
  d_nu_dot_d_acceleration.topRows<2>() = to_body;
  result.d_acceleration = mass * d_nu_dot_d_acceleration;

  // Yaw: rotates both the velocity and the acceleration into the body frame.
  const Eigen::Vector2d velocity_body_d_yaw = to_body_d_yaw * state.velocity;
  Eigen::Vector3d d_nu_d_yaw = Eigen::Vector3d::Zero();
  d_nu_d_yaw.head<2>() = velocity_body_d_yaw;
  Eigen::Vector3d d_nu_dot_d_yaw = Eigen::Vector3d::Zero();
  d_nu_dot_d_yaw.head<2>() =
    to_body_d_yaw * state.acceleration + r * turn * velocity_body_d_yaw;
  result.d_yaw = mass * d_nu_dot_d_yaw + g_jacobian * d_nu_d_yaw;

  // Yaw rate: is nu(2) and scales the turning term.
  Eigen::Vector3d d_nu_dot_d_rate = Eigen::Vector3d::Zero();
  d_nu_dot_d_rate.head<2>() = turn * velocity_body;
  result.d_yaw_rate = mass * d_nu_dot_d_rate + g_jacobian.col(2);

  // Yaw acceleration: is nu_dot(2).
  result.d_yaw_acceleration = mass.col(2);
  return result;
}

}  // namespace rov_planning
