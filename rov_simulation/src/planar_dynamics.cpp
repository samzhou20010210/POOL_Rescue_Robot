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

#include "rov_simulation/planar_dynamics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace rov_simulation
{

namespace
{

YAML::Node LoadYamlFile(const std::string & yaml_path)
{
  try {
    return YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot read '" + yaml_path + "': " + error.what());
  }
}

// The yaw angle wrapped into [-pi, pi].
double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// state + scale * rate, member by member (used by RK4).
PlanarState AddScaled(const PlanarState & state, const PlanarState & rate, double scale)
{
  PlanarState result;
  result.pose = state.pose + scale * rate.pose;
  result.velocity = state.velocity + scale * rate.velocity;
  result.thrust = state.thrust + scale * rate.thrust;
  return result;
}

}  // namespace

double LoadThrusterTimeConstant(const std::string & yaml_path)
{
  const YAML::Node root = LoadYamlFile(yaml_path);
  const YAML::Node node = root["dynamics"]["time_constant"];
  if (!node || node.IsNull()) {
    throw std::runtime_error(yaml_path + ": 'dynamics/time_constant' is missing or null (TBD)");
  }
  const double time_constant = node.as<double>();
  if (!(time_constant > 0.0) || !std::isfinite(time_constant)) {
    throw std::runtime_error(yaml_path + ": dynamics/time_constant must be positive");
  }
  return time_constant;
}

PlanarDynamics::PlanarDynamics(
  const HydrodynamicParams & params, const AllocationMatrix & allocation_matrix,
  double thrust_time_constant, const Eigen::Vector2d & water_current)
: params_(params),
  model_(params),
  allocation_matrix_(allocation_matrix),
  thrust_time_constant_(thrust_time_constant),
  water_current_(water_current)
{
  if (!(thrust_time_constant_ > 0.0) || !std::isfinite(thrust_time_constant_)) {
    throw std::invalid_argument("thrust time constant must be positive");
  }
  if (!allocation_matrix_.allFinite() || !water_current_.allFinite()) {
    throw std::invalid_argument("allocation matrix and water current must be finite");
  }
}

Eigen::Vector3d PlanarWrench(
  const AllocationMatrix & allocation_matrix,
  const ThrustVector & thrust)
{
  // Rows of B: 0 = Fx, 1 = Fy, 2 = Fz, 3 = Mx, 4 = My, 5 = Mz.
  return Eigen::Vector3d(
    allocation_matrix.row(0).dot(thrust),
    allocation_matrix.row(1).dot(thrust),
    allocation_matrix.row(5).dot(thrust));
}

ThrusterBank::ThrusterBank(double time_constant_sec, double command_timeout_sec)
: time_constant_sec_(time_constant_sec), command_timeout_sec_(command_timeout_sec)
{
  if (!(time_constant_sec_ > 0.0) || !(command_timeout_sec_ > 0.0) ||
    !std::isfinite(time_constant_sec_) || !std::isfinite(command_timeout_sec_))
  {
    throw std::invalid_argument("time constant and command timeout must be positive");
  }
}

bool ThrusterBank::SetCommand(const ThrustVector & forces, double time_sec)
{
  if (!forces.allFinite()) {
    return false;
  }
  command_ = forces;
  command_time_ = time_sec;
  has_command_ = true;
  return true;
}

bool ThrusterBank::CommandIsFresh(double time_sec) const
{
  return has_command_ && (time_sec - command_time_) < command_timeout_sec_;
}

const ThrustVector & ThrusterBank::Update(double time_sec, double dt_sec)
{
  if (dt_sec > 0.0) {
    // A stale command means no thrust: the forces decay to zero.
    const ThrustVector target = CommandIsFresh(time_sec) ? command_ : ThrustVector::Zero();
    // f(t + dt) = target + (f(t) - target) * exp(-dt / tau)
    const double approach = 1.0 - std::exp(-dt_sec / time_constant_sec_);
    forces_ += approach * (target - forces_);
  }
  return forces_;
}

void ThrusterBank::Reset()
{
  command_ = ThrustVector::Zero();
  forces_ = ThrustVector::Zero();
  has_command_ = false;
}

Eigen::Vector3d PlanarDynamics::PlanarWrench(const ThrustVector & thrust) const
{
  return rov_simulation::PlanarWrench(allocation_matrix_, thrust);
}

PlanarState PlanarDynamics::Derivative(
  const PlanarState & state, const ThrustVector & command) const
{
  const double u = state.velocity(0);
  const double v = state.velocity(1);
  const double r = state.velocity(2);
  const double yaw = state.pose(2);

  // Forward dynamics from the shared model: M^-1 (tau - C(nu) nu - D(nu) nu).
  PlanarState rate;
  rate.velocity = model_.Acceleration(state.velocity, PlanarWrench(state.thrust));
  rate.pose(0) = std::cos(yaw) * u - std::sin(yaw) * v + water_current_(0);
  rate.pose(1) = std::sin(yaw) * u + std::cos(yaw) * v + water_current_(1);
  rate.pose(2) = r;
  rate.thrust = (command - state.thrust) / thrust_time_constant_;
  return rate;
}

void PlanarDynamics::Step(PlanarState & state, const ThrustVector & command, double dt) const
{
  // Classic 4th-order Runge-Kutta.
  const PlanarState k1 = Derivative(state, command);
  const PlanarState k2 = Derivative(AddScaled(state, k1, 0.5 * dt), command);
  const PlanarState k3 = Derivative(AddScaled(state, k2, 0.5 * dt), command);
  const PlanarState k4 = Derivative(AddScaled(state, k3, dt), command);

  PlanarState sum;
  sum.pose = k1.pose + 2.0 * k2.pose + 2.0 * k3.pose + k4.pose;
  sum.velocity = k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity;
  sum.thrust = k1.thrust + 2.0 * k2.thrust + 2.0 * k3.thrust + k4.thrust;
  state = AddScaled(state, sum, dt / 6.0);
  state.pose(2) = NormalizeAngle(state.pose(2));
}

Eigen::Vector3d PlanarDynamics::GroundVelocity(const PlanarState & state) const
{
  // Rotate the current from odom into the body frame and add it.
  const double yaw = state.pose(2);
  const double current_forward = std::cos(yaw) * water_current_(0) +
    std::sin(yaw) * water_current_(1);
  const double current_left = -std::sin(yaw) * water_current_(0) +
    std::cos(yaw) * water_current_(1);
  return Eigen::Vector3d(
    state.velocity(0) + current_forward, state.velocity(1) + current_left, state.velocity(2));
}

PlanarSimulator::PlanarSimulator(
  const PlanarDynamics & dynamics, double max_step_sec, double command_timeout_sec,
  double start_time_sec)
: dynamics_(dynamics),
  max_step_sec_(max_step_sec),
  command_timeout_sec_(command_timeout_sec),
  time_(start_time_sec)
{
  if (!(max_step_sec_ > 0.0) || !(command_timeout_sec_ > 0.0)) {
    throw std::invalid_argument("max_step_sec and command_timeout_sec must be positive");
  }
}

bool PlanarSimulator::SetCommand(const ThrustVector & forces, double time_sec)
{
  if (!forces.allFinite()) {
    return false;
  }
  command_ = forces;
  command_time_ = time_sec;
  has_command_ = true;
  return true;
}

void PlanarSimulator::Reset(const PlanarState & state)
{
  state_ = state;
  command_ = ThrustVector::Zero();
  has_command_ = false;
}

bool PlanarSimulator::CommandIsFresh(double time_sec) const
{
  return has_command_ && (time_sec - command_time_) < command_timeout_sec_;
}

void PlanarSimulator::AdvanceTo(double time_sec)
{
  const double gap = time_sec - time_;
  if (gap <= 0.0) {
    time_ = time_sec;  // the clock went backwards (e.g. a simulation reset)
    return;
  }
  // Split the gap into equal steps no longer than max_step_sec_.
  const int num_steps = static_cast<int>(std::ceil(gap / max_step_sec_));
  const double step = gap / num_steps;
  for (int i = 0; i < num_steps; ++i) {
    // A stale command means no thrust: the thrusters spin down (with their
    // lag) and damping slows the vehicle.
    const ThrustVector command = CommandIsFresh(time_) ? command_ : ThrustVector::Zero();
    dynamics_.Step(state_, command, step);
    time_ += step;
  }
  time_ = time_sec;  // remove rounding from the sum of steps
}

}  // namespace rov_simulation
