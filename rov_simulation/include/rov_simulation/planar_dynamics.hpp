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

// Thrust-driven planar (surge, sway, yaw) dynamics of the V4 ROV.
// This library does not use ROS.
//
// The hydrodynamic model (Fossen 3 DOF: added mass, Coriolis, linear and
// quadratic damping) is rov_control::PlanarModel, the same code the
// trajectory controller uses for its feedforward. Its input is
//   tau = [Fx, Fy, Mz] = rows 0, 1, 5 of B * thrust (B from the allocator).
// Fz, Mx and My are ignored (planar model).
//
// Pose in the odom frame: [x_dot, y_dot] = R(yaw) * [u, v] + water current,
// yaw_dot = r. Each thruster follows its command with a first-order lag
// (time constant tau_m). Everything is integrated with fixed-step RK4.

#ifndef ROV_SIMULATION__PLANAR_DYNAMICS_HPP_
#define ROV_SIMULATION__PLANAR_DYNAMICS_HPP_

#include <string>

#include "Eigen/Dense"
#include "rov_control/planar_model.hpp"
#include "rov_control/thrust_allocator.hpp"

namespace rov_simulation
{

using rov_control::AllocationMatrix;
using rov_control::HydrodynamicParams;
using rov_control::LoadHydrodynamicParams;
using rov_control::PlanarModel;
using rov_control::kNumThrusters;
using rov_control::ThrustVector;

// Read dynamics/time_constant (tau_m, s) from a thrusters YAML file.
double LoadThrusterTimeConstant(const std::string & yaml_path);

// [Fx, Fy, Mz] that the given thruster forces produce (rows 0, 1, 5 of B).
// Fz, Mx and My are dropped (planar model).
Eigen::Vector3d PlanarWrench(
  const AllocationMatrix & allocation_matrix,
  const ThrustVector & thrust);

// Thruster lag and command timeout for a simulator that does its own
// integration (the Gazebo plugin). Each thruster force follows its command
// with a first-order lag; a command older than the timeout counts as zero.
class ThrusterBank
{
public:
  // Throws std::invalid_argument for a non-positive time constant or timeout.
  ThrusterBank(double time_constant_sec, double command_timeout_sec);

  // Store a command received at time_sec. Returns false and ignores it if
  // any force is NaN or infinite.
  bool SetCommand(const ThrustVector & forces, double time_sec);

  bool CommandIsFresh(double time_sec) const;

  // Advance the forces by dt_sec, starting at time_sec, using the exact
  // solution of the lag for a constant command. Returns the new forces (N).
  const ThrustVector & Update(double time_sec, double dt_sec);

  // Forget the command and set every force to zero.
  void Reset();

  const ThrustVector & forces() const {return forces_;}

private:
  double time_constant_sec_;
  double command_timeout_sec_;
  ThrustVector command_ = ThrustVector::Zero();
  ThrustVector forces_ = ThrustVector::Zero();
  double command_time_ = 0.0;
  bool has_command_ = false;
};

// Planar state of the vehicle.
struct PlanarState
{
  Eigen::Vector3d pose = Eigen::Vector3d::Zero();      // x (m), y (m), yaw (rad) in odom
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // u, v (m/s), r (rad/s): body, vs. water
  ThrustVector thrust = ThrustVector::Zero();          // actual thruster forces (N), after lag
};

class PlanarDynamics
{
public:
  // water_current: constant current in the odom frame (m/s).
  // Throws std::invalid_argument for a non-positive time constant or
  // effective mass, or non-finite inputs.
  PlanarDynamics(
    const HydrodynamicParams & params, const AllocationMatrix & allocation_matrix,
    double thrust_time_constant, const Eigen::Vector2d & water_current = Eigen::Vector2d::Zero());

  // [Fx, Fy, Mz] produced by the given thruster forces.
  Eigen::Vector3d PlanarWrench(const ThrustVector & thrust) const;

  // Advance the state by dt seconds with one RK4 step. The thrust command is
  // held constant during the step.
  void Step(PlanarState & state, const ThrustVector & command, double dt) const;

  // Body-frame velocity over ground [u, v, r] (adds the water current).
  Eigen::Vector3d GroundVelocity(const PlanarState & state) const;

  const HydrodynamicParams & params() const {return params_;}

private:
  // Time derivative of every state member.
  PlanarState Derivative(const PlanarState & state, const ThrustVector & command) const;

  HydrodynamicParams params_;
  PlanarModel model_;  // shared with the trajectory controller (rov_control)
  AllocationMatrix allocation_matrix_;
  double thrust_time_constant_;
  Eigen::Vector2d water_current_;
};

// Holds the state, the latest thrust command, and the command timeout.
// Time is given in seconds by the caller (for example ROS time), so the same
// code runs in real time, simulated time, or a unit test.
class PlanarSimulator
{
public:
  // max_step_sec: longest RK4 step (1 / integration rate).
  // command_timeout_sec: a command older than this counts as zero thrust.
  PlanarSimulator(
    const PlanarDynamics & dynamics, double max_step_sec, double command_timeout_sec,
    double start_time_sec);

  // Store a new thrust command received at time_sec. Returns false and
  // ignores it if any force is NaN or infinite.
  bool SetCommand(const ThrustVector & forces, double time_sec);

  // Integrate from the current time to time_sec in steps of at most
  // max_step_sec. If time goes backwards, only the clock is reset.
  void AdvanceTo(double time_sec);

  // True if a command was received less than command_timeout_sec before
  // time_sec.
  bool CommandIsFresh(double time_sec) const;

  // Put the vehicle in the given state and forget the last command, so the
  // thrust command is zero until a new one arrives.
  void Reset(const PlanarState & state);

  const PlanarState & state() const {return state_;}
  void set_state(const PlanarState & state) {state_ = state;}
  double time() const {return time_;}
  const PlanarDynamics & dynamics() const {return dynamics_;}

private:
  PlanarDynamics dynamics_;
  double max_step_sec_;
  double command_timeout_sec_;
  double time_;
  PlanarState state_;
  ThrustVector command_ = ThrustVector::Zero();
  double command_time_ = 0.0;
  bool has_command_ = false;
};

}  // namespace rov_simulation

#endif  // ROV_SIMULATION__PLANAR_DYNAMICS_HPP_
