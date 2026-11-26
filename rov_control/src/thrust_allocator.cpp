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

#include "rov_control/thrust_allocator.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace rov_control
{

namespace
{

// Each solver iteration clamps or releases one thruster, so a few dozen
// iterations are plenty for 8 thrusters. The cap only guards against an
// endless loop caused by rounding.
constexpr int kMaxSolverIterations = 100;

// "thruster_01" for index 1, and so on.
std::string ThrusterName(int number)
{
  char name[32];
  std::snprintf(name, sizeof(name), "thruster_%02d", number);
  return name;
}

// Read a 3-element list such as "position: [x, y, z]".
Eigen::Vector3d ReadVector3(
  const YAML::Node & thruster, const std::string & key, const std::string & thruster_name)
{
  const YAML::Node values = thruster[key];
  if (!values || !values.IsSequence() || values.size() != 3) {
    throw std::runtime_error(thruster_name + ": '" + key + "' must be a list of 3 numbers");
  }
  return Eigen::Vector3d(values[0].as<double>(), values[1].as<double>(), values[2].as<double>());
}

// Whether a thruster is held at one of its limits in the solver.
enum class BoundState { kFree, kAtLower, kAtUpper };

// The result for a refused request: zero forces, the whole request left
// as residual, and the reason.
AllocationResult FaultResult(const WrenchVector & requested, AllocationFault fault)
{
  AllocationResult result;
  result.residual = requested;
  result.converged = fault != AllocationFault::kSolverNotConverged;
  result.fault = fault;
  return result;
}

}  // namespace

std::vector<Thruster> LoadThrusterLayout(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot read thruster file '" + yaml_path + "': " + error.what());
  }

  if (!root["frame_id"] || root["frame_id"].as<std::string>() != "base_link") {
    throw std::runtime_error(yaml_path + ": frame_id must be base_link");
  }
  const YAML::Node thruster_list = root["thrusters"];
  if (!thruster_list || !thruster_list.IsMap() ||
    thruster_list.size() != static_cast<std::size_t>(kNumThrusters))
  {
    throw std::runtime_error(
            yaml_path + ": 'thrusters' must contain exactly " + std::to_string(kNumThrusters) +
            " entries");
  }

  // Look the thrusters up by name, so the order is always thruster_01,
  // thruster_02, ... no matter how the file is sorted.
  std::vector<Thruster> thrusters;
  for (int number = 1; number <= kNumThrusters; ++number) {
    Thruster thruster;
    thruster.name = ThrusterName(number);
    const YAML::Node entry = thruster_list[thruster.name];
    if (!entry) {
      throw std::runtime_error(yaml_path + ": missing " + thruster.name);
    }
    thruster.position = ReadVector3(entry, "position", thruster.name);
    thruster.direction = ReadVector3(entry, "direction", thruster.name);
    if (std::abs(thruster.direction.norm() - 1.0) > 1e-6) {
      throw std::runtime_error(thruster.name + ": direction must be a unit vector");
    }
    thrusters.push_back(thruster);
  }
  return thrusters;
}

AllocationMatrix BuildAllocationMatrix(const std::vector<Thruster> & thrusters)
{
  if (thrusters.size() != static_cast<std::size_t>(kNumThrusters)) {
    throw std::invalid_argument("expected " + std::to_string(kNumThrusters) + " thrusters");
  }
  AllocationMatrix matrix;
  for (int i = 0; i < kNumThrusters; ++i) {
    const Eigen::Vector3d & r = thrusters[i].position;
    const Eigen::Vector3d & d = thrusters[i].direction;
    matrix.block<3, 1>(0, i) = d;            // force on the vehicle
    matrix.block<3, 1>(3, i) = r.cross(d);   // moment about the base_link origin
  }
  return matrix;
}

const char * AllocationFaultToString(AllocationFault fault)
{
  switch (fault) {
    case AllocationFault::kNone:
      return "none";
    case AllocationFault::kNonFiniteRequest:
      return "non-finite request";
    case AllocationFault::kNonFiniteSolution:
      return "non-finite solver result";
    case AllocationFault::kSolverNotConverged:
      return "solver did not converge";
  }
  return "unknown";
}

ThrustAllocator::ThrustAllocator(
  const AllocationMatrix & allocation_matrix, const AllocatorSettings & settings)
: allocation_matrix_(allocation_matrix), settings_(settings)
{
  if (!allocation_matrix_.allFinite()) {
    throw std::invalid_argument("the allocation matrix contains NaN or infinity");
  }
  // "!(x > 0)" is also true for NaN, so NaN settings are rejected too.
  if (!(settings_.max_forward_force > 0.0) || !(settings_.max_reverse_force > 0.0) ||
    !std::isfinite(settings_.max_forward_force) || !std::isfinite(settings_.max_reverse_force))
  {
    throw std::invalid_argument("max_forward_force and max_reverse_force must be positive");
  }
  if (!settings_.weights.allFinite() || (settings_.weights.array() <= 0.0).any()) {
    throw std::invalid_argument("all wrench weights must be positive");
  }
  if (!(settings_.regularization >= 0.0) || !(settings_.saturation_tolerance >= 0.0) ||
    !std::isfinite(settings_.regularization) || !std::isfinite(settings_.saturation_tolerance))
  {
    throw std::invalid_argument("regularization and saturation_tolerance must not be negative");
  }
  hessian_ = allocation_matrix_.transpose() * settings_.weights.asDiagonal() * allocation_matrix_;
  hessian_.diagonal().array() += settings_.regularization;
}

// The QP is solved with a small "active set" method. It is exact (no
// tolerances to tune) and easy to follow for 8 variables:
//
//   1. Some thrusters are "clamped" at a limit, the rest are "free". Start
//      with every thruster free and every force 0 (0 is inside the limits).
//   2. With the clamped forces held fixed, solve for the free forces that
//      minimize the cost. This is a small linear system.
//   3. If that answer pushes a free thruster past its limit, move only part
//      of the way, stop where the first thruster reaches its limit, clamp it,
//      and go back to 2.
//   4. Otherwise accept the answer, then check every clamped thruster: if
//      moving it back inside its limits would lower the cost, free it and go
//      back to 2. If none would, the answer is optimal.
AllocationResult ThrustAllocator::Allocate(const WrenchVector & requested) const
{
  // A NaN or infinite request has no meaningful answer: command zero thrust.
  if (!requested.allFinite()) {
    return FaultResult(requested, AllocationFault::kNonFiniteRequest);
  }

  const double lower = -settings_.max_reverse_force;
  const double upper = settings_.max_forward_force;
  // The cost is 1/2 f^T H f - c^T f (+ constant), with c = B^T W tau.
  const ThrustVector linear_term =
    allocation_matrix_.transpose() * settings_.weights.asDiagonal() * requested;

  ThrustVector forces = ThrustVector::Zero();
  std::array<BoundState, kNumThrusters> state;
  state.fill(BoundState::kFree);

  AllocationResult result;
  result.converged = false;
  for (int iteration = 0; iteration < kMaxSolverIterations; ++iteration) {
    // Step 2: solve H_ff * f_free = c_free - H_fc * f_clamped.
    std::vector<int> free_index;
    for (int i = 0; i < kNumThrusters; ++i) {
      if (state[i] == BoundState::kFree) {
        free_index.push_back(i);
      }
    }
    ThrustVector candidate = forces;
    if (!free_index.empty()) {
      const int num_free = static_cast<int>(free_index.size());
      Eigen::MatrixXd hessian_free(num_free, num_free);
      Eigen::VectorXd right_side(num_free);
      for (int row = 0; row < num_free; ++row) {
        const int i = free_index[row];
        right_side(row) = linear_term(i);
        for (int j = 0; j < kNumThrusters; ++j) {
          if (state[j] != BoundState::kFree) {
            right_side(row) -= hessian_(i, j) * forces(j);
          }
        }
        for (int col = 0; col < num_free; ++col) {
          hessian_free(row, col) = hessian_(i, free_index[col]);
        }
      }
      const Eigen::VectorXd free_forces = hessian_free.ldlt().solve(right_side);
      for (int row = 0; row < num_free; ++row) {
        candidate(free_index[row]) = free_forces(row);
      }
    }

    // Step 3: find the first free thruster that would cross a limit.
    double step = 1.0;
    int blocking = -1;
    BoundState blocking_state = BoundState::kFree;
    for (const int i : free_index) {
      const double change = candidate(i) - forces(i);
      if (candidate(i) > upper && change > 0.0) {
        const double fraction = (upper - forces(i)) / change;
        if (fraction < step) {
          step = fraction;
          blocking = i;
          blocking_state = BoundState::kAtUpper;
        }
      } else if (candidate(i) < lower && change < 0.0) {
        const double fraction = (lower - forces(i)) / change;
        if (fraction < step) {
          step = fraction;
          blocking = i;
          blocking_state = BoundState::kAtLower;
        }
      }
    }
    if (blocking >= 0) {
      forces += step * (candidate - forces);
      forces(blocking) = (blocking_state == BoundState::kAtUpper) ? upper : lower;
      state[blocking] = blocking_state;
      continue;
    }
    forces = candidate;

    // Step 4: free the clamped thruster that most wants to move inside.
    // The gradient of the cost is H f - c. At the lower limit, a negative
    // gradient means increasing the force lowers the cost; at the upper
    // limit, a positive gradient means decreasing it does.
    const ThrustVector gradient = hessian_ * forces - linear_term;
    int release = -1;
    double strongest_pull = 1e-12;  // ignore rounding-level gradients
    for (int i = 0; i < kNumThrusters; ++i) {
      double pull = 0.0;
      if (state[i] == BoundState::kAtLower) {
        pull = -gradient(i);
      } else if (state[i] == BoundState::kAtUpper) {
        pull = gradient(i);
      }
      if (pull > strongest_pull) {
        strongest_pull = pull;
        release = i;
      }
    }
    if (release < 0) {
      result.converged = true;
      break;
    }
    state[release] = BoundState::kFree;
  }

  // Check the solver output before clamping: clamping can hide a NaN by
  // turning it into a limit value, which would command full thrust.
  if (!forces.allFinite()) {
    return FaultResult(requested, AllocationFault::kNonFiniteSolution);
  }
  if (!result.converged) {
    return FaultResult(requested, AllocationFault::kSolverNotConverged);
  }

  // The forces are always inside the limits; this only removes rounding.
  result.forces = forces.cwiseMax(lower).cwiseMin(upper);
  result.achieved = allocation_matrix_ * result.forces;
  result.residual = requested - result.achieved;
  for (int i = 0; i < kNumThrusters; ++i) {
    const double force = result.forces(i);
    if (force >= upper - settings_.saturation_tolerance ||
      force <= lower + settings_.saturation_tolerance)
    {
      ++result.num_saturated;
    }
  }
  result.saturated = result.num_saturated > 0;
  return result;
}

}  // namespace rov_control
