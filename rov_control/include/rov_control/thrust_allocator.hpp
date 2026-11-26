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

// Thrust allocation for the V4 ROV (8 x T500). This library does not use ROS.
//
// A body wrench tau = [Fx, Fy, Fz, Mx, My, Mz] (base_link, N and N*m) is
// turned into 8 thruster forces f (N) by solving the box-constrained QP
// (SURFACE_APPROACH_DESIGN.md, section 4):
//
//   minimize   (B f - tau)^T W (B f - tau) + lambda * |f|^2
//   subject to -max_reverse_force <= f_i <= max_forward_force
//
// B (6 x 8) is the allocation matrix. Column i is [d_i ; r_i x d_i], where
// r_i is thruster i's position and d_i its unit thrust direction, both in
// base_link. Moments are taken about the base_link origin (the design's
// centre of gravity, r_g = (0, 0, 0), provisional).
//
// Thruster order everywhere (B columns, force vectors) is the order of
// rov_interfaces/msg/ThrusterCommand: index i = thruster_0(i+1).

#ifndef ROV_CONTROL__THRUST_ALLOCATOR_HPP_
#define ROV_CONTROL__THRUST_ALLOCATOR_HPP_

#include <string>
#include <vector>

#include "Eigen/Dense"

namespace rov_control
{

constexpr int kNumThrusters = 8;

// [Fx, Fy, Fz, Mx, My, Mz] in base_link. Forces in N, moments in N*m.
using WrenchVector = Eigen::Matrix<double, 6, 1>;
// One force per thruster, in N. Positive = along the thruster's direction.
using ThrustVector = Eigen::Matrix<double, kNumThrusters, 1>;
using AllocationMatrix = Eigen::Matrix<double, 6, kNumThrusters>;

// Mount position and thrust direction of one thruster, in base_link.
struct Thruster
{
  std::string name;           // e.g. "thruster_01"
  Eigen::Vector3d position;   // m
  Eigen::Vector3d direction;  // unit vector
};

// Read thruster_01 ... thruster_08 from a thrusters YAML file
// (rov_description/config/thrusters_v4.yaml), in that order.
// Throws std::runtime_error if the file or a field is missing, the frame is
// not base_link, or a direction is not a unit vector.
std::vector<Thruster> LoadThrusterLayout(const std::string & yaml_path);

// Build B from the thruster layout (must have exactly kNumThrusters entries).
AllocationMatrix BuildAllocationMatrix(const std::vector<Thruster> & thrusters);

struct AllocatorSettings
{
  // Thrust limits, both given as positive magnitudes, in N.
  double max_forward_force = 140.0;  // f_ctrl, design section 6.2 (set value)
  double max_reverse_force = 100.0;  // f_rev is TBD (bench test); provisional
  // W: weight of each wrench axis [Fx, Fy, Fz, Mx, My, Mz] in the cost.
  // (Fx, Fy, Mz) = (1, 1, 2) from design section 6.6. (Fz, Mx, My) =
  // (1, 10, 10) are provisional, taken from the ROS 1 holo_control allocator.
  WrenchVector weights = (WrenchVector() << 1.0, 1.0, 1.0, 10.0, 10.0, 2.0).finished();
  // lambda: small thrust-effort penalty, design section 6.6.
  double regularization = 1e-3;
  // A thruster closer than this to its limit counts as saturated, in N.
  double saturation_tolerance = 1e-3;
};

// Why an allocation was refused. On any fault the forces are all zero.
enum class AllocationFault
{
  kNone,                // normal result
  kNonFiniteRequest,    // the requested wrench contains NaN or infinity
  kNonFiniteSolution,   // the solver produced NaN or infinity (e.g. overflow)
  kSolverNotConverged,  // the solver hit its iteration cap
};

// Short text for logs and diagnostics, e.g. "non-finite request".
const char * AllocationFaultToString(AllocationFault fault);

struct AllocationResult
{
  ThrustVector forces = ThrustVector::Zero();      // N, within the limits
  WrenchVector achieved = WrenchVector::Zero();    // B * forces
  WrenchVector residual = WrenchVector::Zero();    // requested - achieved
  bool saturated = false;          // true if any thruster is at a limit
  int num_saturated = 0;           // how many thrusters are at a limit
  bool converged = true;           // false only if the solver hit its iteration cap
  // kNone for a normal result. Otherwise forces, achieved and saturated are
  // zero/false, and residual equals the request.
  AllocationFault fault = AllocationFault::kNone;
};

class ThrustAllocator
{
public:
  // Throws std::invalid_argument if the settings are not valid
  // (non-positive or non-finite limits or weights, negative regularization)
  // or the allocation matrix contains NaN or infinity.
  ThrustAllocator(const AllocationMatrix & allocation_matrix, const AllocatorSettings & settings);

  // Find the thruster forces for a requested wrench. Never throws: a
  // non-finite request or an invalid solver result gives zero forces and
  // sets result.fault.
  AllocationResult Allocate(const WrenchVector & requested) const;

  const AllocationMatrix & allocation_matrix() const {return allocation_matrix_;}
  const AllocatorSettings & settings() const {return settings_;}

private:
  AllocationMatrix allocation_matrix_;
  AllocatorSettings settings_;
  // Hessian of the cost: B^T W B + lambda * I. It does not depend on the
  // request, so it is computed once.
  Eigen::Matrix<double, kNumThrusters, kNumThrusters> hessian_;
};

}  // namespace rov_control

#endif  // ROV_CONTROL__THRUST_ALLOCATOR_HPP_
