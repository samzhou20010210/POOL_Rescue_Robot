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

// Attainable planar wrench set of the thrusters (design section 3). No ROS.
//
// Each thruster i adds f_i * g_i to tau = [Fx, Fy, Mz], where g_i is its
// column of the allocation matrix (rows Fx, Fy, Mz) and
// -reverse_limit <= f_i <= forward_limit. The set of all such tau is a
// zonotope. In 3D its faces come from pairs of columns: for every pair
// (i, j) the normal n = g_i x g_j gives the two faces
//   n . tau <= h(n),  h(n) = max over the box of n . tau
//          = n . (G c) + sum_k |n . g_k| * w_k,
// with c the middle and w the half-width of each thruster's range. The
// result is H tau <= h with unit-length rows of H, computed once offline.
// Only thrusters with a planar effect (the horizontal ones, 01-04) count;
// the vertical thrusters have zero planar columns.

#ifndef ROV_PLANNING__WRENCH_POLYTOPE_HPP_
#define ROV_PLANNING__WRENCH_POLYTOPE_HPP_

#include "Eigen/Dense"
#include "rov_control/thrust_allocator.hpp"

namespace rov_planning
{

struct WrenchPolytope
{
  Eigen::MatrixX3d normals;  // H, one unit-length row per face
  Eigen::VectorXd offsets;   // h

  // H tau - h: positive entries are violated faces (N or N m, along the normal).
  Eigen::VectorXd Margins(const Eigen::Vector3d & wrench) const;
  // True if no face is violated by more than tolerance.
  bool Contains(const Eigen::Vector3d & wrench, double tolerance = 1e-9) const;
};

// forward_limit and reverse_limit are positive magnitudes in N (for
// planning: f_plan forward; the reverse value is a separate setting).
// Throws std::invalid_argument for non-positive limits, or if the planar
// columns do not span all three axes.
WrenchPolytope ComputeWrenchPolytope(
  const rov_control::AllocationMatrix & allocation_matrix, double forward_limit,
  double reverse_limit);

}  // namespace rov_planning

#endif  // ROV_PLANNING__WRENCH_POLYTOPE_HPP_
