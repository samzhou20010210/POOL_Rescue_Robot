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

#include "rov_planning/wrench_polytope.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace rov_planning
{

Eigen::VectorXd WrenchPolytope::Margins(const Eigen::Vector3d & wrench) const
{
  return normals * wrench - offsets;
}

bool WrenchPolytope::Contains(const Eigen::Vector3d & wrench, double tolerance) const
{
  return Margins(wrench).maxCoeff() <= tolerance;
}

WrenchPolytope ComputeWrenchPolytope(
  const rov_control::AllocationMatrix & allocation_matrix, double forward_limit,
  double reverse_limit)
{
  if (!(forward_limit > 0.0) || !(reverse_limit > 0.0)) {
    throw std::invalid_argument("thrust limits must be positive");
  }

  // Planar columns g_k = [Fx, Fy, Mz] of the thrusters that have one.
  std::vector<Eigen::Vector3d> columns;
  for (int k = 0; k < rov_control::kNumThrusters; ++k) {
    const Eigen::Vector3d g(
      allocation_matrix(0, k), allocation_matrix(1, k), allocation_matrix(5, k));
    if (g.norm() > 1e-9) {
      columns.push_back(g);
    }
  }
  Eigen::Matrix3Xd generators(3, columns.size());
  for (std::size_t k = 0; k < columns.size(); ++k) {
    generators.col(k) = columns[k];
  }
  if (Eigen::FullPivLU<Eigen::Matrix3Xd>(generators).rank() < 3) {
    throw std::invalid_argument("the thrusters cannot produce every planar wrench direction");
  }

  // Each thruster's range as middle +/- half-width.
  const double middle = 0.5 * (forward_limit - reverse_limit);
  const double half_width = 0.5 * (forward_limit + reverse_limit);
  const Eigen::Vector3d center = generators.rowwise().sum() * middle;

  std::vector<Eigen::Vector3d> normals;
  std::vector<double> offsets;
  auto add_face = [&](const Eigen::Vector3d & n) {
      double support = n.dot(center);
      for (const Eigen::Vector3d & g : columns) {
        support += std::abs(n.dot(g)) * half_width;
      }
      normals.push_back(n);
      offsets.push_back(support);
    };
  for (std::size_t i = 0; i < columns.size(); ++i) {
    for (std::size_t j = i + 1; j < columns.size(); ++j) {
      const Eigen::Vector3d n = columns[i].cross(columns[j]);
      if (n.norm() < 1e-9) {
        continue;  // parallel columns give no face
      }
      add_face(n.normalized());
      add_face(-n.normalized());
    }
  }

  WrenchPolytope polytope;
  polytope.normals.resize(normals.size(), 3);
  polytope.offsets.resize(offsets.size());
  for (std::size_t f = 0; f < normals.size(); ++f) {
    polytope.normals.row(f) = normals[f].transpose();
    polytope.offsets(f) = offsets[f];
  }
  return polytope;
}

}  // namespace rov_planning
