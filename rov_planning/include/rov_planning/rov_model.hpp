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

// Body wrench required by a planar trajectory sample, with its gradients
// (design section 3, thrust-feasibility constraint). No ROS.
//
// The sample is given in the world (map) frame. It is turned into the body
// frame and passed to the shared rov_control::PlanarModel:
//   nu     = [R(yaw)^T v, yaw_rate]
//   nu_dot = [R(yaw)^T a + yaw_rate * [v_b, -u_b], yaw_acceleration]
//   tau    = M nu_dot + C(nu) nu + D(nu) nu
// (the same formula as the tracking controller's feedforward). The position
// does not change the wrench (no water current in the planner).

#ifndef ROV_PLANNING__ROV_MODEL_HPP_
#define ROV_PLANNING__ROV_MODEL_HPP_

#include "Eigen/Dense"
#include "rov_control/planar_model.hpp"

namespace rov_planning
{

// World-frame derivatives of a trajectory sample.
struct TrajectoryState
{
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();      // m/s
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();  // m/s^2
  double yaw = 0.0;                                        // rad
  double yaw_rate = 0.0;                                   // rad/s
  double yaw_acceleration = 0.0;                           // rad/s^2
};

// tau = [Fx, Fy, Mz] in base_link and its partial derivatives.
struct RequiredWrench
{
  Eigen::Vector3d wrench = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 2> d_velocity = Eigen::Matrix<double, 3, 2>::Zero();
  Eigen::Matrix<double, 3, 2> d_acceleration = Eigen::Matrix<double, 3, 2>::Zero();
  Eigen::Vector3d d_yaw = Eigen::Vector3d::Zero();
  Eigen::Vector3d d_yaw_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d d_yaw_acceleration = Eigen::Vector3d::Zero();
};

class RovModel
{
public:
  explicit RovModel(const rov_control::PlanarModel & model);

  RequiredWrench Compute(const TrajectoryState & state) const;
  // The wrench alone (no gradients): Compute(state).wrench, cheaper.
  Eigen::Vector3d Wrench(const TrajectoryState & state) const;

private:
  rov_control::PlanarModel model_;
};

}  // namespace rov_planning

#endif  // ROV_PLANNING__ROV_MODEL_HPP_
