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

// Planar (surge, sway, yaw) hydrodynamic model of the V4 ROV. No ROS.
//
// This is the one model shared by the simulator (rov_simulation, forward
// dynamics) and the trajectory controller (feedforward, inverse dynamics).
//
// Fossen 3-DOF model, body frame, centre of gravity at the base_link origin,
// nu = [u, v, r] the velocity relative to the water:
//
//   M * nu_dot + C(nu) * nu + D(nu) * nu = tau
//
//   M  = diag(m - X_udot, m - Y_vdot, I_z - N_rdot)
//   C(nu) * nu = [ -(m - Y_vdot) * v * r,
//                   (m - X_udot) * u * r,
//                   (X_udot - Y_vdot) * u * v ]
//   D(nu) * nu = -[ (X_u + X_u|u| * |u|) * u,
//                   (Y_v + Y_v|v| * |v|) * v,
//                   (N_r + N_r|r| * |r|) * r ]
//   tau = [Fx (N), Fy (N), Mz (N m)]

#ifndef ROV_CONTROL__PLANAR_MODEL_HPP_
#define ROV_CONTROL__PLANAR_MODEL_HPP_

#include <string>

#include "Eigen/Dense"

namespace rov_control
{

// Hull parameters used by the planar model (design section 6.1). Signs
// follow Fossen: added mass and damping coefficients are <= 0.
struct HydrodynamicParams
{
  double mass = 0.0;       // m, kg
  double inertia_z = 0.0;  // I_z, kg m^2
  double x_udot = 0.0;     // added mass, kg
  double y_vdot = 0.0;     // added mass, kg
  double n_rdot = 0.0;     // added inertia, kg m^2
  double x_u = 0.0;        // linear damping, N s/m
  double y_v = 0.0;        // linear damping, N s/m
  double n_r = 0.0;        // linear damping, N m s
  double x_uu = 0.0;       // quadratic damping, N s^2/m^2
  double y_vv = 0.0;       // quadratic damping, N s^2/m^2
  double n_rr = 0.0;       // quadratic damping, N m s^2
};

// Read the parameters from rov_description/config/hydrodynamics_v4.yaml.
// Throws std::runtime_error if a value is missing, null, has the wrong sign,
// or the centre of gravity is not on the z axis.
HydrodynamicParams LoadHydrodynamicParams(const std::string & yaml_path);

class PlanarModel
{
public:
  // Throws std::invalid_argument if an effective mass or inertia is not
  // positive.
  explicit PlanarModel(const HydrodynamicParams & params);

  // [m - X_udot, m - Y_vdot, I_z - N_rdot]
  const Eigen::Vector3d & effective_mass() const {return effective_mass_;}
  const HydrodynamicParams & params() const {return params_;}

  // C(nu) * nu: Coriolis and centripetal terms (rigid body + added mass).
  Eigen::Vector3d CoriolisTerm(const Eigen::Vector3d & nu) const;

  // D(nu) * nu: damping term. It points along the motion; the water pushes
  // the vehicle with the opposite force, -D(nu) * nu.
  Eigen::Vector3d DampingTerm(const Eigen::Vector3d & nu) const;

  // Forward dynamics (simulator): nu_dot = M^-1 (tau - C(nu) nu - D(nu) nu).
  Eigen::Vector3d Acceleration(const Eigen::Vector3d & nu, const Eigen::Vector3d & tau) const;

  // Inverse dynamics (feedforward): the wrench that gives acceleration
  // nu_dot at velocity nu, tau = M nu_dot + C(nu) nu + D(nu) nu.
  Eigen::Vector3d RequiredWrench(const Eigen::Vector3d & nu, const Eigen::Vector3d & nu_dot) const;

  // d(RequiredWrench) / d(nu) = d(C(nu) nu + D(nu) nu) / d(nu), a 3 x 3
  // matrix (used by the planner's gradients). d(RequiredWrench) / d(nu_dot)
  // is simply diag(effective_mass()).
  Eigen::Matrix3d VelocityJacobian(const Eigen::Vector3d & nu) const;

private:
  HydrodynamicParams params_;
  Eigen::Vector3d effective_mass_;
};

}  // namespace rov_control

#endif  // ROV_CONTROL__PLANAR_MODEL_HPP_
