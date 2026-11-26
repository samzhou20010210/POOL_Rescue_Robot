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

#include "rov_control/planar_model.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace rov_control
{

namespace
{

// Read a required number. "name" is only used in the error message.
double ReadNumber(const YAML::Node & node, const std::string & name, const std::string & yaml_path)
{
  if (!node || node.IsNull()) {
    throw std::runtime_error(yaml_path + ": '" + name + "' is missing or null (TBD)");
  }
  const double value = node.as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(yaml_path + ": '" + name + "' must be a finite number");
  }
  return value;
}

double ReadNegativeOrZero(
  const YAML::Node & node, const std::string & name, const std::string & yaml_path)
{
  const double value = ReadNumber(node, name, yaml_path);
  if (value > 0.0) {
    throw std::runtime_error(
            yaml_path + ": '" + name + "' must be <= 0 (Fossen sign convention)");
  }
  return value;
}

}  // namespace

HydrodynamicParams LoadHydrodynamicParams(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot read '" + yaml_path + "': " + error.what());
  }
  HydrodynamicParams params;
  params.mass = ReadNumber(root["mass"], "mass", yaml_path);
  params.inertia_z = ReadNumber(root["inertia_z"], "inertia_z", yaml_path);
  if (params.mass <= 0.0 || params.inertia_z <= 0.0) {
    throw std::runtime_error(yaml_path + ": mass and inertia_z must be positive");
  }

  // The planar model has no coupling terms for an off-centre centre of
  // gravity, so refuse a CG that is not on the z axis.
  const YAML::Node cg = root["center_of_gravity"];
  if (!cg || !cg.IsSequence() || cg.size() != 3) {
    throw std::runtime_error(yaml_path + ": 'center_of_gravity' must be a list of 3 numbers");
  }
  if (cg[0].as<double>() != 0.0 || cg[1].as<double>() != 0.0) {
    throw std::runtime_error(
            yaml_path + ": the planar model needs center_of_gravity x = y = 0");
  }

  const YAML::Node added = root["added_mass"];
  params.x_udot = ReadNegativeOrZero(added["x_udot"], "added_mass/x_udot", yaml_path);
  params.y_vdot = ReadNegativeOrZero(added["y_vdot"], "added_mass/y_vdot", yaml_path);
  params.n_rdot = ReadNegativeOrZero(added["n_rdot"], "added_mass/n_rdot", yaml_path);

  const YAML::Node linear = root["linear_damping"];
  params.x_u = ReadNegativeOrZero(linear["x_u"], "linear_damping/x_u", yaml_path);
  params.y_v = ReadNegativeOrZero(linear["y_v"], "linear_damping/y_v", yaml_path);
  params.n_r = ReadNegativeOrZero(linear["n_r"], "linear_damping/n_r", yaml_path);

  const YAML::Node quadratic = root["quadratic_damping"];
  params.x_uu = ReadNegativeOrZero(quadratic["x_uu"], "quadratic_damping/x_uu", yaml_path);
  params.y_vv = ReadNegativeOrZero(quadratic["y_vv"], "quadratic_damping/y_vv", yaml_path);
  params.n_rr = ReadNegativeOrZero(quadratic["n_rr"], "quadratic_damping/n_rr", yaml_path);
  return params;
}

PlanarModel::PlanarModel(const HydrodynamicParams & params)
: params_(params)
{
  effective_mass_ << params_.mass - params_.x_udot, params_.mass - params_.y_vdot,
    params_.inertia_z - params_.n_rdot;
  if (!(effective_mass_.array() > 0.0).all() || !effective_mass_.allFinite()) {
    throw std::invalid_argument("effective mass and inertia must be positive");
  }
}

Eigen::Vector3d PlanarModel::CoriolisTerm(const Eigen::Vector3d & nu) const
{
  const double u = nu(0);
  const double v = nu(1);
  const double r = nu(2);
  return Eigen::Vector3d(
    -effective_mass_(1) * v * r,
    effective_mass_(0) * u * r,
    (params_.x_udot - params_.y_vdot) * u * v);
}

Eigen::Vector3d PlanarModel::DampingTerm(const Eigen::Vector3d & nu) const
{
  // The coefficients are negative, so the minus sign makes this term point
  // along the motion.
  const double u = nu(0);
  const double v = nu(1);
  const double r = nu(2);
  return -Eigen::Vector3d(
    (params_.x_u + params_.x_uu * std::abs(u)) * u,
    (params_.y_v + params_.y_vv * std::abs(v)) * v,
    (params_.n_r + params_.n_rr * std::abs(r)) * r);
}

Eigen::Vector3d PlanarModel::Acceleration(
  const Eigen::Vector3d & nu, const Eigen::Vector3d & tau) const
{
  return (tau - CoriolisTerm(nu) - DampingTerm(nu)).cwiseQuotient(effective_mass_);
}

Eigen::Vector3d PlanarModel::RequiredWrench(
  const Eigen::Vector3d & nu, const Eigen::Vector3d & nu_dot) const
{
  return effective_mass_.cwiseProduct(nu_dot) + CoriolisTerm(nu) + DampingTerm(nu);
}

Eigen::Matrix3d PlanarModel::VelocityJacobian(const Eigen::Vector3d & nu) const
{
  const double u = nu(0);
  const double v = nu(1);
  const double r = nu(2);
  const double m_u = effective_mass_(0);
  const double m_v = effective_mass_(1);
  const double munk = params_.x_udot - params_.y_vdot;

  // Columns: derivative with respect to u, v, r.
  Eigen::Matrix3d jacobian;
  jacobian <<
    0.0, -m_v * r, -m_v * v,
    m_u * r, 0.0, m_u * u,
    munk * v, munk * u, 0.0;
  // d/ds of -(k_lin s + k_quad s |s|) is -(k_lin + 2 k_quad |s|).
  jacobian(0, 0) -= params_.x_u + 2.0 * params_.x_uu * std::abs(u);
  jacobian(1, 1) -= params_.y_v + 2.0 * params_.y_vv * std::abs(v);
  jacobian(2, 2) -= params_.n_r + 2.0 * params_.n_rr * std::abs(r);
  return jacobian;
}

}  // namespace rov_control
