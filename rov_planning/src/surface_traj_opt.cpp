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

#include "rov_planning/surface_traj_opt.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"
#include "yaml-cpp/yaml.h"

namespace rov_planning
{

namespace
{

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// GCOPTER's map from an unconstrained tau to a duration T > 0 (gcopter.hpp
// forwardT / backwardT / backwardGradT).
double DurationFromTau(double tau)
{
  return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
}

double TauFromDuration(double duration)
{
  return duration > 1.0 ? (std::sqrt(2.0 * duration - 1.0) - 1.0) :
         (1.0 - std::sqrt(2.0 / duration - 1.0));
}

double GradientOfTau(double tau, double gradient_of_duration)
{
  if (tau > 0.0) {
    return gradient_of_duration * (tau + 1.0);
  }
  const double denominator = (0.5 * tau - 1.0) * tau + 1.0;
  return gradient_of_duration * (1.0 - tau) / (denominator * denominator);
}

// GCOPTER's smoothed L1 penalty: 0 for x < 0, x - mu/2 for x > mu, and a
// smooth cubic in between. Returns false (and leaves f, df) for x < 0.
bool SmoothedL1(double x, double mu, double * f, double * df)
{
  if (x < 0.0) {
    return false;
  }
  if (x > mu) {
    *f = x - 0.5 * mu;
    *df = 1.0;
    return true;
  }
  const double ratio = x / mu;
  const double ratio_squared = ratio * ratio;
  const double m = mu - 0.5 * x;
  *f = m * ratio_squared * ratio;
  *df = ratio_squared * (-0.5 * ratio + 3.0 * m / mu);
  return true;
}

// Value of a polynomial (coefficients in ascending powers) at s.
double PolynomialValue(const Eigen::VectorXd & coefficients, double s)
{
  double value = 0.0;
  for (int k = static_cast<int>(coefficients.size()) - 1; k >= 0; --k) {
    value = value * s + coefficients(k);
  }
  return value;
}

// Largest value of a polynomial (ascending powers) on [0, length]: at an end
// or where its derivative is zero (roots from the companion matrix).
double PolynomialMax(const Eigen::VectorXd & coefficients, double length)
{
  double best = std::max(PolynomialValue(coefficients, 0.0), PolynomialValue(coefficients, length));
  const int n = static_cast<int>(coefficients.size());
  Eigen::VectorXd derivative(n - 1);
  for (int k = 1; k < n; ++k) {
    derivative(k - 1) = k * coefficients(k);
  }
  const double scale = derivative.cwiseAbs().maxCoeff();
  int degree = static_cast<int>(derivative.size()) - 1;
  while (degree > 0 && std::abs(derivative(degree)) <= 1e-12 * scale) {
    --degree;
  }
  if (degree < 1 || scale == 0.0) {
    return best;
  }
  Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
  for (int k = 0; k < degree; ++k) {
    companion(0, k) = -derivative(degree - 1 - k) / derivative(degree);
    if (k + 1 < degree) {
      companion(k + 1, k) = 1.0;
    }
  }
  const Eigen::VectorXcd roots = companion.eigenvalues();
  for (int k = 0; k < roots.size(); ++k) {
    const double s = roots(k).real();
    if (std::abs(roots(k).imag()) <= 1e-9 * (1.0 + std::abs(s)) && s > 0.0 && s < length) {
      best = std::max(best, PolynomialValue(coefficients, s));
    }
  }
  return best;
}

// Time-derivative bases of a quintic at local time s (GCOPTER's beta0..beta3).
struct Bases
{
  Eigen::Matrix<double, 6, 1> position, velocity, acceleration, jerk;
};

Bases BasesAt(double s)
{
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s2 * s2;
  const double s5 = s4 * s;
  Bases b;
  b.position << 1.0, s, s2, s3, s4, s5;
  b.velocity << 0.0, 1.0, 2.0 * s, 3.0 * s2, 4.0 * s3, 5.0 * s4;
  b.acceleration << 0.0, 0.0, 2.0, 6.0 * s, 12.0 * s2, 20.0 * s3;
  b.jerk << 0.0, 0.0, 0.0, 6.0, 24.0 * s, 60.0 * s2;
  return b;
}

double ReadPositive(const YAML::Node & root, const std::string & key, const std::string & path)
{
  const YAML::Node node = root[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(path + ": '" + key + "' is missing");
  }
  const double value = node.as<double>();
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::runtime_error(path + ": '" + key + "' must be a positive number");
  }
  return value;
}

double LbfgsCost(void * instance, const Eigen::VectorXd & x, Eigen::VectorXd & gradient)
{
  return static_cast<SurfaceTrajectoryOptimizer *>(instance)->Cost(x, &gradient);
}

}  // namespace

OptimizerSettings LoadOptimizerSettings(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot read '" + yaml_path + "': " + error.what());
  }
  OptimizerSettings s;
  s.cruise_speed = ReadPositive(root, "cruise_speed", yaml_path);
  s.braking_deceleration = ReadPositive(root, "braking_deceleration", yaml_path);
  s.max_yaw_rate = ReadPositive(root, "max_yaw_rate", yaml_path);
  s.planning_forward_force = ReadPositive(root, "planning_forward_force", yaml_path);
  s.planning_reverse_force = ReadPositive(root, "planning_reverse_force", yaml_path);
  s.time_weight = ReadPositive(root, "time_weight", yaml_path);
  s.samples_per_piece = static_cast<int>(ReadPositive(root, "samples_per_piece", yaml_path));
  s.thrust_penalty_weight = ReadPositive(root, "thrust_penalty_weight", yaml_path);
  s.speed_penalty_weight = ReadPositive(root, "speed_penalty_weight", yaml_path);
  s.yaw_rate_penalty_weight = ReadPositive(root, "yaw_rate_penalty_weight", yaml_path);
  s.corridor_penalty_weight = ReadPositive(root, "corridor_penalty_weight", yaml_path);
  s.penalty_smoothing = ReadPositive(root, "penalty_smoothing", yaml_path);
  s.corridor_margin = ReadPositive(root, "corridor_margin", yaml_path);
  s.speed_margin_fraction = ReadPositive(root, "speed_margin_fraction", yaml_path);
  s.relative_cost_tolerance = ReadPositive(root, "relative_cost_tolerance", yaml_path);
  s.max_iterations = static_cast<int>(ReadPositive(root, "max_iterations", yaml_path));
  s.thrust_tolerance_fraction = ReadPositive(root, "thrust_tolerance_fraction", yaml_path);
  s.check_samples_per_piece =
    static_cast<int>(ReadPositive(root, "check_samples_per_piece", yaml_path));
  s.solve_deadline_ms = ReadPositive(root, "solve_deadline_ms", yaml_path);
  s.final_zone_distance = ReadPositive(root, "final_zone_distance", yaml_path);
  s.final_zone_speed = ReadPositive(root, "final_zone_speed", yaml_path);
  s.final_zone_buffer = ReadPositive(root, "final_zone_buffer", yaml_path);
  s.cap_phase_in_headroom = ReadPositive(root, "cap_phase_in_headroom", yaml_path);
  if (s.speed_margin_fraction >= 1.0) {
    throw std::runtime_error(yaml_path + ": speed_margin_fraction must be below 1");
  }
  return s;
}

void CheckPlanningLimits(
  const OptimizerSettings & settings,
  const std::string & allocator_yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(allocator_yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot read '" + allocator_yaml_path + "': " + error.what());
  }
  const YAML::Node parameters = root["thrust_allocator_node"]["ros__parameters"];
  const double forward = ReadPositive(parameters, "max_forward_force_n", allocator_yaml_path);
  const double reverse = ReadPositive(parameters, "max_reverse_force_n", allocator_yaml_path);
  if (!(settings.planning_forward_force > 0.0) || !(settings.planning_reverse_force > 0.0)) {
    throw std::runtime_error("planning thrust limits must be positive");
  }
  if (settings.planning_forward_force > forward || settings.planning_reverse_force > reverse) {
    char text[200];
    std::snprintf(
      text, sizeof(text),
      "planning limits +%.1f / -%.1f N exceed the allocator limits +%.1f / -%.1f N",
      settings.planning_forward_force, settings.planning_reverse_force, forward, reverse);
    throw std::runtime_error(text);
  }
}

double PolynomialTrajectory::TotalDuration() const
{
  double total = 0.0;
  for (const double d : durations) {
    total += d;
  }
  return total;
}

void PolynomialTrajectory::Evaluate(
  double t, Eigen::Vector3d * position, Eigen::Vector3d * velocity,
  Eigen::Vector3d * acceleration) const
{
  if (durations.empty()) {
    throw std::logic_error("PolynomialTrajectory::Evaluate on an empty trajectory");
  }
  std::size_t piece = 0;
  t = std::max(t, 0.0);
  while (piece + 1 < durations.size() && t > durations[piece]) {
    t -= durations[piece];
    ++piece;
  }
  t = std::min(t, durations[piece]);
  const Bases b = BasesAt(t);
  const Eigen::Matrix<double, 6, 3> & c = coefficients[piece];
  if (position) {*position = c.transpose() * b.position;}
  if (velocity) {*velocity = c.transpose() * b.velocity;}
  if (acceleration) {*acceleration = c.transpose() * b.acceleration;}
}

const char * PlanStatusToString(PlanStatus status)
{
  switch (status) {
    case PlanStatus::kSuccess:
      return "success";
    case PlanStatus::kInvalidInput:
      return "invalid input";
    case PlanStatus::kOptimizerFailed:
      return "optimizer failed";
    case PlanStatus::kInfeasible:
      return "infeasible";
    case PlanStatus::kTimeout:
      return "timeout";
  }
  return "unknown";
}

struct SurfaceTrajectoryOptimizer::Problem
{
  int pieces = 0;
  int xi_dim = 0;
  Eigen::Matrix3d head;                     // columns: position, velocity, acceleration
  Eigen::Matrix3d tail;
  Eigen::Vector2d goal_xy;
  std::vector<Eigen::MatrixX4d> polygons;   // polygon of each piece
  std::vector<Eigen::Matrix2Xd> overlaps;   // per inner waypoint: [v0, v1 - v0, ...]
  minco::MINCO_S3NU minco;

  // Work space for one cost evaluation.
  Eigen::Matrix3Xd points;
  Eigen::VectorXd times;
  Eigen::MatrixX3d grad_coefficients;
  Eigen::VectorXd grad_times;
  Eigen::Matrix3Xd grad_points;
  Eigen::VectorXd grad_times_total;
  Eigen::VectorXd margins;                  // one value per polytope face
  CostTerms terms;
  int evaluations = 0;
};

SurfaceTrajectoryOptimizer::SurfaceTrajectoryOptimizer(
  const rov_control::PlanarModel & model,
  const rov_control::AllocationMatrix & allocation_matrix, const OptimizerSettings & settings)
: model_(model),
  settings_(settings),
  polytope_(ComputeWrenchPolytope(
      allocation_matrix, settings.planning_forward_force, settings.planning_reverse_force))
{
}

SurfaceTrajectoryOptimizer::~SurfaceTrajectoryOptimizer() = default;

void SurfaceTrajectoryOptimizer::SetSpeedOptions(const SpeedOptions & speed)
{
  if (!(speed.max_speed > 0.0)) {
    throw std::invalid_argument("SpeedOptions::max_speed must be > 0");
  }
  speed_options_ = speed;
}

double SurfaceTrajectoryOptimizer::CruiseSpeed() const
{
  return std::min(settings_.cruise_speed, speed_options_.max_speed);
}

double SurfaceTrajectoryOptimizer::SpeedLimitSquared(
  double distance, double * derivative, double zone_buffer, double start_distance,
  double * start_derivative) const
{
  const double d = std::max(distance, 0.0);
  const double a = settings_.braking_deceleration;
  const double cruise = CruiseSpeed();
  double limit = cruise * cruise;
  double slope = 0.0;
  double start_slope = 0.0;
  // A cap below the start speed is phased in at a_b from the start.
  const double v0 = start_speed_ + settings_.cap_phase_in_headroom;
  const double phase_in = v0 * v0 - 2.0 * a * std::max(start_distance, 0.0);
  if (start_speed_ * start_speed_ > limit && phase_in > limit) {
    const double ceiling = settings_.cruise_speed * settings_.cruise_speed;
    if (phase_in < ceiling) {
      limit = phase_in;
      start_slope = -2.0 * a;
    } else {
      limit = std::max(limit, ceiling);
    }
  }
  if (speed_options_.final_zone) {
    const double v = settings_.final_zone_speed;
    const double beyond = d - (settings_.final_zone_distance + zone_buffer);
    const double zone = v * v + 2.0 * a * std::max(beyond, 0.0);
    if (zone < limit) {
      limit = zone;
      slope = beyond > 0.0 ? 2.0 * a : 0.0;
      start_slope = 0.0;
    }
  }
  if (2.0 * a * d < limit) {
    limit = 2.0 * a * d;
    slope = 2.0 * a;
    start_slope = 0.0;
  }
  if (derivative) {
    *derivative = slope;
  }
  if (start_derivative) {
    *start_derivative = start_slope;
  }
  return limit;
}

double SurfaceTrajectoryOptimizer::SpeedLimit(double distance, double start_distance) const
{
  return std::sqrt(SpeedLimitSquared(distance, nullptr, 0.0, start_distance));
}

bool SurfaceTrajectoryOptimizer::Setup(
  const StartState & start, const Eigen::Vector3d & goal, const Corridor & corridor,
  std::string * error)
{
  auto fail = [error](const std::string & text) {
      if (error) {*error = text;}
      return false;
    };
  const int pieces = static_cast<int>(corridor.polygons.size());
  if (pieces < 1) {
    return fail("the corridor has no polygons");
  }
  auto inside = [](const Eigen::MatrixX4d & polygon, const Eigen::Vector2d & p) {
      return (polygon.leftCols<2>() * p + polygon.col(3)).maxCoeff() <= 1e-9;
    };
  if (!inside(corridor.polygons.front(), start.position)) {
    return fail("the start is outside the first polygon");
  }
  if (!inside(corridor.polygons.back(), goal.head<2>())) {
    return fail("the goal is outside the last polygon");
  }

  auto problem = std::make_unique<Problem>();
  problem->pieces = pieces;
  problem->polygons = corridor.polygons;
  for (int i = 0; i < pieces; ++i) {
    if (PolygonVertices(corridor.polygons[i]).size() < 3) {
      return fail("polygon " + std::to_string(i) + " is empty or unbounded");
    }
  }
  // Corners of each overlap, stored as GCOPTER does: [v0, v1 - v0, ...].
  for (int i = 0; i + 1 < pieces; ++i) {
    Eigen::MatrixX4d both(
      corridor.polygons[i].rows() + corridor.polygons[i + 1].rows(), 4);
    both << corridor.polygons[i], corridor.polygons[i + 1];
    const std::vector<Eigen::Vector2d> corners = PolygonVertices(both);
    if (corners.size() < 3) {
      return fail(
        "polygons " + std::to_string(i) + " and " + std::to_string(i + 1) + " do not overlap");
    }
    Eigen::Matrix2Xd overlap(2, corners.size());
    overlap.col(0) = corners[0];
    for (std::size_t k = 1; k < corners.size(); ++k) {
      overlap.col(k) = corners[k] - corners[0];
    }
    problem->overlaps.push_back(overlap);
    problem->xi_dim += static_cast<int>(corners.size());
  }

  // Yaw: take the short way to the goal heading (continuous, unwrapped).
  const double goal_yaw = start.yaw + NormalizeAngle(goal(2) - start.yaw);
  problem->head << start.position.x(), start.velocity.x(), start.acceleration.x(),
    start.position.y(), start.velocity.y(), start.acceleration.y(),
    start.yaw, start.yaw_rate, start.yaw_acceleration;
  problem->tail << goal.x(), 0.0, 0.0, goal.y(), 0.0, 0.0, goal_yaw, 0.0, 0.0;
  problem->goal_xy = goal.head<2>();
  problem->margins.resize(polytope_.normals.rows());
  start_xy_ = start.position;
  start_speed_ = start.velocity.norm();
  problem->minco.setConditions(problem->head, problem->tail, pieces);
  problem->points.resize(3, pieces - 1);
  problem->times.resize(pieces);
  problem->grad_points.resize(3, pieces - 1);
  problem_ = std::move(problem);
  return true;
}

Eigen::VectorXd SurfaceTrajectoryOptimizer::InitialGuess() const
{
  const Problem & p = *problem_;
  const int n = p.pieces;
  Eigen::VectorXd x(n + p.xi_dim + n - 1);
  // xi: equal weights -> each inner waypoint starts at the overlap's centre.
  std::vector<Eigen::Vector2d> waypoints = {p.head.col(0).head<2>()};
  for (int i = 0, j = 0; i + 1 < n; ++i) {
    const int k = static_cast<int>(p.overlaps[i].cols());
    x.segment(n + j, k).setConstant(std::sqrt(1.0 / k));
    Eigen::Vector2d centre = p.overlaps[i].col(0);
    for (int c = 1; c < k; ++c) {
      centre += p.overlaps[i].col(c) / k;
    }
    waypoints.push_back(centre);
    j += k;
  }
  waypoints.push_back(p.goal_xy);

  // Yaw: blend from start to goal along the distance; times from distance
  // and yaw change at half the limits.
  std::vector<double> lengths;
  double total = 0.0;
  for (int i = 0; i < n; ++i) {
    lengths.push_back((waypoints[i + 1] - waypoints[i]).norm());
    total += lengths.back();
  }
  const double yaw_change = p.tail(2, 0) - p.head(2, 0);
  double covered = 0.0;
  for (int i = 0; i < n; ++i) {
    const double piece_yaw = std::abs(yaw_change) * (total > 0.0 ? lengths[i] / total : 1.0 / n);
    const double duration = std::max(
      {lengths[i] / (0.5 * CruiseSpeed()), piece_yaw / (0.5 * settings_.max_yaw_rate),
        0.5});
    x(i) = TauFromDuration(duration);
    covered += lengths[i];
    if (i + 1 < n) {
      x(n + p.xi_dim + i) =
        p.head(2, 0) + yaw_change * (total > 0.0 ? covered / total : (i + 1.0) / n);
    }
  }
  return x;
}

double SurfaceTrajectoryOptimizer::Cost(const Eigen::VectorXd & x, Eigen::VectorXd * gradient)
{
  Problem & p = *problem_;
  const int n = p.pieces;
  ++p.evaluations;
  const Eigen::VectorXd tau = x.head(n);
  const Eigen::VectorXd xi = x.segment(n, p.xi_dim);
  const Eigen::VectorXd yaw = x.tail(n - 1);

  // Decision variables -> durations and inner waypoints.
  for (int i = 0; i < n; ++i) {
    p.times(i) = DurationFromTau(tau(i));
  }
  for (int i = 0, j = 0; i + 1 < n; ++i) {
    const int k = static_cast<int>(p.overlaps[i].cols());
    const Eigen::VectorXd q = xi.segment(j, k).normalized().head(k - 1);
    p.points.col(i).head<2>() =
      p.overlaps[i].rightCols(k - 1) * q.cwiseProduct(q) + p.overlaps[i].col(0);
    p.points(2, i) = yaw(i);
    j += k;
  }

  // MINCO energy and its gradients.
  p.minco.setParameters(p.points, p.times);
  double energy = 0.0;
  p.minco.getEnergy(energy);
  p.minco.getEnergyPartialGradByCoeffs(p.grad_coefficients);
  p.minco.getEnergyPartialGradByTimes(p.grad_times);
  const Eigen::MatrixX3d & coefficients = p.minco.getCoeffs();

  // Penalties, integrated over each piece with the trapezoid rule (as in
  // gcopter.hpp attachPenaltyFunctional).
  CostTerms terms;
  terms.energy = energy;
  const OptimizerSettings & s = settings_;
  const int samples = s.samples_per_piece;
  const double speed_scale = (1.0 - s.speed_margin_fraction) * (1.0 - s.speed_margin_fraction);
  const double yaw_rate_squared = speed_scale * s.max_yaw_rate * s.max_yaw_rate;
  for (int i = 0; i < n; ++i) {
    const Eigen::Matrix<double, 6, 3> c = coefficients.block<6, 3>(6 * i, 0);
    const Eigen::MatrixX4d & polygon = p.polygons[i];
    const double step = p.times(i) / samples;
    for (int j = 0; j <= samples; ++j) {
      const Bases b = BasesAt(j * step);
      const Eigen::Vector3d pos = c.transpose() * b.position;
      const Eigen::Vector3d vel = c.transpose() * b.velocity;
      const Eigen::Vector3d acc = c.transpose() * b.acceleration;
      const Eigen::Vector3d jer = c.transpose() * b.jerk;
      Eigen::Vector3d grad_pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d grad_vel = Eigen::Vector3d::Zero();
      Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();
      double f = 0.0;
      double df = 0.0;
      CostTerms here;

      // Corridor: stay corridor_margin inside the piece's polygon.
      for (int k = 0; k < polygon.rows(); ++k) {
        const Eigen::Vector2d normal = polygon.block<1, 2>(k, 0).transpose();
        const double violation = normal.dot(pos.head<2>()) + polygon(k, 3) + s.corridor_margin;
        if (SmoothedL1(violation, s.penalty_smoothing, &f, &df)) {
          here.corridor += s.corridor_penalty_weight * f;
          grad_pos.head<2>() += s.corridor_penalty_weight * df * normal;
        }
      }

      // Speed: |v|^2 <= (1 - margin)^2 * v_max(d)^2 (SpeedLimitSquared).
      const Eigen::Vector2d to_goal = pos.head<2>() - p.goal_xy;
      const double distance = to_goal.norm();
      const Eigen::Vector2d from_start = pos.head<2>() - start_xy_;
      const double start_distance = from_start.norm();
      double limit_slope = 0.0;
      double start_slope = 0.0;
      const double scaled_limit = speed_scale * SpeedLimitSquared(
        distance, &limit_slope, s.final_zone_buffer, start_distance, &start_slope);
      // Aim one smoothing width further in: the smoothed-L1 penalty is weak for
      // its first penalty_smoothing (m^2/s^2) of violation, which at low speed
      // near the goal is more than the fractional margin.
      const double limit_squared = scaled_limit - s.penalty_smoothing;
      const double speed_violation = vel.head<2>().squaredNorm() - limit_squared;
      if (SmoothedL1(speed_violation, s.penalty_smoothing, &f, &df)) {
        here.speed += s.speed_penalty_weight * f;
        grad_vel.head<2>() += s.speed_penalty_weight * df * 2.0 * vel.head<2>();
        if (limit_slope > 0.0 && distance > 1e-9) {
          // The limit grows with the distance to the goal.
          grad_pos.head<2>() -= s.speed_penalty_weight * df * speed_scale * limit_slope *
            to_goal / distance;
        }
        if (start_slope != 0.0 && start_distance > 1e-9) {
          // The phase-in limit falls with the distance from the start.
          grad_pos.head<2>() -= s.speed_penalty_weight * df * speed_scale * start_slope *
            from_start / start_distance;
        }
      }

      // Yaw rate.
      if (SmoothedL1(
          vel(2) * vel(2) - (yaw_rate_squared - s.penalty_smoothing), s.penalty_smoothing,
          &f, &df))
      {
        here.yaw_rate += s.yaw_rate_penalty_weight * f;
        grad_vel(2) += s.yaw_rate_penalty_weight * df * 2.0 * vel(2);
      }

      // Thrust: sum over faces of max(0, H tau - h)^3.
      TrajectoryState state;
      state.velocity = vel.head<2>();
      state.acceleration = acc.head<2>();
      state.yaw = pos(2);
      state.yaw_rate = vel(2);
      state.yaw_acceleration = acc(2);
      // Wrench first; its gradients only where a face is violated (rare).
      Eigen::VectorXd & margins = p.margins;
      margins.noalias() = polytope_.normals * model_.Wrench(state);
      margins -= polytope_.offsets;
      Eigen::Vector3d grad_wrench = Eigen::Vector3d::Zero();
      for (int k = 0; k < margins.size(); ++k) {
        if (margins(k) > 0.0) {
          here.thrust += s.thrust_penalty_weight * margins(k) * margins(k) * margins(k);
          grad_wrench += s.thrust_penalty_weight * 3.0 * margins(k) * margins(k) *
            polytope_.normals.row(k).transpose();
        }
      }
      if (here.thrust > 0.0) {
        const RequiredWrench required = model_.Compute(state);
        grad_vel.head<2>() += required.d_velocity.transpose() * grad_wrench;
        grad_acc.head<2>() += required.d_acceleration.transpose() * grad_wrench;
        grad_pos(2) += required.d_yaw.dot(grad_wrench);
        grad_vel(2) += required.d_yaw_rate.dot(grad_wrench);
        grad_acc(2) += required.d_yaw_acceleration.dot(grad_wrench);
      }

      // Trapezoid rule; chain to the coefficients and the duration.
      const double penalty = here.corridor + here.speed + here.yaw_rate + here.thrust;
      const double node = (j == 0 || j == samples) ? 0.5 : 1.0;
      const double alpha = static_cast<double>(j) / samples;
      p.grad_coefficients.block<6, 3>(6 * i, 0) +=
        (b.position * grad_pos.transpose() + b.velocity * grad_vel.transpose() +
        b.acceleration * grad_acc.transpose()) * node * step;
      p.grad_times(i) += (grad_pos.dot(vel) + grad_vel.dot(acc) + grad_acc.dot(jer)) *
        alpha * node * step + node * penalty / samples;
      terms.corridor += node * step * here.corridor;
      terms.speed += node * step * here.speed;
      terms.yaw_rate += node * step * here.yaw_rate;
      terms.thrust += node * step * here.thrust;
    }
  }

  // Back to waypoints and durations, then to the decision variables.
  p.minco.propogateGrad(p.grad_coefficients, p.grad_times, p.grad_points, p.grad_times_total);
  terms.time = s.time_weight * p.times.sum();
  double cost = terms.energy + terms.time + terms.corridor + terms.speed + terms.yaw_rate +
    terms.thrust;
  p.grad_times_total.array() += s.time_weight;

  Eigen::VectorXd & g = *gradient;
  g.resize(x.size());
  for (int i = 0; i < n; ++i) {
    g(i) = GradientOfTau(tau(i), p.grad_times_total(i));
  }
  for (int i = 0, j = 0; i + 1 < n; ++i) {
    // gcopter.hpp backwardGradP and normRetrictionLayer.
    const int k = static_cast<int>(p.overlaps[i].cols());
    const Eigen::VectorXd q = xi.segment(j, k);
    const double inverse_norm = 1.0 / q.norm();
    const Eigen::VectorXd unit = q * inverse_norm;
    Eigen::VectorXd grad_q(k);
    grad_q.head(k - 1) =
      (p.overlaps[i].rightCols(k - 1).transpose() * p.grad_points.col(i).head<2>()).array() *
      unit.head(k - 1).array() * 2.0;
    grad_q(k - 1) = 0.0;
    g.segment(n + j, k) = (grad_q - unit * unit.dot(grad_q)) * inverse_norm;
    const double excess = q.squaredNorm() - 1.0;
    if (excess > 0.0) {
      cost += excess * excess * excess;
      g.segment(n + j, k) += 3.0 * excess * excess * 2.0 * q;
    }
    j += k;
  }
  g.tail(n - 1) = p.grad_points.row(2).transpose();
  p.terms = terms;
  return cost;
}

PolynomialTrajectory SurfaceTrajectoryOptimizer::TrajectoryFrom(const Eigen::VectorXd & x)
{
  Eigen::VectorXd unused;
  Cost(x, &unused);  // sets the MINCO parameters for x
  const Problem & p = *problem_;
  PolynomialTrajectory trajectory;
  const Eigen::MatrixX3d & coefficients = p.minco.getCoeffs();
  for (int i = 0; i < p.pieces; ++i) {
    trajectory.durations.push_back(p.times(i));
    trajectory.coefficients.push_back(coefficients.block<6, 3>(6 * i, 0));
  }
  return trajectory;
}

double SurfaceTrajectoryOptimizer::ThrustMargin(
  const PolynomialTrajectory & trajectory, double t) const
{
  Eigen::Vector3d pos, vel, acc;
  trajectory.Evaluate(t, &pos, &vel, &acc);
  TrajectoryState state;
  state.velocity = vel.head<2>();
  state.acceleration = acc.head<2>();
  state.yaw = pos(2);
  state.yaw_rate = vel(2);
  state.yaw_acceleration = acc(2);
  return polytope_.Margins(model_.Compute(state).wrench).maxCoeff();
}

ConstraintReport SurfaceTrajectoryOptimizer::Check(const PolynomialTrajectory & trajectory) const
{
  ConstraintReport report;
  report.corridor_m = -1e9;
  report.thrust_n = -1e9;
  report.speed_mps = -1e9;
  report.yaw_rate_radps = -1e9;
  const Problem & p = *problem_;
  double start_time = 0.0;
  for (std::size_t i = 0; i < trajectory.durations.size(); ++i) {
    const double duration = trajectory.durations[i];
    const Eigen::Matrix<double, 6, 3> & c = trajectory.coefficients[i];
    // Corridor, exactly: each edge's value along the piece is a quintic.
    const Eigen::MatrixX4d & polygon = p.polygons[i];
    for (int k = 0; k < polygon.rows(); ++k) {
      Eigen::VectorXd edge = c.leftCols<2>() * polygon.block<1, 2>(k, 0).transpose();
      edge(0) += polygon(k, 3);
      report.corridor_m = std::max(report.corridor_m, PolynomialMax(edge, duration));
    }
    // The limits, on dense samples.
    const int samples = settings_.check_samples_per_piece;
    for (int j = 0; j <= samples; ++j) {
      const double t = start_time + duration * j / samples;
      Eigen::Vector3d pos, vel;
      trajectory.Evaluate(t, &pos, &vel, nullptr);
      const double speed = vel.head<2>().norm();
      const double distance = (pos.head<2>() - p.goal_xy).norm();
      const double start_distance = (pos.head<2>() - start_xy_).norm();
      report.max_speed_mps = std::max(report.max_speed_mps, speed);
      report.speed_mps =
        std::max(report.speed_mps, speed - SpeedLimit(distance, start_distance));
      report.yaw_rate_radps =
        std::max(report.yaw_rate_radps, std::abs(vel(2)) - settings_.max_yaw_rate);
      report.thrust_n = std::max(report.thrust_n, ThrustMargin(trajectory, t));
    }
    start_time += duration;
  }
  return report;
}

int SurfaceTrajectoryOptimizer::Progress(
  void * instance, const Eigen::VectorXd &, const Eigen::VectorXd &, double, double, int, int)
{
  auto * self = static_cast<SurfaceTrajectoryOptimizer *>(instance);
  if (std::chrono::steady_clock::now() >= self->deadline_) {
    self->deadline_hit_ = true;
    return 1;  // non-zero: L-BFGS stops (LBFGSERR_CANCELED)
  }
  return 0;
}

std::optional<Eigen::VectorXd> SurfaceTrajectoryOptimizer::WarmGuess(
  const WarmStart & warm, std::string * why) const
{
  auto fail = [why](const std::string & reason) {
      if (why) {*why = reason;}
      return std::optional<Eigen::VectorXd>();
    };
  const Problem & p = *problem_;
  const PolynomialTrajectory & old = warm.trajectory;
  if (old.durations.empty()) {
    return fail("the previous trajectory is empty");
  }
  const double total = old.TotalDuration();
  const double t_start = std::clamp(warm.elapsed, 0.0, total);
  const double remaining = total - t_start;
  if (!std::isfinite(warm.elapsed) || remaining < 0.2) {
    return fail("less than 0.2 s of the previous trajectory remains");
  }

  // The old yaw may differ from the new start yaw by whole turns.
  Eigen::Vector3d pos;
  old.Evaluate(t_start, &pos, nullptr, nullptr);
  const double turns = std::round((p.head(2, 0) - pos(2)) / (2.0 * M_PI));
  const double yaw_offset = 2.0 * M_PI * turns;

  const int n = p.pieces;
  Eigen::VectorXd x(n + p.xi_dim + n - 1);
  const double step = std::min(0.01, remaining / 1000.0);
  auto inside_both = [&p](int i, const Eigen::Vector2d & q) {
      for (const Eigen::MatrixX4d * polygon : {&p.polygons[i], &p.polygons[i + 1]}) {
        if ((polygon->leftCols<2>() * q + polygon->col(3)).maxCoeff() > -1e-3) {
          return false;
        }
      }
      return true;
    };

  double piece_start = t_start;
  double t = t_start;
  for (int i = 0, j = 0; i + 1 < n; ++i) {
    // The stretch of the remaining trajectory inside overlap i; take its middle.
    double entry = -1.0;
    double exit = -1.0;
    for (; t <= total; t += step) {
      old.Evaluate(t, &pos, nullptr, nullptr);
      if (inside_both(i, pos.head<2>())) {
        if (entry < 0.0) {entry = t;}
        exit = t;
      } else if (entry >= 0.0) {
        break;
      }
    }
    if (entry < 0.0) {
      return fail("the remaining trajectory does not pass overlap " + std::to_string(i));
    }
    const double t_waypoint = 0.5 * (entry + exit);
    old.Evaluate(t_waypoint, &pos, nullptr, nullptr);
    const Eigen::Vector2d q = pos.head<2>();

    // Convex weights of q over the overlap's corners (fan from corner 0),
    // mixed with a little of the centre so every weight stays positive.
    const Eigen::Matrix2Xd & overlap = p.overlaps[i];
    const int k = static_cast<int>(overlap.cols());
    std::vector<Eigen::Vector2d> corners = {overlap.col(0)};
    for (int c = 1; c < k; ++c) {
      corners.push_back(overlap.col(0) + overlap.col(c));
    }
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(k);
    bool found = false;
    for (int c = 1; c + 1 < k && !found; ++c) {
      Eigen::Matrix2d edges;
      edges << corners[c] - corners[0], corners[c + 1] - corners[0];
      const Eigen::Vector2d b = edges.inverse() * (q - corners[0]);
      if (b.minCoeff() >= -1e-9 && b.sum() <= 1.0 + 1e-9) {
        weights(0) = 1.0 - b.sum();
        weights(c) = b(0);
        weights(c + 1) = b(1);
        found = true;
      }
    }
    if (!found) {
      return fail("waypoint " + std::to_string(i) + " is not inside its overlap");
    }
    weights = 0.95 * weights.cwiseMax(0.0) + Eigen::VectorXd::Constant(k, 0.05 / k);
    // q = v0 + sum_c r_c^2 (v_c - v0): r_c = sqrt(weight of corner c), and the
    // last (slack) entry carries the weight of corner 0.
    for (int c = 1; c < k; ++c) {
      x(n + j + c - 1) = std::sqrt(weights(c));
    }
    x(n + j + k - 1) = std::sqrt(weights(0));
    j += k;

    x(n + p.xi_dim + i) = pos(2) + yaw_offset;
    x(i) = TauFromDuration(std::max(t_waypoint - piece_start, 0.05));
    piece_start = t_waypoint;
    t = t_waypoint + step;
  }
  x(n - 1) = TauFromDuration(std::max(total - piece_start, 0.05));
  return x;
}

PlanResult SurfaceTrajectoryOptimizer::Plan(
  const StartState & start, const Eigen::Vector3d & goal, const Corridor & corridor,
  const WarmStart * warm, const SpeedOptions & speed)
{
  const auto clock_start = std::chrono::steady_clock::now();
  SetSpeedOptions(speed);
  deadline_ = clock_start + std::chrono::microseconds(
    static_cast<int64_t>(settings_.solve_deadline_ms * 1000.0));
  deadline_hit_ = false;
  PlanResult result;
  auto elapsed_ms = [&clock_start]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - clock_start).count();
    };
  std::string error;
  if (!Setup(start, goal, corridor, &error)) {
    result.status = PlanStatus::kInvalidInput;
    result.message = error;
    result.total_time_ms = elapsed_ms();
    return result;
  }

  // Warm start if it maps onto this problem, otherwise cold (same deadline).
  Eigen::VectorXd x;
  if (warm != nullptr) {
    std::optional<Eigen::VectorXd> guess = WarmGuess(*warm, &result.warm_start_note);
    if (guess) {
      x = std::move(*guess);
      result.warm_started = true;
    }
  }
  if (!result.warm_started) {
    x = InitialGuess();
  }

  lbfgs::lbfgs_parameter_t parameters;
  parameters.mem_size = 256;  // as gcopter.hpp
  parameters.past = 3;
  parameters.min_step = 1.0e-32;
  parameters.g_epsilon = 0.0;
  parameters.delta = settings_.relative_cost_tolerance;
  parameters.max_iterations = settings_.max_iterations;
  double cost = 0.0;
  int code = 0;
  if (std::chrono::steady_clock::now() >= deadline_) {
    deadline_hit_ = true;  // the setup alone used the whole budget
  } else {
    code = lbfgs::lbfgs_optimize(x, cost, &LbfgsCost, nullptr, &Progress, this, parameters);
  }
  result.evaluations = problem_->evaluations;
  result.solve_time_ms = elapsed_ms();
  // Keep the last iterate even if L-BFGS stopped with an error, so the
  // result can be inspected and its checks reported.
  result.trajectory = TrajectoryFrom(x);
  result.cost = cost;
  result.cost_terms = problem_->terms;
  result.report = Check(result.trajectory);

  // Report every failed check.
  const ConstraintReport & r = result.report;
  const double thrust_tolerance =
    settings_.thrust_tolerance_fraction * settings_.planning_forward_force;
  std::string failures;
  char text[120];
  if (r.corridor_m > 1e-9) {
    std::snprintf(text, sizeof(text), "corridor exceeded by %.3f m; ", r.corridor_m);
    failures += text;
  }
  if (r.thrust_n > thrust_tolerance) {
    std::snprintf(
      text, sizeof(text), "thrust exceeded by %.2f N (allowed %.2f N); ", r.thrust_n,
      thrust_tolerance);
    failures += text;
  }
  // 1 mm/s: rounding in the sampled check, far below any speed that matters
  // (the matrix run victim_near_wall failed once on a 0.000 m/s excess).
  constexpr double kSpeedToleranceMps = 1e-3;
  if (r.speed_mps > kSpeedToleranceMps) {
    std::snprintf(text, sizeof(text), "speed above v_max(d) by %.3f m/s; ", r.speed_mps);
    failures += text;
  }
  if (r.yaw_rate_radps > 1e-9) {
    std::snprintf(text, sizeof(text), "yaw rate above limit by %.3f rad/s; ", r.yaw_rate_radps);
    failures += text;
  }
  // The deadline covers the whole call: the validation above must also have
  // finished in time, or the result is a timeout (and must not be used).
  result.total_time_ms = elapsed_ms();
  const bool validation_late = std::chrono::steady_clock::now() >= deadline_;
  if (deadline_hit_ || validation_late) {
    char deadline[200];
    std::snprintf(
      deadline, sizeof(deadline), "solve deadline %.0f ms reached %s (optimization %.1f ms, "
      "total %.1f ms); ", settings_.solve_deadline_ms,
      deadline_hit_ ? "during optimization" : "during validation", result.solve_time_ms,
      result.total_time_ms);
    result.status = PlanStatus::kTimeout;
    result.message = deadline + (failures.empty() ? std::string(
        "the last iterate passes the checks but is not used") : failures);
    return result;
  }
  // A line search that cannot make progress is how a converged L-BFGS run
  // often ends; the result is used only if every check passes.
  const bool line_search_stop = code == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
    code == lbfgs::LBFGSERR_MINIMUMSTEP;
  if (code < 0 && line_search_stop && failures.empty()) {
    result.status = PlanStatus::kSuccess;
    result.message = std::string("all checks passed (L-BFGS stopped: ") +
      lbfgs::lbfgs_strerror(code) + ")";
    return result;
  }
  if (code < 0) {
    result.status = PlanStatus::kOptimizerFailed;
    result.message = std::string("L-BFGS: ") + lbfgs::lbfgs_strerror(code) + "; " +
      (failures.empty() ? "the last iterate passes all checks" : failures);
    return result;
  }
  result.status = failures.empty() ? PlanStatus::kSuccess : PlanStatus::kInfeasible;
  result.message = failures.empty() ? "all checks passed" : failures;
  return result;
}

}  // namespace rov_planning
