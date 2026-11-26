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

// MINCO trajectory optimizer for the surface approach (design section 3,
// items 4-5; implementation step 7). No ROS.
//
// The trajectory is a MINCO (s = 3, quintic pieces) in (x, y, yaw), one piece
// per corridor polygon, structured after GCOPTER's gcopter.hpp:
//   decision variables x = [tau, xi, yaw_inner]
//     tau        piece durations through GCOPTER's smooth map T(tau) > 0;
//     xi         each inner (x, y) waypoint as a weighted mix of the corners
//                of the overlap of two neighbouring polygons, so it is
//                always inside both ("constraint elimination", hard);
//     yaw_inner  inner yaw waypoints, free: yaw is not part of the corridor.
//   cost = integral |jerk|^2 (MINCO energy) + time_weight * T_total
//        + integral of the penalties over each piece (trapezoid rule with
//          samples_per_piece intervals):
//     thrust     thrust_penalty_weight * sum over faces max(0, H tau - h)^3,
//                tau from the shared model (RovModel) at every sample;
//     speed      |v|^2 above v_max(d)^2,
//                v_max(d) = min(v_c, sqrt(2 a_b d), v_fz(d)), d = distance to
//                the goal (braking curve), v_c = min(cruise_speed,
//                SpeedOptions::max_speed); with SpeedOptions::final_zone
//                (surface victim, design 3 item 5) also
//                v_fz(d) = sqrt(v_final^2 + 2 a_b max(0, d - d_final)): at most
//                v_final inside d_final, and the braking curve that reaches
//                v_final at d_final outside it;
//                if the start speed v0 is above v_c (a lower cap arrived while
//                moving), v_c is phased in along the braking envelope from
//                the start: v_c(s) = max(v_c, sqrt((v0 + h)^2 - 2 a_b s))
//                (capped at cruise_speed), s = distance from the start, h =
//                cap_phase_in_headroom (a smooth start cannot brake at a_b at
//                once);
//     yaw rate   r^2 above max_yaw_rate^2;
//     corridor   sample (x, y) outside its polygon shrunk by corridor_margin.
//   The speed, yaw-rate and corridor penalties use GCOPTER's smoothed L1
//   and aim slightly inside the limits (speed_margin_fraction,
//   corridor_margin; speed^2 and yaw rate^2 also one penalty_smoothing
//   further in, because the smoothed L1 is weak over that width), so the
//   checked limits below hold.
//   Solved with L-BFGS (lbfgs.hpp).
// Start: given position, velocity, acceleration, yaw, yaw rate and yaw
// acceleration. End: at the goal pose (x, y, yaw), at rest (v = 0, a = 0).
//
// After solving, the result is checked and every violation is reported:
//   - corridor: each piece's (x, y) polynomial against every edge of its
//     polygon, exactly (largest value of the polynomial on the piece);
//   - thrust (tolerance thrust_tolerance_fraction * planning_forward_force),
//     speed against v_max(d) and yaw rate, on check_samples_per_piece samples.

#ifndef ROV_PLANNING__SURFACE_TRAJ_OPT_HPP_
#define ROV_PLANNING__SURFACE_TRAJ_OPT_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "rov_control/planar_model.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_planning/corridor_2d.hpp"
#include "rov_planning/rov_model.hpp"
#include "rov_planning/wrench_polytope.hpp"

namespace rov_planning
{

// Parameters (config/surface_trajectory_optimizer.yaml). Defaults follow
// design section 6.5; see the YAML file for units and sources.
struct OptimizerSettings
{
  double cruise_speed = 1.5;             // v_cruise, m/s
  double braking_deceleration = 1.0;     // a_b, m/s^2
  double max_yaw_rate = 1.5;             // omega_max, rad/s
  double planning_forward_force = 110.0;  // f_plan, N per thruster
  double planning_reverse_force = 70.0;   // N per thruster (provisional)
  double time_weight = 1024.0;           // rho_T
  int samples_per_piece = 16;            // K
  double thrust_penalty_weight = 1.0e4;  // w_tau
  double speed_penalty_weight = 1.0e5;   // w_v (tuned)
  double yaw_rate_penalty_weight = 1.0e4;
  double corridor_penalty_weight = 1.0e4;
  double penalty_smoothing = 1.0e-2;     // smoothed L1 width
  double corridor_margin = 0.05;         // m
  double speed_margin_fraction = 0.03;   // aim (1 - f) * limit for speed and yaw rate
  double relative_cost_tolerance = 1.0e-5;
  int max_iterations = 1000;
  double thrust_tolerance_fraction = 0.02;  // of planning_forward_force
  int check_samples_per_piece = 64;
  double solve_deadline_ms = 2000.0;  // whole Plan() call, incl. warm mapping and checks
  double final_zone_distance = 2.0;   // d_final, m (surface victim, design 6.5)
  double final_zone_speed = 0.3;      // v_final, m/s (surface victim, design 6.5)
  // m: the speed PENALTY starts the zone this much farther out, so that the
  // smooth trajectory is down to v_final at d_final (the kink in v_max(d)
  // falls between penalty samples otherwise). The checks use d_final.
  double final_zone_buffer = 0.25;
  double cap_phase_in_headroom = 0.1;  // m/s, see the speed penalty above (provisional)
};

// Per-plan speed options (the defaults reproduce the settings alone).
struct SpeedOptions
{
  // m/s: extra cap on the cruise speed (safety speed limits, 0.7 v_cruise
  // retries). Infinity: none. Must be > 0.
  double max_speed = std::numeric_limits<double>::infinity();
  bool final_zone = false;  // surface victim: v_final within d_final of the goal
};

// Read the settings from a YAML file. Throws std::runtime_error on a missing
// or invalid value.
OptimizerSettings LoadOptimizerSettings(const std::string & yaml_path);

// Throw std::runtime_error unless the planning thrust limits are positive and
// within the allocator's limits (max_forward_force_n / max_reverse_force_n in
// rov_control/config/thrust_allocator.yaml), so planned thrust is always
// something the allocator may command.
void CheckPlanningLimits(
  const OptimizerSettings & settings, const std::string & allocator_yaml_path);

// Quintic pieces in (x, y, yaw). coefficients[i].row(k) multiplies t^k,
// t the time since the start of piece i.
struct PolynomialTrajectory
{
  std::vector<double> durations;
  std::vector<Eigen::Matrix<double, 6, 3>> coefficients;

  double TotalDuration() const;
  // Position (x, y, yaw), velocity and acceleration at time t (clamped to
  // the trajectory).
  void Evaluate(
    double t, Eigen::Vector3d * position, Eigen::Vector3d * velocity,
    Eigen::Vector3d * acceleration) const;
};

struct StartState
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double yaw_acceleration = 0.0;
};

enum class PlanStatus
{
  kSuccess,          // optimized and every check passed
  kInvalidInput,     // corridor / start / goal not usable
  kOptimizerFailed,  // L-BFGS stopped with an error (the last iterate is still reported)
  kInfeasible,       // optimized, but a check failed (see the report)
  kTimeout,          // the solve deadline was reached (never dispatch the result)
};

const char * PlanStatusToString(PlanStatus status);

// Largest violation of each limit over the result (0 or negative = inside).
struct ConstraintReport
{
  double corridor_m = 0.0;     // outside its polygon, m
  double thrust_n = 0.0;       // max(H tau - h), N (or N m along the face normal)
  double speed_mps = 0.0;      // |v| - v_max(d)
  double yaw_rate_radps = 0.0;  // |r| - max_yaw_rate
  double max_speed_mps = 0.0;  // highest speed reached
};

// Cost terms at the solution.
struct CostTerms
{
  double energy = 0.0;        // integral |jerk|^2
  double time = 0.0;          // time_weight * T
  double thrust = 0.0;
  double speed = 0.0;
  double yaw_rate = 0.0;
  double corridor = 0.0;
};

// A previous feasible trajectory to start from (warm start). `elapsed` is
// the time already flown along it (s); only the remaining part is used.
struct WarmStart
{
  PolynomialTrajectory trajectory;
  double elapsed = 0.0;
};

struct PlanResult
{
  PlanStatus status = PlanStatus::kInvalidInput;
  std::string message;
  PolynomialTrajectory trajectory;
  ConstraintReport report;
  CostTerms cost_terms;
  double cost = 0.0;
  int evaluations = 0;        // cost function calls (L-BFGS does not report iterations)
  double solve_time_ms = 0.0;  // setup + initial guess + L-BFGS
  double total_time_ms = 0.0;  // the whole Plan() call, including the checks
  bool warm_started = false;   // the initial guess came from the warm start
  std::string warm_start_note;  // why a requested warm start was not used
};

class SurfaceTrajectoryOptimizer
{
public:
  // Builds the wrench polytope from the settings' planning thrust limits.
  SurfaceTrajectoryOptimizer(
    const rov_control::PlanarModel & model,
    const rov_control::AllocationMatrix & allocation_matrix, const OptimizerSettings & settings);
  ~SurfaceTrajectoryOptimizer();

  // goal: (x, y, yaw). The start and goal must lie in the first and last
  // polygons. Returns the result with status and report.
  //
  // warm (optional): initialize from the remaining part of a previous
  // feasible trajectory, fitted to this corridor, start and goal: the inner
  // waypoints are where that trajectory passes through each polygon overlap
  // (in order), with its timing and yaw. If it cannot be mapped (e.g. no
  // remaining time, or it misses an overlap), a cold start is used within
  // the same deadline and warm_start_note says why.
  //
  // Deadline: solve_deadline_ms from the start of the call, covering setup,
  // optimization and validation. L-BFGS is stopped cooperatively once it
  // passes; and if validation finishes after it, the result is kTimeout too,
  // even when every check passed.
  // speed: per-plan speed options; they also apply to Check() and
  // SpeedLimit() until the next Plan() or SetSpeedOptions().
  PlanResult Plan(
    const StartState & start, const Eigen::Vector3d & goal, const Corridor & corridor,
    const WarmStart * warm = nullptr, const SpeedOptions & speed = SpeedOptions());
  void SetSpeedOptions(const SpeedOptions & speed);
  // Cruise speed in use: min(cruise_speed, max_speed).
  double CruiseSpeed() const;

  // Lower-level access (used by the tests): prepare a problem, then
  // evaluate the cost and its gradient at any decision vector.
  bool Setup(
    const StartState & start, const Eigen::Vector3d & goal, const Corridor & corridor,
    std::string * error);
  Eigen::VectorXd InitialGuess() const;
  double Cost(const Eigen::VectorXd & x, Eigen::VectorXd * gradient);

  // Initial decision vector from a warm start, or empty with the reason.
  // (Needs Setup() first; public for the tests.)
  std::optional<Eigen::VectorXd> WarmGuess(const WarmStart & warm, std::string * why) const;

  // Check a trajectory against the corridor and the limits.
  ConstraintReport Check(const PolynomialTrajectory & trajectory) const;

  // v_max(d) at distance d from the goal (with the current speed options),
  // and start_distance from the start (infinity: far from it).
  double SpeedLimit(
    double distance,
    double start_distance = std::numeric_limits<double>::infinity()) const;
  // v_max^2 and its derivatives with respect to d and to the distance from
  // the start; `zone_buffer` (m) moves the final-zone edge outwards (penalty
  // only).
  double SpeedLimitSquared(
    double distance, double * derivative, double zone_buffer = 0.0,
    double start_distance = std::numeric_limits<double>::infinity(),
    double * start_derivative = nullptr) const;
  // Largest H tau - h for the state at time t of the trajectory.
  double ThrustMargin(const PolynomialTrajectory & trajectory, double t) const;

  const OptimizerSettings & settings() const {return settings_;}
  const WrenchPolytope & polytope() const {return polytope_;}

private:
  struct Problem;  // MINCO, corridor corners, and the current problem data

  PolynomialTrajectory TrajectoryFrom(const Eigen::VectorXd & x);
  // L-BFGS progress callback: non-zero stops the solve at the deadline.
  static int Progress(
    void * instance, const Eigen::VectorXd & x, const Eigen::VectorXd & g, double fx,
    double step, int k, int ls);

  RovModel model_;
  OptimizerSettings settings_;
  SpeedOptions speed_options_;
  double start_speed_ = 0.0;                          // |v| at the start (Setup)
  Eigen::Vector2d start_xy_ = Eigen::Vector2d::Zero();  // start position (Setup)
  WrenchPolytope polytope_;
  std::unique_ptr<Problem> problem_;
  std::chrono::steady_clock::time_point deadline_;
  bool deadline_hit_ = false;
};

}  // namespace rov_planning

#endif  // ROV_PLANNING__SURFACE_TRAJ_OPT_HPP_
