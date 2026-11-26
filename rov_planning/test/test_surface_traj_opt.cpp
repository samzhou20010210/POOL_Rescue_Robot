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

// Offline tests of the MINCO surface trajectory optimizer (step 7).
// Set ROV_PLANNING_PLOT_DIR to also write CSV files for
// scripts/plot_surface_trajectory.py.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rov_control/planar_model.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_planning/corridor_2d.hpp"
#include "rov_planning/surface_traj_opt.hpp"
#include "rov_simulation/planar_dynamics.hpp"

namespace rov_planning
{
namespace
{

struct Case
{
  std::string name;
  OccupancyGrid2D grid;
  std::vector<Eigen::Vector2d> path;
  double goal_yaw = 0.0;
};

OccupancyGrid2D EmptyGrid(double width_m, double height_m)
{
  OccupancyGrid2D grid;
  grid.width = static_cast<int>(std::lround(width_m / grid.resolution));
  grid.height = static_cast<int>(std::lround(height_m / grid.resolution));
  grid.data.assign(grid.width * grid.height, 0);
  return grid;
}

void Block(OccupancyGrid2D * grid, double x0, double y0, double x1, double y1)
{
  for (int y = 0; y < grid->height; ++y) {
    for (int x = 0; x < grid->width; ++x) {
      const double cx = (x + 0.5) * grid->resolution;
      const double cy = (y + 0.5) * grid->resolution;
      if (cx > x0 && cx < x1 && cy > y0 && cy < y1) {
        grid->data[y * grid->width + x] = 100;
      }
    }
  }
}

// Open 50 m straight.
Case Straight()
{
  Case c{"straight", EmptyGrid(60.0, 20.0), {{5.0, 10.0}, {55.0, 10.0}}, 0.0};
  return c;
}

// L-turn around a wall (wall x 6..14, y 0..6), finishing facing +y.
Case LTurn()
{
  Case c{"l_turn", EmptyGrid(20.0, 14.0), {{2.0, 3.0}, {3.5, 3.0}, {3.5, 9.0}, {17.0, 9.0}},
    M_PI / 2.0};
  Block(&c.grid, 6.0, 0.0, 14.0, 6.0);
  return c;
}

// Gap of 1.2 m between two obstacles (centre line y = 5).
Case NarrowGap()
{
  Case c{"narrow_gap", EmptyGrid(16.0, 10.0), {{2.0, 5.0}, {14.0, 5.0}}, 0.0};
  Block(&c.grid, 7.0, 0.0, 9.0, 4.4);
  Block(&c.grid, 7.0, 5.6, 9.0, 10.0);
  return c;
}

class SurfaceTrajOptTest : public ::testing::Test
{
protected:
  SurfaceTrajOptTest()
  : model_(rov_control::LoadHydrodynamicParams(HYDRODYNAMICS_V4_YAML)),
    allocation_(rov_control::BuildAllocationMatrix(
        rov_control::LoadThrusterLayout(THRUSTERS_V4_YAML))),
    settings_(LoadOptimizerSettings(OPTIMIZER_YAML))
  {
    corridor_settings_.clearance = 0.3;
  }

  Corridor CorridorFor(const Case & c) const
  {
    return BuildCorridor(c.path, c.grid, corridor_settings_);
  }

  // Plan a case and check every result requirement of step 7.
  PlanResult PlanAndCheck(const Case & c)
  {
    const Corridor corridor = CorridorFor(c);
    SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
    StartState start;
    start.position = c.path.front();
    const Eigen::Vector3d goal(c.path.back().x(), c.path.back().y(), c.goal_yaw);
    const PlanResult result = optimizer.Plan(start, goal, corridor);

    EXPECT_EQ(result.status, PlanStatus::kSuccess) << c.name << ": " << result.message;
    if (result.trajectory.durations.empty()) {
      ADD_FAILURE() << c.name << ": no trajectory";
      return result;
    }
    // Same tolerances as the optimizer's own checks (1e-9 for rounding).
    const ConstraintReport & r = result.report;
    EXPECT_LE(r.corridor_m, 1e-9) << c.name;
    EXPECT_LE(r.thrust_n, 0.02 * settings_.planning_forward_force) << c.name;
    EXPECT_LE(r.speed_mps, 1e-9) << c.name;
    EXPECT_LE(r.yaw_rate_radps, 1e-9) << c.name;
    // Terminal rest at the goal pose; start at the start.
    const double end = result.trajectory.TotalDuration();
    Eigen::Vector3d pos, vel, acc;
    result.trajectory.Evaluate(end, &pos, &vel, &acc);
    EXPECT_LT((pos.head<2>() - goal.head<2>()).norm(), 1e-6) << c.name;
    EXPECT_NEAR(std::remainder(pos(2) - goal(2), 2.0 * M_PI), 0.0, 1e-6) << c.name;
    EXPECT_LT(vel.norm(), 1e-6) << c.name;
    EXPECT_LT(acc.norm(), 1e-6) << c.name;
    result.trajectory.Evaluate(0.0, &pos, &vel, &acc);
    EXPECT_LT((pos.head<2>() - start.position).norm(), 1e-6) << c.name;

    std::printf(
      "[ %-10s ] %2zu pieces, T %.2f s, max speed %.3f m/s, solve %.1f ms (%d evals); "
      "corridor %+.4f m, thrust %+.3f N, speed %+.4f m/s, yaw rate %+.4f rad/s\n",
      c.name.c_str(), result.trajectory.durations.size(), end, r.max_speed_mps,
      result.solve_time_ms, result.evaluations, r.corridor_m, r.thrust_n, r.speed_mps,
      r.yaw_rate_radps);
    WritePlotFiles(c.name, optimizer, corridor, result, goal);
    return result;
  }

  // Optional CSV output for scripts/plot_surface_trajectory.py.
  static void WritePlotFiles(
    const std::string & name, const SurfaceTrajectoryOptimizer & optimizer,
    const Corridor & corridor, const PlanResult & result, const Eigen::Vector3d & goal)
  {
    const char * directory = std::getenv("ROV_PLANNING_PLOT_DIR");
    if (directory == nullptr || result.trajectory.durations.empty()) {
      return;
    }
    std::ofstream samples(std::string(directory) + "/" + name + "_trajectory.csv");
    samples << "t,x,y,yaw,speed,v_max,thrust_margin\n";
    const double end = result.trajectory.TotalDuration();
    for (double t = 0.0; t <= end + 1e-9; t += 0.02) {
      Eigen::Vector3d pos, vel;
      result.trajectory.Evaluate(t, &pos, &vel, nullptr);
      samples << t << "," << pos.x() << "," << pos.y() << "," << pos(2) << ","
              << vel.head<2>().norm() << ","
              << optimizer.SpeedLimit((pos.head<2>() - goal.head<2>()).norm()) << ","
              << optimizer.ThrustMargin(result.trajectory, t) << "\n";
    }
    std::ofstream polygons(std::string(directory) + "/" + name + "_corridor.csv");
    polygons << "polygon,x,y\n";
    for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
      for (const Eigen::Vector2d & v : PolygonVertices(corridor.polygons[i])) {
        polygons << i << "," << v.x() << "," << v.y() << "\n";
      }
    }
  }

  rov_control::PlanarModel model_;
  rov_control::AllocationMatrix allocation_;
  OptimizerSettings settings_;
  CorridorSettings corridor_settings_;
};

TEST_F(SurfaceTrajOptTest, PlanningLimitsStayWithinAllocatorLimits)
{
  EXPECT_NO_THROW(CheckPlanningLimits(settings_, ALLOCATOR_YAML));
  OptimizerSettings too_strong = settings_;
  too_strong.planning_forward_force = 150.0;  // allocator allows 140 N
  EXPECT_THROW(CheckPlanningLimits(too_strong, ALLOCATOR_YAML), std::runtime_error);
  too_strong = settings_;
  too_strong.planning_reverse_force = 120.0;  // allocator allows 100 N
  EXPECT_THROW(CheckPlanningLimits(too_strong, ALLOCATOR_YAML), std::runtime_error);
}

// The full cost gradient (MINCO energy, time, every penalty, the waypoint
// and time maps) against central finite differences, at a point where the
// thrust, speed and corridor penalties are all active.
TEST_F(SurfaceTrajOptTest, CostGradientMatchesFiniteDifferences)
{
  const Case c = LTurn();
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  StartState start;
  start.position = c.path.front();
  std::string error;
  ASSERT_TRUE(
    optimizer.Setup(start, Eigen::Vector3d(17.0, 9.0, M_PI / 2.0), CorridorFor(c), &error))
    << error;
  Eigen::VectorXd x = optimizer.InitialGuess();
  const int pieces = static_cast<int>(CorridorFor(c).polygons.size());
  x.head(pieces).array() -= 1.2;  // much too fast: thrust and speed penalties active
  for (int i = pieces; i < x.size(); ++i) {
    x(i) += 0.05 * std::sin(1.7 * i);  // move waypoints and yaw off the initial guess
  }

  Eigen::VectorXd analytic;
  optimizer.Cost(x, &analytic);
  Eigen::VectorXd unused;
  Eigen::VectorXd numeric(x.size());
  for (int i = 0; i < x.size(); ++i) {
    const double h = 1e-6 * std::max(1.0, std::abs(x(i)));
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += h;
    minus(i) -= h;
    numeric(i) = (optimizer.Cost(plus, &unused) - optimizer.Cost(minus, &unused)) / (2.0 * h);
  }
  const double relative_error = (analytic - numeric).norm() / numeric.norm();
  std::printf("[ gradient   ] %ld variables, relative error %.2e\n", x.size(), relative_error);
  EXPECT_LT(relative_error, 1e-4);
}

TEST_F(SurfaceTrajOptTest, StraightFiftyMetres)
{
  const PlanResult result = PlanAndCheck(Straight());
  EXPECT_LE(result.trajectory.TotalDuration(), 40.0);  // design: 50 m in <= 40 s
}

TEST_F(SurfaceTrajOptTest, LTurnAroundAWall)
{
  PlanAndCheck(LTurn());
}

TEST_F(SurfaceTrajOptTest, NarrowGap)
{
  PlanAndCheck(NarrowGap());
}

// Starting at 1.5 m/s only 1 m before the goal breaks the braking curve
// (v_max(1 m) = 1.41 m/s) at once: the result must say so, not hide it.
TEST_F(SurfaceTrajOptTest, InfeasibleRequestIsReported)
{
  Case c = Straight();
  c.path = {{5.0, 10.0}, {6.0, 10.0}};
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  StartState start;
  start.position = c.path.front();
  start.velocity = Eigen::Vector2d(1.5, 0.0);
  const PlanResult result =
    optimizer.Plan(start, Eigen::Vector3d(6.0, 10.0, 0.0), CorridorFor(c));
  EXPECT_EQ(result.status, PlanStatus::kInfeasible);
  EXPECT_GT(result.report.speed_mps, 0.0);
  EXPECT_NE(result.message.find("speed above v_max"), std::string::npos) << result.message;

  // A goal outside the corridor is invalid input.
  const PlanResult invalid =
    optimizer.Plan(StartState{}, Eigen::Vector3d(6.0, 10.0, 0.0), CorridorFor(c));
  EXPECT_EQ(invalid.status, PlanStatus::kInvalidInput);
}

// Feedforward only, no feedback: the straight trajectory's required wrench,
// allocated to thrusters and applied to the lightweight model (step 3, with
// thrust lag), must end within 0.5 m of the goal.
TEST_F(SurfaceTrajOptTest, StraightFeedforwardOnLightweightModelEndsNearGoal)
{
  const Case c = Straight();
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  StartState start;
  start.position = c.path.front();
  const Eigen::Vector3d goal(55.0, 10.0, 0.0);
  const PlanResult plan = optimizer.Plan(start, goal, CorridorFor(c));
  ASSERT_EQ(plan.status, PlanStatus::kSuccess) << plan.message;

  const RovModel rov_model(model_);
  const rov_control::ThrustAllocator allocator(allocation_, rov_control::AllocatorSettings());
  const rov_simulation::PlanarDynamics dynamics(
    model_.params(), allocation_, rov_simulation::LoadThrusterTimeConstant(THRUSTERS_V4_YAML));
  rov_simulation::PlanarSimulator simulator(dynamics, 1.0 / 200.0, 0.2, 0.0);
  rov_simulation::PlanarState state;
  state.pose = Eigen::Vector3d(start.position.x(), start.position.y(), 0.0);
  simulator.Reset(state);

  const double period = 0.02;  // 50 Hz commands
  const double end = plan.trajectory.TotalDuration();
  for (double t = 0.0; t < end + 5.0; t += period) {
    Eigen::Vector3d pos, vel, acc;
    plan.trajectory.Evaluate(std::min(t, end), &pos, &vel, &acc);
    TrajectoryState reference;
    if (t < end) {
      reference.velocity = vel.head<2>();
      reference.acceleration = acc.head<2>();
      reference.yaw = pos(2);
      reference.yaw_rate = vel(2);
      reference.yaw_acceleration = acc(2);
    }
    const Eigen::Vector3d planar = rov_model.Compute(reference).wrench;
    rov_control::WrenchVector wrench = rov_control::WrenchVector::Zero();
    wrench << planar(0), planar(1), 0.0, 0.0, 0.0, planar(2);
    simulator.SetCommand(allocator.Allocate(wrench).forces, t);
    simulator.AdvanceTo(t + period);
  }
  const double error = (simulator.state().pose.head<2>() - goal.head<2>()).norm();
  std::printf("[ ff check   ] open-loop feedforward ends %.3f m from the goal\n", error);
  EXPECT_LT(error, 0.5);
}


// Surface victim (design 3 item 5): within d_final = 2 m of the goal the speed
// stays at or below v_final = 0.3 m/s, and v_max(d) brakes to it at a_b.
TEST_F(SurfaceTrajOptTest, SurfaceFinalZoneCapsTheSpeed)
{
  Case c = Straight();
  c.path = {{5.0, 10.0}, {25.0, 10.0}};
  const Corridor corridor = CorridorFor(c);
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  StartState start;
  start.position = c.path.front();
  const Eigen::Vector3d goal(25.0, 10.0, 0.0);
  SpeedOptions surface;
  surface.final_zone = true;
  const PlanResult result = optimizer.Plan(start, goal, corridor, nullptr, surface);
  ASSERT_EQ(result.status, PlanStatus::kSuccess) << result.message;
  EXPECT_LE(result.report.speed_mps, 1e-9);
  EXPECT_NEAR(optimizer.SpeedLimit(1.0), 0.3, 1e-12);
  EXPECT_NEAR(optimizer.SpeedLimit(2.5), std::sqrt(0.09 + 2.0 * 1.0 * 0.5), 1e-12);
  double max_in_zone = 0.0;
  for (double t = 0.0; t <= result.trajectory.TotalDuration(); t += 0.01) {
    Eigen::Vector3d pos, vel;
    result.trajectory.Evaluate(t, &pos, &vel, nullptr);
    if ((pos.head<2>() - goal.head<2>()).norm() <= 2.0) {
      max_in_zone = std::max(max_in_zone, vel.head<2>().norm());
    }
  }
  std::printf(
    "[ surface    ] T %.2f s, max speed %.3f m/s, max within 2 m %.3f m/s\n",
    result.trajectory.TotalDuration(), result.report.max_speed_mps, max_in_zone);
  EXPECT_LE(max_in_zone, 0.3 + 1e-6);
  // Without the zone the same request is faster near the goal.
  SurfaceTrajectoryOptimizer plain(model_, allocation_, settings_);
  EXPECT_GT(optimizer.SpeedLimit(1.9), 0.0);
  const double plain_duration = plain.Plan(start, goal, corridor).trajectory.TotalDuration();
  EXPECT_LT(result.trajectory.TotalDuration() - plain_duration, 20.0);
}

// An extra speed cap (safety limit or a 0.7 v_cruise retry) is respected.
TEST_F(SurfaceTrajOptTest, ExtraSpeedCapIsRespected)
{
  Case c = Straight();
  c.path = {{5.0, 10.0}, {25.0, 10.0}};
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  StartState start;
  start.position = c.path.front();
  SpeedOptions capped;
  capped.max_speed = 0.7 * settings_.cruise_speed;
  const PlanResult result =
    optimizer.Plan(start, Eigen::Vector3d(25.0, 10.0, 0.0), CorridorFor(c), nullptr, capped);
  ASSERT_EQ(result.status, PlanStatus::kSuccess) << result.message;
  EXPECT_LE(result.report.max_speed_mps, capped.max_speed + 1e-6);
  EXPECT_DOUBLE_EQ(optimizer.CruiseSpeed(), capped.max_speed);
  SpeedOptions invalid;
  invalid.max_speed = 0.0;
  EXPECT_THROW(optimizer.SetSpeedOptions(invalid), std::invalid_argument);
}

// The gradient stays exact with the final-zone cap active.
TEST_F(SurfaceTrajOptTest, FinalZoneGradientMatchesFiniteDifferences)
{
  Case c = Straight();
  c.path = {{5.0, 10.0}, {12.0, 10.0}};
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  SpeedOptions surface;
  surface.final_zone = true;
  optimizer.SetSpeedOptions(surface);
  StartState start;
  start.position = c.path.front();
  std::string error;
  ASSERT_TRUE(optimizer.Setup(start, Eigen::Vector3d(12.0, 10.0, 0.0), CorridorFor(c), &error))
    << error;
  Eigen::VectorXd x = optimizer.InitialGuess();
  const int pieces = static_cast<int>(CorridorFor(c).polygons.size());
  x.head(pieces).array() -= 0.8;  // too fast: the zone cap is violated
  Eigen::VectorXd analytic;
  optimizer.Cost(x, &analytic);
  Eigen::VectorXd unused;
  Eigen::VectorXd numeric(x.size());
  for (int i = 0; i < x.size(); ++i) {
    const double h = 1e-6 * std::max(1.0, std::abs(x(i)));
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += h;
    minus(i) -= h;
    numeric(i) = (optimizer.Cost(plus, &unused) - optimizer.Cost(minus, &unused)) / (2.0 * h);
  }
  EXPECT_LT((analytic - numeric).norm() / numeric.norm(), 1e-4);
}

// A cap below the current speed (a safety limit arriving at 1.4 m/s) is
// phased in at a_b from the start, so the replan is feasible and slows down.
TEST_F(SurfaceTrajOptTest, LowerCapWhileMovingIsPhasedIn)
{
  Case c = Straight();
  c.path = {{5.0, 10.0}, {25.0, 10.0}};
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  StartState start;
  start.position = c.path.front();
  start.velocity = Eigen::Vector2d(1.4, 0.0);
  SpeedOptions capped;
  capped.max_speed = 0.5;
  const PlanResult result =
    optimizer.Plan(start, Eigen::Vector3d(25.0, 10.0, 0.0), CorridorFor(c), nullptr, capped);
  ASSERT_EQ(result.status, PlanStatus::kSuccess) << result.message;
  // Envelope: sqrt(1.5^2 - 2 * 1 * s) (0.1 m/s headroom) reaches 0.5 m/s after 1.0 m.
  EXPECT_NEAR(optimizer.SpeedLimit(10.0, 0.0), 1.5, 1e-9);
  EXPECT_NEAR(optimizer.SpeedLimit(10.0, 2.0), 0.5, 1e-9);
  for (double t = 0.0; t <= result.trajectory.TotalDuration(); t += 0.01) {
    Eigen::Vector3d pos, vel;
    result.trajectory.Evaluate(t, &pos, &vel, nullptr);
    if ((pos.head<2>() - start.position).norm() > 1.0 + 1e-6) {
      EXPECT_LE(vel.head<2>().norm(), 0.5 + 1e-6) << "t " << t;
    }
  }
  // The gradient is exact with the phase-in active.
  ASSERT_TRUE(optimizer.Setup(start, Eigen::Vector3d(25.0, 10.0, 0.0), CorridorFor(c), nullptr));
  Eigen::VectorXd x = optimizer.InitialGuess();
  x.head(static_cast<int>(CorridorFor(c).polygons.size())).array() -= 0.5;
  Eigen::VectorXd analytic;
  optimizer.Cost(x, &analytic);
  Eigen::VectorXd unused;
  Eigen::VectorXd numeric(x.size());
  for (int i = 0; i < x.size(); ++i) {
    const double h = 1e-6 * std::max(1.0, std::abs(x(i)));
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += h;
    minus(i) -= h;
    numeric(i) = (optimizer.Cost(plus, &unused) - optimizer.Cost(minus, &unused)) / (2.0 * h);
  }
  EXPECT_LT((analytic - numeric).norm() / numeric.norm(), 1e-4);
}

}  // namespace
}  // namespace rov_planning
