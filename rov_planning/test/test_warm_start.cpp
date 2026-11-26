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

// MINCO warm starts and the solve deadline (offline).
//
// Replanning scenario: plan cold from the start, fly t_advance seconds along
// the result, then plan again from that state (position, velocity,
// acceleration, yaw) to the same goal, cold and warm, on the same corridor.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rov_control/planar_model.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_planning/corridor_2d.hpp"
#include "rov_planning/surface_traj_opt.hpp"

namespace rov_planning
{
namespace
{

constexpr double kAdvance = 3.0;  // s flown before replanning

OccupancyGrid2D EmptyGrid(double width_m, double height_m)
{
  OccupancyGrid2D grid;
  grid.width = static_cast<int>(std::lround(width_m / grid.resolution));
  grid.height = static_cast<int>(std::lround(height_m / grid.resolution));
  grid.data.assign(grid.width * grid.height, 0);
  return grid;
}

struct Route
{
  std::string name;
  OccupancyGrid2D grid;
  std::vector<Eigen::Vector2d> path;
  double goal_yaw = 0.0;
};

Route Straight()
{
  return {"straight", EmptyGrid(60.0, 20.0), {{5.0, 10.0}, {55.0, 10.0}}, 0.0};
}

Route LTurn()
{
  Route r{"l_turn", EmptyGrid(20.0, 14.0), {{2.0, 3.0}, {3.5, 3.0}, {3.5, 9.0}, {17.0, 9.0}},
    M_PI / 2.0};
  for (int y = 0; y < r.grid.height; ++y) {
    for (int x = 0; x < r.grid.width; ++x) {
      const double cx = (x + 0.5) * r.grid.resolution;
      const double cy = (y + 0.5) * r.grid.resolution;
      if (cx > 6.0 && cx < 14.0 && cy < 6.0) {
        r.grid.data[y * r.grid.width + x] = 100;
      }
    }
  }
  return r;
}

// The rest of `path` from point p: p, then the vertices after the segment
// closest to p.
std::vector<Eigen::Vector2d> RemainingPath(
  const std::vector<Eigen::Vector2d> & path, const Eigen::Vector2d & p)
{
  std::size_t best = 0;
  double best_distance = 1e9;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector2d ab = path[i + 1] - path[i];
    const double t = std::clamp((p - path[i]).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
    const double d = (path[i] + t * ab - p).norm();
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  std::vector<Eigen::Vector2d> rest = {p};
  for (std::size_t i = best + 1; i < path.size(); ++i) {
    rest.push_back(path[i]);
  }
  return rest;
}

class WarmStartTest : public ::testing::Test
{
protected:
  WarmStartTest()
  : model_(rov_control::LoadHydrodynamicParams(HYDRODYNAMICS_V4_YAML)),
    allocation_(rov_control::BuildAllocationMatrix(
        rov_control::LoadThrusterLayout(THRUSTERS_V4_YAML))),
    settings_(LoadOptimizerSettings(OPTIMIZER_YAML))
  {
    corridor_settings_.clearance = 0.3;
  }

  // Cold plan from the start, then replan from t = kAdvance cold and warm.
  void CompareColdAndWarm(const Route & route)
  {
    const Eigen::Vector3d goal(route.path.back().x(), route.path.back().y(), route.goal_yaw);
    StartState start;
    start.position = route.path.front();
    SurfaceTrajectoryOptimizer first(model_, allocation_, settings_);
    const PlanResult original =
      first.Plan(start, goal, BuildCorridor(route.path, route.grid, corridor_settings_));
    ASSERT_EQ(original.status, PlanStatus::kSuccess) << original.message;

    // The state reached after kAdvance seconds.
    Eigen::Vector3d pos, vel, acc;
    original.trajectory.Evaluate(kAdvance, &pos, &vel, &acc);
    StartState advanced;
    advanced.position = pos.head<2>();
    advanced.velocity = vel.head<2>();
    advanced.acceleration = acc.head<2>();
    advanced.yaw = pos(2);
    advanced.yaw_rate = vel(2);
    advanced.yaw_acceleration = acc(2);

    // One corridor for both, so only the initial guess differs.
    const auto corridor_start = std::chrono::steady_clock::now();
    const Corridor corridor =
      BuildCorridor(RemainingPath(route.path, advanced.position), route.grid, corridor_settings_);
    const double corridor_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - corridor_start).count();

    SurfaceTrajectoryOptimizer cold_optimizer(model_, allocation_, settings_);
    const PlanResult cold = cold_optimizer.Plan(advanced, goal, corridor);
    SurfaceTrajectoryOptimizer warm_optimizer(model_, allocation_, settings_);
    const WarmStart warm{original.trajectory, kAdvance};
    const PlanResult warmed = warm_optimizer.Plan(advanced, goal, corridor, &warm);

    for (const PlanResult * r : {&cold, &warmed}) {
      EXPECT_EQ(r->status, PlanStatus::kSuccess) << r->message;  // all checks passed
      EXPECT_LE(r->report.corridor_m, 1e-9);
      EXPECT_LE(r->report.thrust_n, 0.02 * settings_.planning_forward_force);
      EXPECT_LE(r->report.speed_mps, 1e-9);
    }
    EXPECT_FALSE(cold.warm_started);
    EXPECT_TRUE(warmed.warm_started) << warmed.warm_start_note;
    std::printf(
      "[ %-8s ] replan at t = %.1f s, %zu polygons, corridor %.1f ms\n"
      "[ %-8s ]   cold: optimizer %6.1f ms (%4d evals), incl. checks %6.1f ms, "
      "total with corridor %6.1f ms, T %.2f s\n"
      "[ %-8s ]   warm: optimizer %6.1f ms (%4d evals), incl. checks %6.1f ms, "
      "total with corridor %6.1f ms, T %.2f s (remaining old %.2f s)\n",
      route.name.c_str(), kAdvance, corridor.polygons.size(), corridor_ms,
      route.name.c_str(), cold.solve_time_ms, cold.evaluations, cold.total_time_ms,
      corridor_ms + cold.total_time_ms, cold.trajectory.TotalDuration(),
      route.name.c_str(), warmed.solve_time_ms, warmed.evaluations, warmed.total_time_ms,
      corridor_ms + warmed.total_time_ms, warmed.trajectory.TotalDuration(),
      original.trajectory.TotalDuration() - kAdvance);
  }

  rov_control::PlanarModel model_;
  rov_control::AllocationMatrix allocation_;
  OptimizerSettings settings_;
  CorridorSettings corridor_settings_;
};

TEST_F(WarmStartTest, StraightColdVersusWarm)
{
  CompareColdAndWarm(Straight());
}

TEST_F(WarmStartTest, LTurnColdVersusWarm)
{
  CompareColdAndWarm(LTurn());
}

// A finished previous trajectory cannot be mapped: cold start, same budget.
TEST_F(WarmStartTest, UnmappableWarmStartFallsBackToCold)
{
  const Route route = Straight();
  const Eigen::Vector3d goal(55.0, 10.0, 0.0);
  StartState start;
  start.position = route.path.front();
  const Corridor corridor = BuildCorridor(route.path, route.grid, corridor_settings_);
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, settings_);
  const PlanResult first = optimizer.Plan(start, goal, corridor);
  ASSERT_EQ(first.status, PlanStatus::kSuccess);
  const WarmStart finished{first.trajectory, first.trajectory.TotalDuration() + 1.0};
  const PlanResult result = optimizer.Plan(start, goal, corridor, &finished);
  EXPECT_FALSE(result.warm_started);
  EXPECT_NE(result.warm_start_note.find("remains"), std::string::npos) << result.warm_start_note;
  EXPECT_EQ(result.status, PlanStatus::kSuccess) << result.message;
}

// The deadline stops the solve and gives an explicit timeout.
TEST_F(WarmStartTest, DeadlineGivesTimeout)
{
  OptimizerSettings tight = settings_;
  tight.solve_deadline_ms = 1.0;
  const Route route = LTurn();
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, tight);
  StartState start;
  start.position = route.path.front();
  const PlanResult result = optimizer.Plan(
    start, Eigen::Vector3d(17.0, 9.0, M_PI / 2.0),
    BuildCorridor(route.path, route.grid, corridor_settings_));
  EXPECT_EQ(result.status, PlanStatus::kTimeout);
  EXPECT_NE(result.message.find("deadline"), std::string::npos) << result.message;
  EXPECT_LT(result.solve_time_ms, 50.0);  // stopped promptly, not after ~850 ms
  std::printf("[ deadline ] 1 ms deadline: stopped after %.1f ms (%d evals): %s\n",
    result.solve_time_ms, result.evaluations, result.message.c_str());
}

// Regression: optimization finishes before the deadline but validation
// crosses it. The result must be kTimeout (never success, so never sent).
TEST_F(WarmStartTest, ValidationCrossingTheDeadlineIsATimeout)
{
  const Route route = Straight();
  const Eigen::Vector3d goal(55.0, 10.0, 0.0);
  StartState start;
  start.position = route.path.front();
  const Corridor corridor = BuildCorridor(route.path, route.grid, corridor_settings_);

  // How long the optimization alone takes here.
  SurfaceTrajectoryOptimizer probe(model_, allocation_, settings_);
  const PlanResult normal = probe.Plan(start, goal, corridor);
  ASSERT_EQ(normal.status, PlanStatus::kSuccess) << normal.message;

  // Deadline well after the optimization, but a validation made ~1000x
  // denser than usual cannot finish before it.
  OptimizerSettings slow_check = settings_;
  slow_check.solve_deadline_ms = 2.0 * normal.solve_time_ms + 100.0;
  slow_check.check_samples_per_piece = settings_.check_samples_per_piece * 1000;
  SurfaceTrajectoryOptimizer optimizer(model_, allocation_, slow_check);
  const PlanResult result = optimizer.Plan(start, goal, corridor);
  EXPECT_LT(result.solve_time_ms, slow_check.solve_deadline_ms);   // optimized on time
  EXPECT_GT(result.total_time_ms, slow_check.solve_deadline_ms);   // validation was late
  EXPECT_EQ(result.status, PlanStatus::kTimeout);
  EXPECT_NE(result.message.find("during validation"), std::string::npos) << result.message;
  std::printf(
    "[ deadline ] validation late: deadline %.0f ms, optimization %.1f ms, total %.1f ms -> %s\n",
    slow_check.solve_deadline_ms, result.solve_time_ms, result.total_time_ms,
    PlanStatusToString(result.status));
}

}  // namespace
}  // namespace rov_planning
