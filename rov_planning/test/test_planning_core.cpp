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

// Unit tests for the planning core: required wrench and gradients, wrench
// polytope, and 2D corridor. Real V4 parameter files (paths set by CMake).

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "rov_control/planar_model.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_planning/corridor_2d.hpp"
#include "rov_planning/rov_model.hpp"
#include "rov_planning/wrench_polytope.hpp"

namespace rov_planning
{
namespace
{

constexpr double kForward = 110.0;  // f_plan, N (thrusters_v4.yaml)
constexpr double kReverse = 70.0;   // test value; f_rev is TBD in the design

RovModel V4Model()
{
  return RovModel(rov_control::PlanarModel(
           rov_control::LoadHydrodynamicParams(HYDRODYNAMICS_V4_YAML)));
}

rov_control::AllocationMatrix V4Allocation()
{
  return rov_control::BuildAllocationMatrix(rov_control::LoadThrusterLayout(THRUSTERS_V4_YAML));
}

TEST(RovModelTest, ConstantSurgeNeedsOnlyTheDampingForce)
{
  TrajectoryState state;
  state.velocity = Eigen::Vector2d(1.5, 0.0);  // facing +x, so pure surge
  const RequiredWrench result = V4Model().Compute(state);
  // -X_u u - X_u|u| u|u| = 10 * 1.5 + 100 * 1.5^2
  EXPECT_NEAR(result.wrench(0), 240.0, 1e-9);
  EXPECT_NEAR(result.wrench(1), 0.0, 1e-12);
  EXPECT_NEAR(result.wrench(2), 0.0, 1e-12);
}

// Every gradient against central finite differences, at a few general states.
// The gradient-free wrench equals the full computation.
TEST(RovModelTest, WrenchOnlyMatchesCompute)
{
  const RovModel model(rov_control::PlanarModel(
      rov_control::LoadHydrodynamicParams(HYDRODYNAMICS_V4_YAML)));
  TrajectoryState state;
  state.velocity = Eigen::Vector2d(1.2, -0.4);
  state.acceleration = Eigen::Vector2d(-0.3, 0.7);
  state.yaw = 0.8;
  state.yaw_rate = -0.5;
  state.yaw_acceleration = 0.2;
  EXPECT_TRUE(model.Wrench(state).isApprox(model.Compute(state).wrench, 1e-12));
}

TEST(RovModelTest, GradientsMatchFiniteDifferences)
{
  const RovModel model = V4Model();
  const std::vector<TrajectoryState> states = {
    {{1.2, -0.7}, {0.4, 0.9}, 0.6, 0.35, -0.8},
    {{-0.5, 1.4}, {-1.1, 0.2}, -2.3, -0.9, 1.5},
    {{0.3, 0.2}, {0.0, -1.3}, 3.0, 1.2, 0.1},
  };
  const double h = 1e-6;
  for (const TrajectoryState & state : states) {
    const RequiredWrench analytic = model.Compute(state);
    // Derivative of the wrench along one input, by central differences.
    auto numeric = [&](const std::function<void(TrajectoryState &, double)> & nudge) {
        TrajectoryState plus = state;
        TrajectoryState minus = state;
        nudge(plus, h);
        nudge(minus, -h);
        return Eigen::Vector3d(
          (model.Compute(plus).wrench - model.Compute(minus).wrench) / (2.0 * h));
      };
    auto expect_close = [](const Eigen::Vector3d & a, const Eigen::Vector3d & n) {
        EXPECT_LE((a - n).norm(), 1e-4 * std::max(n.norm(), 1e-3))
          << "analytic " << a.transpose() << "\nnumeric  " << n.transpose();
      };
    for (int i = 0; i < 2; ++i) {
      expect_close(
        analytic.d_velocity.col(i),
        numeric([i](TrajectoryState & s, double d) {s.velocity(i) += d;}));
      expect_close(
        analytic.d_acceleration.col(i),
        numeric([i](TrajectoryState & s, double d) {s.acceleration(i) += d;}));
    }
    expect_close(analytic.d_yaw, numeric([](TrajectoryState & s, double d) {s.yaw += d;}));
    expect_close(
      analytic.d_yaw_rate, numeric([](TrajectoryState & s, double d) {s.yaw_rate += d;}));
    expect_close(
      analytic.d_yaw_acceleration,
      numeric([](TrajectoryState & s, double d) {s.yaw_acceleration += d;}));
  }
}

TEST(WrenchPolytopeTest, FeasibleWrenchesAreInsideAndFacesAreTight)
{
  const rov_control::AllocationMatrix b = V4Allocation();
  const WrenchPolytope polytope = ComputeWrenchPolytope(b, kForward, kReverse);
  EXPECT_EQ(polytope.normals.rows(), 12);  // 4 horizontal thrusters: 6 pairs x 2

  // Every corner of the thrust box (16 for thrusters 01-04) maps inside, and
  // each face is touched by some corner (so the faces are not too loose).
  Eigen::VectorXd tightest = Eigen::VectorXd::Constant(polytope.offsets.size(), -1e9);
  for (int mask = 0; mask < 16; ++mask) {
    rov_control::ThrustVector f = rov_control::ThrustVector::Zero();
    for (int k = 0; k < 4; ++k) {
      f(k) = (mask >> k & 1) ? kForward : -kReverse;
    }
    const Eigen::Vector3d tau((b * f)(0), (b * f)(1), (b * f)(5));
    EXPECT_TRUE(polytope.Contains(tau, 1e-9));
    tightest = tightest.cwiseMax(polytope.Margins(tau));
  }
  EXPECT_LT(tightest.cwiseAbs().maxCoeff(), 1e-9);

  // One feasible wrench: 200 N surge (needs about 71 N per thruster).
  EXPECT_TRUE(polytope.Contains(Eigen::Vector3d(200.0, 0.0, 0.0)));
}

TEST(WrenchPolytopeTest, WrenchNeedingMoreThanTheLimitIsOutside)
{
  const WrenchPolytope polytope = ComputeWrenchPolytope(V4Allocation(), kForward, kReverse);
  // Pure surge from 4 thrusters at 45 degrees: Fx = 4 * cos(45) * f.
  const double per_newton = 4.0 * std::sqrt(0.5);
  // Forward: just inside at f_plan, outside 5% above it.
  EXPECT_TRUE(polytope.Contains(Eigen::Vector3d(0.99 * per_newton * kForward, 0.0, 0.0)));
  EXPECT_FALSE(polytope.Contains(Eigen::Vector3d(1.05 * per_newton * kForward, 0.0, 0.0)));
  // Reverse uses its own, smaller limit.
  EXPECT_TRUE(polytope.Contains(Eigen::Vector3d(-0.99 * per_newton * kReverse, 0.0, 0.0)));
  EXPECT_FALSE(polytope.Contains(Eigen::Vector3d(-1.05 * per_newton * kReverse, 0.0, 0.0)));
}

// 8 m x 6 m map, 0.05 m cells, one wall rising from the bottom edge in the
// middle (x 3..5 m, y 0..3.5 m). The path goes up, over the wall, and down.
TEST(CorridorTest, PolygonsOverlapCoverThePathAndKeepClearance)
{
  OccupancyGrid2D grid;
  grid.resolution = 0.05;
  grid.width = 160;
  grid.height = 120;
  grid.data.assign(grid.width * grid.height, 0);
  for (int y = 0; y < 70; ++y) {
    for (int x = 60; x < 100; ++x) {
      grid.data[y * grid.width + x] = 100;
    }
  }
  const std::vector<Eigen::Vector2d> path = {
    {1.0, 1.0}, {2.2, 1.0}, {2.2, 4.6}, {5.8, 4.6}, {5.8, 1.0}, {7.0, 1.0}};
  CorridorSettings settings;
  settings.clearance = 0.3;
  const auto start = std::chrono::steady_clock::now();
  const Corridor corridor = BuildCorridor(path, grid, settings);
  const double build_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();

  ASSERT_FALSE(corridor.polygons.empty());
  EXPECT_LE(static_cast<int>(corridor.polygons.size()), settings.max_polygons);
  ASSERT_EQ(corridor.polygons.size(), corridor.seeds.size());
  auto margin = [](const Eigen::MatrixX4d & polygon, const Eigen::Vector2d & p) {
      // Largest a_x x + a_y y + d: <= 0 inside, < 0 strictly inside.
      return (polygon.leftCols<2>() * p + polygon.col(3)).maxCoeff();
    };

  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const Eigen::MatrixX4d & polygon = corridor.polygons[i];
    // Half-space format: zero yaw coefficient, unit normals.
    EXPECT_TRUE(polygon.col(2).isZero(0.0));
    EXPECT_LT((polygon.leftCols<2>().rowwise().norm().array() - 1.0).abs().maxCoeff(), 1e-12);
    // The seed segment is inside its polygon.
    EXPECT_LE(margin(polygon, corridor.seeds[i][0]), 1e-9);
    EXPECT_LE(margin(polygon, corridor.seeds[i][1]), 1e-9);
    // Overlap: the shared seed end is strictly inside both polygons.
    if (i + 1 < corridor.polygons.size()) {
      EXPECT_LT(margin(polygon, corridor.seeds[i][1]), -1e-6) << "polygon " << i;
      EXPECT_LT(margin(corridor.polygons[i + 1], corridor.seeds[i][1]), -1e-6);
    }
    // Clearance from every occupied cell.
    const std::vector<Eigen::Vector2d> vertices = PolygonVertices(polygon);
    ASSERT_GE(vertices.size(), 3u);
    double closest = 1e9;
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        if (grid.data[y * grid.width + x] == 100) {
          const Eigen::Vector2d center =
            grid.origin + grid.resolution * Eigen::Vector2d(x + 0.5, y + 0.5);
          closest = std::min(
            closest, PolygonSquareDistance(vertices, center, 0.5 * grid.resolution));
        }
      }
    }
    EXPECT_GE(closest, settings.clearance - 1e-9) << "polygon " << i;
  }

  // The whole path (every 2 cm) lies in at least one polygon.
  for (std::size_t k = 1; k < path.size(); ++k) {
    const double length = (path[k] - path[k - 1]).norm();
    for (double s = 0.0; s <= length; s += 0.02) {
      const Eigen::Vector2d p = path[k - 1] + (path[k] - path[k - 1]) * (s / length);
      bool covered = false;
      for (const Eigen::MatrixX4d & polygon : corridor.polygons) {
        covered = covered || margin(polygon, p) <= 1e-9;
      }
      EXPECT_TRUE(covered) << "path point " << p.transpose() << " is in no polygon";
    }
  }
  std::cout << "[ corridor ] " << corridor.polygons.size() << " polygons for a 13.2 m path, "
            << "built in " << build_ms << " ms" << std::endl;
}

TEST(CorridorTest, PathTooCloseToAWallIsRejected)
{
  OccupancyGrid2D grid;
  grid.width = 40;
  grid.height = 40;
  grid.data.assign(grid.width * grid.height, 0);
  for (int y = 0; y < 40; ++y) {
    grid.data[y * grid.width + 20] = 100;  // wall at x = 1.0 .. 1.05 m
  }
  CorridorSettings settings;
  settings.clearance = 0.3;
  // Runs 0.15 m from the wall.
  EXPECT_THROW(
    BuildCorridor({{0.85, 0.2}, {0.85, 1.8}}, grid, settings), std::runtime_error);
}

// Regression: the waypoint (2.0, 2.0) sits on an occupied block. The straight
// shortcut between the first and the last point passes 0.5 m below it, so
// checking only shortcuts would accept a path that runs through the block.
TEST(CorridorTest, ShortcutDoesNotHideAnOccupiedWaypoint)
{
  OccupancyGrid2D grid;
  grid.width = 80;
  grid.height = 80;
  grid.data.assign(grid.width * grid.height, 0);
  for (int y = 38; y < 42; ++y) {
    for (int x = 38; x < 42; ++x) {
      grid.data[y * grid.width + x] = 100;  // 0.2 m block around (2.0, 2.0)
    }
  }
  CorridorSettings settings;
  settings.clearance = 0.1;
  settings.seed_spacing = 5.0;  // one shortcut over the whole path
  const std::vector<Eigen::Vector2d> through_block = {{1.0, 1.5}, {2.0, 2.0}, {3.0, 1.5}};
  EXPECT_THROW(BuildCorridor(through_block, grid, settings), std::runtime_error);
  // The same shortcut alone is fine, so only the path check can catch it.
  EXPECT_NO_THROW(BuildCorridor({{1.0, 1.5}, {3.0, 1.5}}, grid, settings));
}

// The inflation cache: reused only for the very same map snapshot and the
// same inflation settings; the cached inflation is exactly a fresh one.
TEST(CorridorTest, ObstacleCacheFollowsSnapshotAndSettings)
{
  auto grid = std::make_shared<OccupancyGrid2D>();
  grid->width = 160;
  grid->height = 120;
  grid->data.assign(grid->width * grid->height, 0);
  for (int y = 0; y < 70; ++y) {
    for (int x = 60; x < 100; ++x) {
      grid->data[y * grid->width + x] = 100;
    }
  }
  const std::shared_ptr<const OccupancyGrid2D> snapshot = grid;
  const std::vector<Eigen::Vector2d> path = {
    {1.0, 1.0}, {2.2, 1.0}, {2.2, 4.6}, {5.8, 4.6}, {5.8, 1.0}, {7.0, 1.0}};
  CorridorSettings settings;
  settings.clearance = 0.3;

  CorridorObstacleCache cache;
  cache.Get(snapshot, settings);
  EXPECT_FALSE(cache.last_was_hit());
  const auto cached = cache.Get(snapshot, settings);
  EXPECT_TRUE(cache.last_was_hit());
  EXPECT_EQ(cached->grid, snapshot);  // bound to its own snapshot
  // (Polygons are not compared: the vendored sdlp.hpp shuffles constraints
  // with a static random generator, so FIRI varies slightly per call.)
  const auto fresh = PrepareCorridorObstacles(snapshot, settings);
  EXPECT_EQ(cached->blocked, fresh->blocked);
  EXPECT_EQ(cached->grown, fresh->grown);
  EXPECT_FALSE(BuildCorridor(path, settings, *cached).polygons.empty());

  CorridorSettings wider = settings;
  wider.clearance = 0.4;  // other inflation settings: rebuilt
  cache.Get(snapshot, wider);
  EXPECT_FALSE(cache.last_was_hit());
  // Inflation made with other settings is refused, not silently used.
  EXPECT_THROW(
    BuildCorridor(path, settings, *PrepareCorridorObstacles(snapshot, wider)),
    std::invalid_argument);
}

// Regression: a map with the same size and origin but one newly occupied
// cell must not reuse the old inflation.
TEST(CorridorTest, SameSizeMapWithNewObstacleDoesNotReuseInflation)
{
  auto first = std::make_shared<OccupancyGrid2D>();
  first->width = 160;
  first->height = 120;
  first->data.assign(first->width * first->height, 0);
  auto second = std::make_shared<OccupancyGrid2D>(*first);  // same size, origin, resolution
  const int new_cell = 60 * second->width + 80;  // (4.0, 3.0) m
  second->data[new_cell] = 100;

  CorridorSettings settings;
  settings.clearance = 0.3;
  CorridorObstacleCache cache;
  const auto old_inflation = cache.Get(first, settings);
  EXPECT_FALSE(old_inflation->blocked[new_cell]);
  const auto new_inflation = cache.Get(second, settings);
  EXPECT_FALSE(cache.last_was_hit());
  EXPECT_EQ(new_inflation->grid, second);
  EXPECT_TRUE(new_inflation->blocked[new_cell]);

  // A path through the new obstacle: rejected with the new inflation; the
  // old one carries its own (empty) map, so it cannot be applied to `second`.
  const std::vector<Eigen::Vector2d> through = {{1.0, 3.0}, {7.0, 3.0}};
  EXPECT_THROW(BuildCorridor(through, settings, *new_inflation), std::runtime_error);
  EXPECT_EQ(old_inflation->grid, first);
}

}  // namespace
}  // namespace rov_planning
