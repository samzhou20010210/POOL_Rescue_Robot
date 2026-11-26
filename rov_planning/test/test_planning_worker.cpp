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

// PlanningWorker: hold / release, one job at a time, clean shutdown.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

#include "rov_control/planar_model.hpp"
#include "rov_control/thrust_allocator.hpp"
#include "rov_planning/planning_worker.hpp"

namespace rov_planning
{
namespace
{

std::unique_ptr<SurfaceTrajectoryOptimizer> MakeOptimizer()
{
  return std::make_unique<SurfaceTrajectoryOptimizer>(
    rov_control::PlanarModel(rov_control::LoadHydrodynamicParams(HYDRODYNAMICS_V4_YAML)),
    rov_control::BuildAllocationMatrix(rov_control::LoadThrusterLayout(THRUSTERS_V4_YAML)),
    LoadOptimizerSettings(OPTIMIZER_YAML));
}

PlanningJob StraightJob(uint64_t request)
{
  auto grid = std::make_shared<OccupancyGrid2D>();
  grid->width = 200;
  grid->height = 100;
  grid->data.assign(grid->width * grid->height, 0);
  PlanningJob job;
  job.request = request;
  job.grid = grid;
  job.corridor_settings.clearance = 0.3;
  job.path = {{1.0, 2.5}, {7.0, 2.5}};
  job.start.position = job.path.front();
  job.goal = Eigen::Vector3d(7.0, 2.5, 0.0);
  return job;
}

std::optional<PlanningOutcome> WaitForResult(PlanningWorker & worker, double seconds)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end) {
    if (auto result = worker.TakeResult()) {
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return std::nullopt;
}

TEST(PlanningWorkerTest, HeldJobWaitsAndReleasedJobFinishes)
{
  PlanningWorker worker(MakeOptimizer());
  worker.SetHold(true);
  PlanningJob job = StraightJob(7);
  const auto snapshot = job.grid;
  ASSERT_TRUE(worker.Submit(std::move(job)));
  EXPECT_FALSE(worker.Submit(StraightJob(8)));  // one job at a time
  EXPECT_FALSE(WaitForResult(worker, 0.3).has_value());  // held: nothing solved
  EXPECT_TRUE(worker.Busy());

  worker.SetHold(false);
  const std::optional<PlanningOutcome> outcome = WaitForResult(worker, 5.0);
  ASSERT_TRUE(outcome.has_value());
  EXPECT_EQ(outcome->request, 7u);
  EXPECT_EQ(outcome->grid, snapshot);  // reports the snapshot it planned on
  EXPECT_TRUE(outcome->ok) << outcome->error;
  EXPECT_FALSE(worker.Busy());
}

TEST(PlanningWorkerTest, StopWithAHeldJobJoinsWithoutSolving)
{
  PlanningWorker worker(MakeOptimizer());
  worker.SetHold(true);
  ASSERT_TRUE(worker.Submit(StraightJob(1)));
  const auto start = std::chrono::steady_clock::now();
  worker.Stop();  // wakes the held thread; the job is dropped
  const double stop_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  EXPECT_LT(stop_ms, 100.0);
  EXPECT_FALSE(worker.TakeResult().has_value());
  EXPECT_FALSE(worker.Submit(StraightJob(2)));  // stopped
}

}  // namespace
}  // namespace rov_planning
