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

// One background thread that runs the slow planning steps (FIRI corridor
// and MINCO optimization) so the ROS executor never blocks on them. No ROS.
//
// Usage (from one thread, e.g. the ROS executor):
//   Submit(job)    hand over one job (false if a job is already queued or
//                  running, or the worker has stopped);
//   TakeResult()   poll for the finished outcome (at most one);
//   SetHold(true)  test hook: a queued job waits before it starts until
//                  SetHold(false) (or Stop());
//   Stop()         ask the thread to end and join it. A job that is running
//                  is finished first; a queued or held job is dropped. The
//                  destructor calls Stop().
// The worker owns the optimizer and the corridor inflation cache, and only
// its thread uses them. Jobs carry their inputs by value or as immutable
// shared snapshots, so nothing is shared with the caller while it runs.

#ifndef ROV_PLANNING__PLANNING_WORKER_HPP_
#define ROV_PLANNING__PLANNING_WORKER_HPP_

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "Eigen/Dense"
#include "rov_planning/corridor_2d.hpp"
#include "rov_planning/surface_traj_opt.hpp"

namespace rov_planning
{

struct PlanningJob
{
  uint64_t request = 0;                          // caller's request ID
  std::shared_ptr<const OccupancyGrid2D> grid;   // immutable map snapshot
  CorridorSettings corridor_settings;
  std::vector<Eigen::Vector2d> path;             // A* path, map frame
  StartState start;
  Eigen::Vector3d goal = Eigen::Vector3d::Zero();  // terminal pose (x, y, yaw)
  std::optional<WarmStart> warm;                 // previous feasible trajectory, if any
  SpeedOptions speed;                            // speed cap, surface final zone
  bool force_failure = false;  // test hook: report an optimizer failure without solving
};

struct PlanningOutcome
{
  uint64_t request = 0;
  std::shared_ptr<const OccupancyGrid2D> grid;  // the snapshot it was planned on
  bool ok = false;               // corridor built and optimizer succeeded
  std::string error;             // why not, if !ok
  bool cache_hit = false;        // corridor inflation reused
  std::size_t polygons = 0;
  std::vector<Eigen::MatrixX4d> corridor;  // the polygons (for display)
  double corridor_ms = 0.0;
  PlanResult plan;               // valid if the corridor was built
};

class PlanningWorker
{
public:
  explicit PlanningWorker(std::unique_ptr<SurfaceTrajectoryOptimizer> optimizer);
  ~PlanningWorker();

  PlanningWorker(const PlanningWorker &) = delete;
  PlanningWorker & operator=(const PlanningWorker &) = delete;

  bool Submit(PlanningJob job);
  std::optional<PlanningOutcome> TakeResult();
  void SetHold(bool hold);
  void Stop();

  // True while a job is queued, held or running.
  bool Busy() const;

private:
  void Run();
  PlanningOutcome Solve(const PlanningJob & job);

  // Used only by the worker thread.
  std::unique_ptr<SurfaceTrajectoryOptimizer> optimizer_;
  CorridorObstacleCache cache_;

  // Shared with the caller, guarded by mutex_.
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<PlanningJob> job_;
  std::optional<PlanningOutcome> result_;
  bool running_job_ = false;
  bool hold_ = false;
  bool stop_ = false;

  std::thread thread_;  // last: starts after everything above exists
};

}  // namespace rov_planning

#endif  // ROV_PLANNING__PLANNING_WORKER_HPP_
