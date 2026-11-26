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

#include "rov_planning/planning_worker.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace rov_planning
{

PlanningWorker::PlanningWorker(std::unique_ptr<SurfaceTrajectoryOptimizer> optimizer)
: optimizer_(std::move(optimizer))
{
  if (!optimizer_) {
    throw std::invalid_argument("PlanningWorker needs an optimizer");
  }
  thread_ = std::thread([this]() {Run();});
}

PlanningWorker::~PlanningWorker()
{
  Stop();
}

bool PlanningWorker::Submit(PlanningJob job)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_ || job_ || running_job_) {
      return false;
    }
    job_ = std::move(job);
  }
  wake_.notify_one();
  return true;
}

std::optional<PlanningOutcome> PlanningWorker::TakeResult()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::optional<PlanningOutcome> result = std::move(result_);
  result_.reset();
  return result;
}

void PlanningWorker::SetHold(bool hold)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_ = hold;
  }
  wake_.notify_one();
}

void PlanningWorker::Stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool PlanningWorker::Busy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return job_.has_value() || running_job_;
}

void PlanningWorker::Run()
{
  while (true) {
    PlanningJob job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // Wait for a job that is not held, or for Stop().
      wake_.wait(lock, [this]() {return stop_ || (job_.has_value() && !hold_);});
      if (stop_) {
        return;  // a queued or held job is dropped
      }
      job = std::move(*job_);
      job_.reset();
      running_job_ = true;
    }
    PlanningOutcome outcome = Solve(job);  // outside the lock
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result_ = std::move(outcome);
      running_job_ = false;
    }
  }
}

PlanningOutcome PlanningWorker::Solve(const PlanningJob & job)
{
  PlanningOutcome outcome;
  outcome.request = job.request;
  outcome.grid = job.grid;
  try {
    const auto corridor_start = std::chrono::steady_clock::now();
    // The inflation is bound to this snapshot; the corridor uses its map.
    const std::shared_ptr<const CorridorObstacles> obstacles =
      cache_.Get(job.grid, job.corridor_settings);
    outcome.cache_hit = cache_.last_was_hit();
    const Corridor corridor = BuildCorridor(job.path, job.corridor_settings, *obstacles);
    outcome.corridor_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - corridor_start).count();
    outcome.polygons = corridor.polygons.size();
    outcome.corridor = corridor.polygons;
    if (job.force_failure) {
      outcome.ok = false;
      outcome.plan.status = PlanStatus::kOptimizerFailed;
      outcome.error = "optimizer failure forced by the test hook (force_optimizer_failures)";
      return outcome;
    }
    outcome.plan = optimizer_->Plan(
      job.start, job.goal, corridor, job.warm ? &*job.warm : nullptr, job.speed);
    outcome.ok = outcome.plan.status == PlanStatus::kSuccess;
    if (!outcome.ok) {
      outcome.error = std::string("optimizer ") + PlanStatusToString(outcome.plan.status) +
        ": " + outcome.plan.message;
    }
  } catch (const std::exception & error) {
    outcome.ok = false;
    outcome.error = std::string("corridor: ") + error.what();
  }
  return outcome;
}

}  // namespace rov_planning
