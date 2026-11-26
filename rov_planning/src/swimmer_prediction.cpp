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

#include "rov_planning/swimmer_prediction.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace rov_planning
{

namespace
{

double ReadNumber(const YAML::Node & root, const std::string & key, const std::string & path)
{
  const YAML::Node node = root[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(path + ": missing key '" + key + "'");
  }
  return node.as<double>();
}

}  // namespace

SwimmerPredictionSettings LoadSwimmerPredictionSettings(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(yaml_path + ": " + error.what());
  }
  SwimmerPredictionSettings settings;
  settings.horizon = ReadNumber(root, "prediction_horizon_s", yaml_path);
  settings.max_speed = ReadNumber(root, "max_swimmer_speed_mps", yaml_path);
  settings.clearance = ReadNumber(root, "clearance_m", yaml_path);
  settings.time_step = ReadNumber(root, "prediction_step_s", yaml_path);
  settings.max_track_age = ReadNumber(root, "max_track_age_s", yaml_path);
  settings.max_future_stamp = ReadNumber(root, "max_future_stamp_s", yaml_path);
  const std::string problem = ValidateSwimmerPredictionSettings(settings);
  if (!problem.empty()) {
    throw std::runtime_error(yaml_path + ": " + problem);
  }
  return settings;
}

std::string ValidateSwimmerPredictionSettings(const SwimmerPredictionSettings & s)
{
  if (!(s.horizon >= 0.0) || !(s.max_speed >= 0.0) || !(s.clearance >= 0.0)) {
    return "prediction_horizon_s, max_swimmer_speed_mps and clearance_m must be >= 0";
  }
  if (!(s.time_step > 0.0) || !(s.max_track_age > 0.0) || !(s.max_future_stamp >= 0.0)) {
    return "prediction_step_s and max_track_age_s must be > 0, max_future_stamp_s >= 0";
  }
  if (!std::isfinite(s.horizon) || !std::isfinite(s.max_speed) || !std::isfinite(s.clearance) ||
    !std::isfinite(s.time_step) || !std::isfinite(s.max_track_age) ||
    !std::isfinite(s.max_future_stamp))
  {
    return "swimmer prediction settings must be finite";
  }
  return "";
}

Disc PredictSwimmerDisc(
  const SwimmerTrack & track, double t, const SwimmerPredictionSettings & settings)
{
  const double age = std::max(0.0, t - track.stamp);
  return {track.position + track.velocity * age, settings.clearance + settings.max_speed * age};
}

std::vector<Disc> PredictSwimmerDiscs(
  const SwimmerTrack & track, double now, const SwimmerPredictionSettings & settings)
{
  std::vector<Disc> discs;
  const int steps = static_cast<int>(std::ceil(settings.horizon / settings.time_step - 1e-9));
  for (int k = 0; k <= steps; ++k) {
    const double t = now + std::min(k * settings.time_step, settings.horizon);
    discs.push_back(PredictSwimmerDisc(track, t, settings));
  }
  return discs;
}

const char * TrackUpdateToString(TrackUpdate update)
{
  switch (update) {
    case TrackUpdate::kAccepted: return "accepted";
    case TrackUpdate::kInvalid: return "invalid (non-finite)";
    case TrackUpdate::kOutOfOrder: return "out of order (not newer than the last update)";
    case TrackUpdate::kStale: return "stale (older than max_track_age_s)";
    case TrackUpdate::kFuture: return "stamped in the future";
  }
  return "unknown";
}

SwimmerTrackStore::SwimmerTrackStore(const SwimmerPredictionSettings & settings)
: settings_(settings)
{
  const std::string problem = ValidateSwimmerPredictionSettings(settings_);
  if (!problem.empty()) {
    throw std::invalid_argument(problem);
  }
}

TrackUpdate SwimmerTrackStore::Update(const SwimmerTrack & track, double now)
{
  if (!track.position.allFinite() || !track.velocity.allFinite() ||
    !std::isfinite(track.stamp))
  {
    return TrackUpdate::kInvalid;
  }
  const auto last = last_stamp_.find(track.id);
  if (last != last_stamp_.end() && track.stamp <= last->second) {
    return TrackUpdate::kOutOfOrder;
  }
  if (now - track.stamp > settings_.max_track_age) {
    return TrackUpdate::kStale;
  }
  if (track.stamp - now > settings_.max_future_stamp) {
    return TrackUpdate::kFuture;
  }
  tracks_[track.id] = track;
  last_stamp_[track.id] = track.stamp;
  return TrackUpdate::kAccepted;
}

std::vector<uint32_t> SwimmerTrackStore::Prune(double now)
{
  std::vector<uint32_t> removed;
  for (auto it = tracks_.begin(); it != tracks_.end(); ) {
    if (now - it->second.stamp > settings_.max_track_age) {
      removed.push_back(it->first);
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
  return removed;
}

std::vector<SwimmerTrack> SwimmerTrackStore::Tracks() const
{
  std::vector<SwimmerTrack> tracks;
  for (const auto & entry : tracks_) {
    tracks.push_back(entry.second);
  }
  return tracks;
}

double FootprintDistance(
  const Eigen::Vector3d & pose, const Footprint & footprint, const Eigen::Vector2d & point)
{
  // The point in the body frame, then the distance to the rectangle.
  const Eigen::Vector2d body =
    Eigen::Rotation2Dd(-pose(2)) * (point - Eigen::Vector2d(pose(0), pose(1)));
  const double dx = std::max({-footprint.rear - body.x(), 0.0, body.x() - footprint.front});
  const double dy = std::max(std::abs(body.y()) - footprint.half_width, 0.0);
  return std::hypot(dx, dy);
}

std::optional<SwimmerConflict> FindSwimmerConflict(
  const std::vector<TimedPose> & trajectory, const Footprint & footprint,
  const std::vector<SwimmerTrack> & tracks, double now,
  const SwimmerPredictionSettings & settings)
{
  if (trajectory.empty() || tracks.empty()) {
    return std::nullopt;
  }
  const double end = now + settings.horizon;
  // Farthest outline point from base_link: bounds the sweep of a rotation.
  const double reach = std::hypot(std::max(footprint.front, footprint.rear), footprint.half_width);

  // One interval [t_lo, t_hi] with the vehicle at `pose` at its start and
  // moving by at most `motion` (m) within it. Every pose in the interval is
  // inside the outline at `pose` grown by `motion`, and every predicted disc
  // in the interval is inside the disc at t_hi (it grows at max_speed),
  // plus the track's own speed beyond max_speed for a fast track.
  auto check = [&](double t_lo, double t_hi, const Eigen::Vector3d & pose, double motion)
    -> std::optional<SwimmerConflict> {
      for (const SwimmerTrack & track : tracks) {
        const Disc disc = PredictSwimmerDisc(track, t_hi, settings);
        const double excess =
          std::max(0.0, track.velocity.norm() - settings.max_speed) * (t_hi - t_lo);
        const double gap = FootprintDistance(pose, footprint, disc.center);
        if (gap < disc.radius + motion + excess) {
          SwimmerConflict conflict;
          conflict.id = track.id;
          conflict.t = t_hi;
          conflict.gap = gap - motion;
          conflict.radius = disc.radius + excess;
          conflict.age = std::max(0.0, t_hi - track.stamp);
          conflict.robot = pose;
          conflict.swimmer = disc.center;
          return conflict;
        }
      }
      return std::nullopt;
    };

  if (trajectory.size() == 1 && trajectory.front().t >= now && trajectory.front().t <= end) {
    if (auto conflict = check(trajectory.front().t, trajectory.front().t,
      trajectory.front().pose, 0.0))
    {
      return conflict;
    }
  }
  for (std::size_t k = 0; k + 1 < trajectory.size(); ++k) {
    const TimedPose & a = trajectory[k];
    const TimedPose & b = trajectory[k + 1];
    if (b.t < now) {
      continue;  // already flown
    }
    if (a.t > end) {
      return std::nullopt;  // beyond the prediction horizon
    }
    const double motion = (b.pose.head<2>() - a.pose.head<2>()).norm() +
      reach * std::abs(std::remainder(b.pose(2) - a.pose(2), 2.0 * M_PI));
    if (auto conflict = check(std::max(a.t, now), std::min(b.t, end), a.pose, motion)) {
      return conflict;
    }
  }
  // The vehicle holds the last pose after the trajectory ends.
  const TimedPose & last = trajectory.back();
  for (double t = std::max(last.t, now); t < end - 1e-9; t += settings.time_step) {
    if (auto conflict = check(t, std::min(t + settings.time_step, end), last.pose, 0.0)) {
      return conflict;
    }
  }
  return std::nullopt;
}

}  // namespace rov_planning
