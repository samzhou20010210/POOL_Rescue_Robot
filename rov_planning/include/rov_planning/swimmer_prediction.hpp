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

// Non-target swimmer prediction and clearance checks (design section 3
// item 2, implementation step 9). Shared by the swimmer_layer costmap plugin
// and trajectory_generator_node. No ROS.
//
// Times are seconds on one clock (ROS time in the nodes). A track is one
// measurement: position p and velocity v at its measurement time t_m. At a
// time t >= t_m the swimmer is predicted inside the disc
//   centre  p + v (t - t_m)
//   radius  clearance + max_speed (t - t_m)
// so the radius grows with the assumed maximum swimmer speed (the swimmer
// may change direction), and an older measurement gives a larger disc.
// Predictions are used over [now, now + horizon].

#ifndef ROV_PLANNING__SWIMMER_PREDICTION_HPP_
#define ROV_PLANNING__SWIMMER_PREDICTION_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Dense"

namespace rov_planning
{

// config/swimmer_prediction.yaml.
struct SwimmerPredictionSettings
{
  double horizon = 2.0;           // t_pred, s (design 6.5)
  double max_speed = 2.0;         // v_swim, m/s (design 6.5): radius growth rate
  double clearance = 1.0;         // r_swim, m (design 6.7): hull edge to swimmer
  double time_step = 0.25;        // s between the discs marked in the costmap
  double max_track_age = 1.0;     // s: an older measurement is stale
  double max_future_stamp = 0.1;  // s: a stamp further ahead of now is rejected
};

// Read the settings; every key is required. Throws std::runtime_error if a
// key is missing or the settings are invalid.
SwimmerPredictionSettings LoadSwimmerPredictionSettings(const std::string & yaml_path);

// Empty string if the settings are usable, otherwise the reason.
std::string ValidateSwimmerPredictionSettings(const SwimmerPredictionSettings & settings);

// One swimmer measurement, in the planning (map) frame.
struct SwimmerTrack
{
  uint32_t id = 0;
  double stamp = 0.0;                                  // t_m, s
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  // m
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();  // m/s
};

struct Disc
{
  Eigen::Vector2d center = Eigen::Vector2d::Zero();  // m
  double radius = 0.0;                               // m
};

// Predicted disc at time t (times before the measurement count as t_m).
Disc PredictSwimmerDisc(
  const SwimmerTrack & track, double t, const SwimmerPredictionSettings & settings);

// Discs every time_step over [now, now + horizon] (both ends included).
std::vector<Disc> PredictSwimmerDiscs(
  const SwimmerTrack & track, double now, const SwimmerPredictionSettings & settings);

enum class TrackUpdate
{
  kAccepted,
  kInvalid,      // non-finite position, velocity or stamp
  kOutOfOrder,   // not newer than the last accepted measurement of this ID
  kStale,        // older than max_track_age
  kFuture,       // more than max_future_stamp ahead of now
};

const char * TrackUpdateToString(TrackUpdate update);

// Latest measurement per track ID. Not thread-safe.
class SwimmerTrackStore
{
public:
  explicit SwimmerTrackStore(const SwimmerPredictionSettings & settings);

  // Accept the measurement if it is valid, not stale, not from the future
  // and newer than the last accepted one with the same ID.
  TrackUpdate Update(const SwimmerTrack & track, double now);

  // Remove tracks whose latest measurement is older than max_track_age;
  // returns their IDs. The last stamp of a removed ID is still remembered,
  // so an older measurement of it is still rejected as out of order.
  std::vector<uint32_t> Prune(double now);

  std::vector<SwimmerTrack> Tracks() const;
  const SwimmerPredictionSettings & settings() const {return settings_;}

private:
  SwimmerPredictionSettings settings_;
  std::map<uint32_t, SwimmerTrack> tracks_;
  std::map<uint32_t, double> last_stamp_;
};

// Rectangular hull outline in base_link (x forward), m.
struct Footprint
{
  double front = 0.6;        // bow (incl. stowed arms) ahead of base_link
  double rear = 0.5;         // stern behind base_link
  double half_width = 0.52;  // half of the beam
};

// Distance (m) from the footprint at pose (x, y, yaw) to a point; 0 inside.
double FootprintDistance(
  const Eigen::Vector3d & pose, const Footprint & footprint, const Eigen::Vector2d & point);

// One trajectory sample: absolute time (s) and pose (x, y, yaw).
struct TimedPose
{
  double t = 0.0;
  Eigen::Vector3d pose = Eigen::Vector3d::Zero();
};

struct SwimmerConflict
{
  uint32_t id = 0;
  double t = 0.0;        // s, absolute time of the conflict
  double gap = 0.0;      // m, hull edge to the predicted swimmer position
  double radius = 0.0;   // m, predicted disc radius (the gap needed)
  double age = 0.0;      // s, t minus the measurement time
  Eigen::Vector3d robot = Eigen::Vector3d::Zero();
  Eigen::Vector2d swimmer = Eigen::Vector2d::Zero();
};

// Earliest conflict between the footprint along the time-stamped
// trajectory and the predicted discs at the same times, over
// [now, now + horizon]: a conflict is a hull-to-centre gap smaller than the
// disc radius. Every interval between consecutive samples that overlaps the
// window is covered (not only the sample instants): the outline at the
// interval's first sample is grown by the vehicle's motion within the
// interval (translation + rotation sweep) and compared with the disc at the
// interval's end, which contains the earlier discs. Intervals that ended
// before now are skipped. After the last sample the vehicle is taken to hold
// the last pose, checked in time_step intervals up to now + horizon.
// Samples must be in time order.
std::optional<SwimmerConflict> FindSwimmerConflict(
  const std::vector<TimedPose> & trajectory, const Footprint & footprint,
  const std::vector<SwimmerTrack> & tracks, double now,
  const SwimmerPredictionSettings & settings);

}  // namespace rov_planning

#endif  // ROV_PLANNING__SWIMMER_PREDICTION_HPP_
