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

// Swimmer prediction, track store and timed clearance check (step 9).
// Units: m, m/s, s. Default settings are the design baseline: t_pred 2 s,
// v_swim 2 m/s, r_swim 1 m, max track age 1 s.

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "rov_planning/swimmer_prediction.hpp"

namespace rov_planning
{
namespace
{

SwimmerTrack Track(uint32_t id, double stamp, double x, double y, double vx, double vy)
{
  SwimmerTrack track;
  track.id = id;
  track.stamp = stamp;
  track.position = Eigen::Vector2d(x, y);
  track.velocity = Eigen::Vector2d(vx, vy);
  return track;
}

// Straight run along +x at `speed`, through (x0, 0) at t = 0, yaw 0,
// sampled every 0.05 s over [t_begin, t_end].
std::vector<TimedPose> StraightRun(double x0, double speed, double t_begin, double t_end)
{
  std::vector<TimedPose> samples;
  for (double t = t_begin; t <= t_end + 1e-9; t += 0.05) {
    samples.push_back({t, Eigen::Vector3d(x0 + speed * t, 0.0, 0.0)});
  }
  return samples;
}

TEST(SwimmerPredictionTest, DiscsGrowFromTheMeasurementTime)
{
  const SwimmerPredictionSettings settings;
  const SwimmerTrack track = Track(1, 10.0, 1.0, 2.0, 0.5, 0.0);
  const Disc at_measurement = PredictSwimmerDisc(track, 10.0, settings);
  EXPECT_TRUE(at_measurement.center.isApprox(Eigen::Vector2d(1.0, 2.0)));
  EXPECT_DOUBLE_EQ(at_measurement.radius, 1.0);  // r_swim only
  // Costmap discs from now = 10.5 (measurement 0.5 s old) over 2 s.
  const std::vector<Disc> discs = PredictSwimmerDiscs(track, 10.5, settings);
  ASSERT_EQ(discs.size(), 9u);
  EXPECT_TRUE(discs.front().center.isApprox(Eigen::Vector2d(1.25, 2.0)));
  EXPECT_DOUBLE_EQ(discs.front().radius, 1.0 + 2.0 * 0.5);
  EXPECT_TRUE(discs.back().center.isApprox(Eigen::Vector2d(2.25, 2.0)));
  EXPECT_DOUBLE_EQ(discs.back().radius, 1.0 + 2.0 * 2.5);
}

TEST(SwimmerPredictionTest, StoreRejectsStaleFutureAndOutOfOrderUpdates)
{
  SwimmerTrackStore store{SwimmerPredictionSettings{}};
  EXPECT_EQ(store.Update(Track(1, 9.8, 0, 0, 0, 0), 10.0), TrackUpdate::kAccepted);
  EXPECT_EQ(store.Update(Track(1, 9.7, 5, 5, 0, 0), 10.0), TrackUpdate::kOutOfOrder);
  EXPECT_EQ(store.Update(Track(1, 9.8, 5, 5, 0, 0), 10.0), TrackUpdate::kOutOfOrder);
  EXPECT_EQ(store.Update(Track(2, 8.5, 0, 0, 0, 0), 10.0), TrackUpdate::kStale);
  EXPECT_EQ(store.Update(Track(3, 10.5, 0, 0, 0, 0), 10.0), TrackUpdate::kFuture);
  EXPECT_EQ(
    store.Update(Track(4, std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0), 10.0),
    TrackUpdate::kInvalid);
  ASSERT_EQ(store.Tracks().size(), 1u);
  EXPECT_TRUE(store.Tracks()[0].position.isZero());  // the rejected update changed nothing
  // Expiry: the track goes, and an older measurement of it is still refused.
  EXPECT_EQ(store.Prune(11.0), std::vector<uint32_t>{1});
  EXPECT_TRUE(store.Tracks().empty());
  // 9.75 s is not stale at 10.5 s (0.75 s old), but older than the 9.8 s one.
  EXPECT_EQ(store.Update(Track(1, 9.75, 0, 0, 0, 0), 10.5), TrackUpdate::kOutOfOrder);
}

TEST(SwimmerPredictionTest, FootprintDistanceUsesTheRotatedOutline)
{
  const Footprint footprint{0.6, 0.5, 0.52};
  const Eigen::Vector3d pose(0.0, 0.0, M_PI / 2);  // bow towards +y
  EXPECT_NEAR(FootprintDistance(pose, footprint, Eigen::Vector2d(0.0, 1.6)), 1.0, 1e-12);
  EXPECT_NEAR(FootprintDistance(pose, footprint, Eigen::Vector2d(1.52, 0.0)), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(FootprintDistance(pose, footprint, Eigen::Vector2d(0.1, 0.1)), 0.0);
}

// The swimmer reaches the robot's route exactly when the robot does.
TEST(SwimmerPredictionTest, SimultaneousCrossingIsAConflict)
{
  const SwimmerPredictionSettings settings;
  // Robot through (3, 0) at t = 2 s; swimmer from (3, -1) along +y at
  // 0.5 m/s, also at (3, 0) at t = 2 s. Measured now (t = 0).
  const auto conflict = FindSwimmerConflict(
    StraightRun(0.0, 1.5, 0.0, 10.0), Footprint{}, {Track(7, 0.0, 3.0, -1.0, 0.0, 0.5)}, 0.0,
    settings);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(conflict->id, 7u);
  EXPECT_LE(conflict->t, 2.0);
  EXPECT_LT(conflict->gap, conflict->radius);
}

// The route passes the swimmer's current position, but only long after the
// prediction horizon: at matching times the robot is far away.
TEST(SwimmerPredictionTest, CrossingAtDifferentTimesIsNotAConflict)
{
  const SwimmerPredictionSettings settings;
  const SwimmerTrack swimmer = Track(8, 0.0, 12.0, -0.5, 0.0, 0.5);  // on the route now
  const std::vector<TimedPose> run = StraightRun(0.0, 1.0, 0.0, 15.0);  // there at t = 12 s
  // A time-blind check of the route against the swimmer's current disc fails...
  double closest = 1e9;
  for (const TimedPose & sample : run) {
    closest = std::min(closest, FootprintDistance(sample.pose, Footprint{}, swimmer.position));
  }
  EXPECT_LT(closest, settings.clearance);
  // ...but at matching times the robot stays outside every predicted disc.
  EXPECT_FALSE(FindSwimmerConflict(run, Footprint{}, {swimmer}, 0.0, settings).has_value());
  // The same check one step before the robot arrives (now = 10 s, fresh
  // measurement) does see the conflict.
  SwimmerTrack later = swimmer;
  later.stamp = 10.0;
  later.position = Eigen::Vector2d(12.0, -0.5);
  later.velocity = Eigen::Vector2d::Zero();
  EXPECT_TRUE(FindSwimmerConflict(run, Footprint{}, {later}, 10.0, settings).has_value());
}

// A delayed measurement predicts a larger disc at the same time.
TEST(SwimmerPredictionTest, DelayedTrackUpdateGivesALargerDisc)
{
  const SwimmerPredictionSettings settings;
  // The robot holds (0, 0) for the 0.5 s that remain of its trajectory;
  // a stationary swimmer is 3.2 m ahead of its side (gap 3.2 - 0.52 = 2.68 m).
  const std::vector<TimedPose> hold = {
    {10.0, Eigen::Vector3d::Zero()}, {10.5, Eigen::Vector3d::Zero()}};
  SwimmerPredictionSettings short_horizon = settings;
  short_horizon.horizon = 0.5;
  // Fresh measurement (stamp = now): radius 1 + 2 * 0.5 = 2.0 m < 2.68 m.
  EXPECT_FALSE(
    FindSwimmerConflict(
      hold, Footprint{}, {Track(9, 10.0, 0.0, 3.2, 0.0, 0.0)}, 10.0, short_horizon).has_value());
  // The same swimmer measured 0.6 s ago: radius 1 + 2 * 1.1 = 3.2 m > 2.68 m.
  const auto conflict = FindSwimmerConflict(
    hold, Footprint{}, {Track(9, 9.4, 0.0, 3.2, 0.0, 0.0)}, 10.0, short_horizon);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_NEAR(conflict->age, 1.1, 1e-9);
}

// Samples before now are not checked (already flown), and the final pose is
// held after the trajectory ends.
TEST(SwimmerPredictionTest, PastSamplesSkippedAndFinalPoseHeld)
{
  const SwimmerPredictionSettings settings;
  // The swimmer sits where the robot was at t = 0..1 s.
  const SwimmerTrack swimmer = Track(1, 5.0, 0.0, 0.0, 0.0, 0.0);
  const std::vector<TimedPose> run = StraightRun(0.0, 1.0, 0.0, 2.0);  // ends at (2, 0)
  EXPECT_TRUE(FindSwimmerConflict(run, Footprint{}, {swimmer}, 0.0, settings).has_value());
  // At now = 5 s the run is over and the robot holds (2, 0): gap 1.5 m to the
  // swimmer, disc 1 m now but 1 + 2 * 2 m at the end of the horizon.
  EXPECT_TRUE(FindSwimmerConflict(run, Footprint{}, {swimmer}, 5.0, settings).has_value());
  SwimmerPredictionSettings no_growth = settings;
  no_growth.max_speed = 0.0;
  EXPECT_FALSE(FindSwimmerConflict(run, Footprint{}, {swimmer}, 5.0, no_growth).has_value());
}

// A conflict between two samples (1 s apart) is found, although the gap is
// large enough at both sample instants.
TEST(SwimmerPredictionTest, ConflictBetweenSamplesIsFound)
{
  SwimmerPredictionSettings settings;
  settings.max_speed = 0.0;  // fixed 1 m disc: only the sampling matters here
  const std::vector<TimedPose> coarse = {
    {0.0, Eigen::Vector3d(0.0, 0.0, 0.0)}, {1.0, Eigen::Vector3d(3.0, 0.0, 0.0)}};
  const SwimmerTrack swimmer = Track(3, 0.0, 1.5, 1.4, 0.0, 0.0);
  const Footprint footprint{0.6, 0.5, 0.52};
  // At t = 0 and t = 1 the hull is more than 1 m away...
  EXPECT_GT(FootprintDistance(coarse[0].pose, footprint, swimmer.position), 1.2);
  EXPECT_GT(FootprintDistance(coarse[1].pose, footprint, swimmer.position), 1.2);
  // ...but at t = 0.5 it passes 0.88 m from the swimmer.
  EXPECT_LT(FootprintDistance(Eigen::Vector3d(1.5, 0.0, 0.0), footprint, swimmer.position), 1.0);
  EXPECT_TRUE(FindSwimmerConflict(coarse, footprint, {swimmer}, 0.0, settings).has_value());
  // The same run sampled finely agrees.
  std::vector<TimedPose> fine;
  for (double t = 0.0; t <= 1.0 + 1e-9; t += 0.05) {
    fine.push_back({t, Eigen::Vector3d(3.0 * t, 0.0, 0.0)});
  }
  EXPECT_TRUE(FindSwimmerConflict(fine, footprint, {swimmer}, 0.0, settings).has_value());
  // The interval that straddles "now" is checked too.
  EXPECT_TRUE(FindSwimmerConflict(coarse, footprint, {swimmer}, 0.4, settings).has_value());
}

TEST(SwimmerPredictionTest, InvalidSettingsAreRejected)
{
  SwimmerPredictionSettings settings;
  settings.time_step = 0.0;
  EXPECT_FALSE(ValidateSwimmerPredictionSettings(settings).empty());
  EXPECT_THROW(SwimmerTrackStore{settings}, std::invalid_argument);
}

}  // namespace
}  // namespace rov_planning
