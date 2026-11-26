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

// SwimmerLayer: Nav2 costmap layer that keeps planned paths away from the
// predicted positions of non-target swimmers (design 3 item 2, step 9).
//
// It subscribes to rov_interfaces/SwimmerTrackArray (parameter `topic`) and
// keeps the latest measurement of each track ID in a SwimmerTrackStore,
// shared with trajectory_generator_node (swimmer_prediction.hpp):
//   - tracks are transformed into the costmap frame with TF at their
//     measurement stamp (swimmer_tracks_ros.hpp);
//   - stale, future-stamped and out-of-order measurements are rejected;
//   - a track whose latest measurement is older than max_track_age_s is
//     dropped.
// On every costmap update it marks, for every track, the predicted discs
// over [now, now + t_pred] as LETHAL_OBSTACLE (cell centres inside a disc).
// The discs are predicted from the measurement time, so a delayed
// measurement gives larger discs.
//
// Clearing: the layer never stores costs of its own. It only raises master
// costs (never lowers them), so static obstacles are preserved, and its
// update bounds always include the area it marked on the previous update.
// The layered costmap resets that area and the static layer repaints it, so
// the marks of a moved or expired track disappear on the next update.
//
// Parameters (under the layer's name): enabled, topic, prediction_file
// (config/swimmer_prediction.yaml of rov_planning if empty).

#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/msg/swimmer_track_array.hpp"
#include "rov_planning/swimmer_prediction.hpp"
#include "rov_planning/swimmer_tracks_ros.hpp"

namespace rov_planning
{

class SwimmerLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("SwimmerLayer: no costmap node");
    }
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("topic", rclcpp::ParameterValue(std::string("/swimmer_tracks")));
    declareParameter("prediction_file", rclcpp::ParameterValue(std::string("")));
    std::string topic;
    std::string prediction_file;
    node->get_parameter(getFullName("enabled"), enabled_);
    node->get_parameter(getFullName("topic"), topic);
    node->get_parameter(getFullName("prediction_file"), prediction_file);
    if (prediction_file.empty()) {
      prediction_file = ament_index_cpp::get_package_share_directory("rov_planning") +
        "/config/swimmer_prediction.yaml";
    }
    store_.emplace(LoadSwimmerPredictionSettings(prediction_file));
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    subscription_ = node->create_subscription<rov_interfaces::msg::SwimmerTrackArray>(
      topic, rclcpp::SensorDataQoS(),
      [this](const rov_interfaces::msg::SwimmerTrackArray & message) {OnTracks(message);});
    current_ = true;
    const SwimmerPredictionSettings & s = store_->settings();
    RCLCPP_INFO(
      logger_, "SwimmerLayer %s: %s, horizon %.2f s, v_swim %.2f m/s, clearance %.2f m, "
      "step %.2f s, max track age %.2f s (%s)", name_.c_str(), topic.c_str(), s.horizon,
      s.max_speed, s.clearance, s.time_step, s.max_track_age, prediction_file.c_str());
  }

  void updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x,
    double * max_y) override
  {
    if (!enabled_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = clock_->now().seconds();
    for (const uint32_t id : store_->Prune(now)) {
      RCLCPP_INFO(logger_, "SwimmerLayer: track %u expired", id);
    }
    // Predicted discs of all live tracks over [now, now + t_pred].
    discs_.clear();
    for (const SwimmerTrack & track : store_->Tracks()) {
      const std::vector<Disc> discs = PredictSwimmerDiscs(track, now, store_->settings());
      discs_.insert(discs_.end(), discs.begin(), discs.end());
    }
    // Bounds: this update's discs plus last update's (so they are cleared).
    Bounds marked;
    for (const Disc & d : discs_) {
      marked.Add(d.center - Eigen::Vector2d::Constant(d.radius));
      marked.Add(d.center + Eigen::Vector2d::Constant(d.radius));
    }
    for (const Bounds * b : {&marked, &last_marked_}) {
      if (b->valid) {
        *min_x = std::min(*min_x, b->low.x());
        *min_y = std::min(*min_y, b->low.y());
        *max_x = std::max(*max_x, b->high.x());
        *max_y = std::max(*max_y, b->high.y());
      }
    }
    last_marked_ = marked;
    current_ = true;
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override
  {
    if (!enabled_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const Disc & d : discs_) {
      // Cells whose centre lies inside the disc, within the update window.
      int i0, j0, i1, j1;
      master_grid.worldToMapEnforceBounds(d.center.x() - d.radius, d.center.y() - d.radius, i0, j0);
      master_grid.worldToMapEnforceBounds(d.center.x() + d.radius, d.center.y() + d.radius, i1, j1);
      i0 = std::max(i0, min_i);
      j0 = std::max(j0, min_j);
      i1 = std::min(i1, max_i - 1);
      j1 = std::min(j1, max_j - 1);
      const double radius_squared = d.radius * d.radius;
      for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
          double x, y;
          master_grid.mapToWorld(i, j, x, y);
          if ((Eigen::Vector2d(x, y) - d.center).squaredNorm() > radius_squared) {
            continue;
          }
          // Only raise the cost: static obstacles stay as they are.
          const unsigned char cost = master_grid.getCost(i, j);
          if (cost == nav2_costmap_2d::NO_INFORMATION ||
            cost < nav2_costmap_2d::LETHAL_OBSTACLE)
          {
            master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
          }
        }
      }
    }
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    store_.emplace(store_->settings());
    discs_.clear();
    current_ = true;
  }

  bool isClearable() override {return true;}

private:
  struct Bounds
  {
    bool valid = false;
    Eigen::Vector2d low = Eigen::Vector2d::Zero();
    Eigen::Vector2d high = Eigen::Vector2d::Zero();
    void Add(const Eigen::Vector2d & p)
    {
      low = valid ? low.cwiseMin(p) : p;
      high = valid ? high.cwiseMax(p) : p;
      valid = true;
    }
  };

  void OnTracks(const rov_interfaces::msg::SwimmerTrackArray & message)
  {
    // TF at each measurement stamp, outside the lock.
    const TrackConversion conversion =
      TracksFromMessage(message, layered_costmap_->getGlobalFrameID(), *tf_);
    for (const std::string & error : conversion.errors) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "SwimmerLayer: dropped %s", error.c_str());
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = clock_->now().seconds();
    for (const SwimmerTrack & track : conversion.tracks) {
      const TrackUpdate update = store_->Update(track, now);
      if (update != TrackUpdate::kAccepted) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000, "SwimmerLayer: track %u update rejected: %s (age %.3f s)",
          track.id, TrackUpdateToString(update), now - track.stamp);
      }
    }
  }

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("swimmer_layer")};
  rclcpp::Subscription<rov_interfaces::msg::SwimmerTrackArray>::SharedPtr subscription_;

  // Written by the subscription, read by the costmap updates.
  std::mutex mutex_;
  std::optional<SwimmerTrackStore> store_;
  std::vector<Disc> discs_;
  Bounds last_marked_;
};

}  // namespace rov_planning

PLUGINLIB_EXPORT_CLASS(rov_planning::SwimmerLayer, nav2_costmap_2d::Layer)
