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

#include "rov_planning/swimmer_tracks_ros.hpp"

#include <string>

#include "rclcpp/time.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer_interface.hpp"

namespace rov_planning
{

TrackConversion TracksFromMessage(
  const rov_interfaces::msg::SwimmerTrackArray & message, const std::string & target_frame,
  const tf2_ros::Buffer & tf)
{
  TrackConversion conversion;
  for (const auto & track : message.tracks) {
    const bool own_stamp = track.header.stamp.sec != 0 || track.header.stamp.nanosec != 0;
    const builtin_interfaces::msg::Time & stamp =
      own_stamp ? track.header.stamp : message.header.stamp;
    const std::string & frame =
      track.header.frame_id.empty() ? message.header.frame_id : track.header.frame_id;
    // A malformed stamp would throw in rclcpp::Time: drop the track instead.
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u || (stamp.sec == 0 && stamp.nanosec == 0)) {
      conversion.errors.push_back(
        "track " + std::to_string(track.id) + ": malformed or missing stamp (" +
        std::to_string(stamp.sec) + " s, " + std::to_string(stamp.nanosec) + " ns)");
      continue;
    }
    if (frame.empty()) {
      conversion.errors.push_back("track " + std::to_string(track.id) + ": no frame_id");
      continue;
    }
    SwimmerTrack out;
    out.id = track.id;
    out.stamp = rclcpp::Time(stamp).seconds();
    out.position = Eigen::Vector2d(track.position.x, track.position.y);
    out.velocity = Eigen::Vector2d(track.velocity.x, track.velocity.y);
    if (frame != target_frame) {
      try {
        const auto transform =
          tf.lookupTransform(target_frame, frame, tf2_ros::fromMsg(stamp));
        const Eigen::Rotation2Dd rotation(tf2::getYaw(transform.transform.rotation));
        out.position = rotation * out.position +
          Eigen::Vector2d(transform.transform.translation.x, transform.transform.translation.y);
        out.velocity = rotation * out.velocity;
      } catch (const tf2::TransformException & error) {
        conversion.errors.push_back(
          "track " + std::to_string(track.id) + " (frame '" + frame + "'): " + error.what());
        continue;
      }
    }
    conversion.tracks.push_back(out);
  }
  return conversion;
}

}  // namespace rov_planning
