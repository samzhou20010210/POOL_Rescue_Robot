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

// ROS side of the swimmer tracks, shared by the swimmer_layer costmap plugin
// and trajectory_generator_node (implementation step 9).

#ifndef ROV_PLANNING__SWIMMER_TRACKS_ROS_HPP_
#define ROV_PLANNING__SWIMMER_TRACKS_ROS_HPP_

#include <string>
#include <vector>

#include "rov_interfaces/msg/swimmer_track_array.hpp"
#include "rov_planning/swimmer_prediction.hpp"
#include "tf2_ros/buffer.hpp"

namespace rov_planning
{

struct TrackConversion
{
  std::vector<SwimmerTrack> tracks;  // in target_frame, stamped with t_m
  std::vector<std::string> errors;   // one line per track that was dropped
};

// Converts every track of the message into target_frame. Each track uses its
// own header (or the message header if the track's stamp is zero or its
// frame is empty). A track whose stamp is negative, has nanosec >= 1e9 or is
// zero in both headers, or whose frame is empty in both, is dropped with a
// reason. The transform is looked up AT THE MEASUREMENT STAMP, so a
// moving frame is applied as it was when the swimmer was measured; a track
// whose transform is not available at that time is dropped. Only the planar
// part (x, y, yaw) of the transform is used, and the velocity is only
// rotated (the motion of the frame itself is not added).
TrackConversion TracksFromMessage(
  const rov_interfaces::msg::SwimmerTrackArray & message, const std::string & target_frame,
  const tf2_ros::Buffer & tf);

}  // namespace rov_planning

#endif  // ROV_PLANNING__SWIMMER_TRACKS_ROS_HPP_
