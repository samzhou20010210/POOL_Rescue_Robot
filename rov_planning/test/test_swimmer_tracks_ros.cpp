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

// Swimmer track messages are transformed with TF at the measurement stamp.

#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rov_planning/swimmer_tracks_ros.hpp"
#include "tf2_ros/buffer.hpp"

namespace rov_planning
{
namespace
{

geometry_msgs::msg::TransformStamped Shift(double stamp, double x)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
  t.header.frame_id = "map";
  t.child_frame_id = "shore";
  t.transform.translation.x = x;
  t.transform.rotation.w = 1.0;
  return t;
}

TEST(SwimmerTracksRosTest, TransformIsLookedUpAtTheMeasurementStamp)
{
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  buffer.setTransform(Shift(100.0, 0.0), "test");  // the shore frame moves +1 m/s in x
  buffer.setTransform(Shift(101.0, 1.0), "test");

  rov_interfaces::msg::SwimmerTrackArray message;
  message.header.stamp = rclcpp::Time(static_cast<int64_t>(101.0e9));  // published late
  message.header.frame_id = "shore";
  rov_interfaces::msg::SwimmerTrack track;
  track.id = 4;
  track.header.stamp = rclcpp::Time(static_cast<int64_t>(100.25e9));  // measured earlier
  track.header.frame_id = "shore";
  track.velocity.y = 0.5;
  message.tracks.push_back(track);
  rov_interfaces::msg::SwimmerTrack unstamped;  // falls back to the message header
  unstamped.id = 5;
  message.tracks.push_back(unstamped);

  const TrackConversion conversion = TracksFromMessage(message, "map", buffer);
  ASSERT_TRUE(conversion.errors.empty());
  ASSERT_EQ(conversion.tracks.size(), 2u);
  EXPECT_DOUBLE_EQ(conversion.tracks[0].stamp, 100.25);
  EXPECT_NEAR(conversion.tracks[0].position.x(), 0.25, 1e-9);  // frame as at 100.25 s
  EXPECT_NEAR(conversion.tracks[0].velocity.y(), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(conversion.tracks[1].stamp, 101.0);
  EXPECT_NEAR(conversion.tracks[1].position.x(), 1.0, 1e-9);   // frame as at 101 s

  // No transform at the stamp: the track is dropped with a reason.
  message.tracks[0].header.stamp = rclcpp::Time(static_cast<int64_t>(105.0e9));
  const TrackConversion late = TracksFromMessage(message, "map", buffer);
  EXPECT_EQ(late.tracks.size(), 1u);
  EXPECT_EQ(late.errors.size(), 1u);
}

TEST(SwimmerTracksRosTest, MalformedStampsAreDroppedNotThrown)
{
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  rov_interfaces::msg::SwimmerTrackArray message;
  message.header.frame_id = "map";
  rov_interfaces::msg::SwimmerTrack negative;
  negative.id = 1;
  negative.header.stamp.sec = -5;
  rov_interfaces::msg::SwimmerTrack bad_nanosec;
  bad_nanosec.id = 2;
  bad_nanosec.header.stamp.sec = 10;
  bad_nanosec.header.stamp.nanosec = 1500000000u;
  rov_interfaces::msg::SwimmerTrack unstamped;  // and the message has no stamp either
  unstamped.id = 3;
  message.tracks = {negative, bad_nanosec, unstamped};
  TrackConversion conversion;
  ASSERT_NO_THROW(conversion = TracksFromMessage(message, "map", buffer));
  EXPECT_TRUE(conversion.tracks.empty());
  EXPECT_EQ(conversion.errors.size(), 3u);
}

}  // namespace
}  // namespace rov_planning
