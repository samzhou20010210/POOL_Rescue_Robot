# Time-parameterized planar trajectory, sampled.
#
# Timing: point k is the reference at absolute time
#   header.stamp + points[k].time_from_start.
#   points[0].time_from_start = 0; times strictly increase.
#   The controller interpolates between samples; after the last point
#   it holds the last point (terminal state: v = 0, a = 0).

# header.stamp: absolute start time t0 (ROS time; sim time if use_sim_time).
# header.frame_id: map
std_msgs/Header header

TrajectoryPoint[] points
