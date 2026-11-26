# One sample of a planar (x, y, yaw) trajectory.
# Frame: the parent Trajectory's header.frame_id (map: pool-centred,
#   x along the pool length, y across, z up).

builtin_interfaces/Duration time_from_start  # since Trajectory.header.stamp; >= 0

float64 x      # m
float64 y      # m
float64 vx     # m/s, in the map frame (not body frame)
float64 vy     # m/s, in the map frame
float64 ax     # m/s^2, in the map frame
float64 ay     # m/s^2, in the map frame
float64 yaw    # rad, heading of base_link x axis from map x axis, CCW positive.
               # Continuous (unwrapped) along the trajectory, so it may leave [-pi, pi].
float64 yaw_rate  # rad/s, CCW positive
