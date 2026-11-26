# One non-target swimmer track from the shore tracker (design 3 item 2, 6.4).
#
# header.stamp: time of the measurement (not of publishing).
# header.frame_id: frame of position and velocity (the map frame in use).

std_msgs/Header header

# Stable ID: the same swimmer keeps the same ID for its whole track.
uint32 id

# Planar position and velocity in header.frame_id; z is unused (0).
geometry_msgs/Point position     # m
geometry_msgs/Vector3 velocity   # m/s
