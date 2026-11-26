# All swimmer tracks currently seen by the shore tracker, published at
# f_swim (design 6.4: 10 Hz). A track missing from later messages is lost;
# consumers drop it after their own timeout.

# header.stamp: publishing time. header.frame_id: frame shared by all tracks.
std_msgs/Header header

SwimmerTrack[] tracks
