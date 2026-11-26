# Victim position and state from the shore (design 3 item 1, 6.4).
# Used for the shore alarm ("shore_alarm") and for target updates ("victim").
#
# header.stamp: measurement time; header.frame_id: the map frame.

std_msgs/Header header

geometry_msgs/Point position   # m; z unused (0)

uint8 SURFACE=0
uint8 SUBMERGED=1
uint8 victim_state
