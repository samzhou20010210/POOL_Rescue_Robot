# Simulated (or operator) mission event, on "mission_event".

std_msgs/Header header             # stamp: when it happened

uint8 RESCUE_COMPLETE=0            # victim lifted (DIVE_AND_GRASP) / secured (SURFACE_HOLD)
uint8 MANUAL_TAKEOVER=1            # operator takes over after an abort-and-hold
uint8 type

string detail
