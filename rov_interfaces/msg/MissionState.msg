# Mission phase (design 2), published on "mission_state" on every transition
# and at 1 Hz.

std_msgs/Header header

uint8 STANDBY=0
uint8 TRANSIT=1
uint8 FINAL_APPROACH=2
uint8 DIVE_AND_GRASP=3
uint8 SURFACE_HOLD=4
uint8 RETURN=5
uint8 SAFETY_STOP=6
uint8 phase
uint8 preempted_phase              # the phase SAFETY_STOP interrupted (if in SAFETY_STOP)

uint8 victim_state                 # VictimTarget.SURFACE / SUBMERGED
bool aborted                       # the mission was aborted (no resume)
string last_transition             # "t = ... s: FROM -> TO (reason)"
