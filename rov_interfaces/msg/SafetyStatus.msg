# Output of the independent safety monitor (design 5), published at a fixed
# rate on "safety_status". Several rules can be active at once; every action
# below is the combination of all active rules.

std_msgs/Header header

bool speed_limited
float64 speed_limit                # m/s, the lowest active limit (if speed_limited)

bool zero_thrust                   # all thruster forces 0 (odometry timeout)
bool stop_hold                     # brake to a stop and hold
bool replan_now                    # replan immediately
bool replan_slow                   # replan with slow_speed_factor * v_cruise
float64 slow_speed_factor          # e.g. 0.7

bool abort                         # abort the mission
uint8 ABORT_NONE=0
uint8 ABORT_HOLD=1                 # hold and wait for manual takeover
uint8 ABORT_RETURN=2               # return to the shore
uint8 ABORT_SURFACE_RETURN=3       # surface, then return
uint8 abort_kind

string[] active_rules              # names of the triggered rules
string reason                      # one line per active rule, joined
