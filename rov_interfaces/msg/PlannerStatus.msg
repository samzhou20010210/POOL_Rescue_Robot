# One event of the trajectory generator (design 3, steps 8B-10). Published on
# "planner_status" next to the human-readable "planner_report".

std_msgs/Header header             # stamp: time of the event

uint8 PLANNED=0                    # first trajectory for a goal sent and accepted
uint8 REPLANNED=1                  # replacement sent and accepted
uint8 REPLAN_FAILED=2              # attempt failed; see detail (the active goal is kept if valid)
uint8 REJECTED=3                   # goal refused (gate) or given up
uint8 BLOCKED=4                    # a trajectory was not sent: predicted swimmer conflict
uint8 BRAKING=5                    # braking to a stop (fallback)
uint8 HOLDING=6                    # stopped and holding position
uint8 ARRIVED=7                    # controller finished; see verified
uint8 SWIMMER_CONFLICT=8           # conflict predicted on the active trajectory
uint8 event

uint64 request                     # planning request ID
uint64 goal_id                     # ID of the goal (victim target or pose goal) it serves

uint8 GOAL_NONE=0
uint8 GOAL_VICTIM=1
uint8 GOAL_POSE=2
uint8 goal_kind

float64 terminal_x                 # m, map frame: terminal pose of the goal
float64 terminal_y                 # m
float64 terminal_yaw               # rad

bool verified                      # ARRIVED: acceptance and clearance passed
float64 position_error             # m, ARRIVED: to the latest applicable terminal pose
float64 yaw_error                  # rad, ARRIVED
float64 max_speed                  # m/s, PLANNED / REPLANNED: highest planned speed
bool trajectory_active             # a valid trajectory (or hold) is being executed

string detail
