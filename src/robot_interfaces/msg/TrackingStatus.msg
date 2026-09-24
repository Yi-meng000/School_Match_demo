# Current state of the path-tracking controller.  Distances use metres and
# speeds use metres per second in the path/world frame.
uint8 DISABLED=0
uint8 WAITING_FOR_ODOMETRY=1
uint8 WAITING_FOR_PATH=2
uint8 TRACKING=3
uint8 GOAL_REACHED=4
uint8 PATH_REJECTED=5
uint8 EMERGENCY_BRAKING=6
uint8 EMERGENCY_STOP=7
uint8 MPC_FAILURE=8
uint8 ODOMETRY_TIMEOUT=9

std_msgs/Header header
bool has_path
uint64 path_id
uint8 state
float64 progress
float64 remaining_distance
float64 stop_deficit
float64 terminal_speed
string detail
