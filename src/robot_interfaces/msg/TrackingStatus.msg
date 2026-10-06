uint8 DISABLED=0
uint8 WAITING_FOR_ODOMETRY=1
uint8 WAITING_FOR_PATH=2
uint8 TRACKING=3
uint8 GOAL_REACHED=4
uint8 PATH_REJECTED=5
uint8 MPC_FAILURE=6
uint8 ODOMETRY_TIMEOUT=7

std_msgs/Header header
bool has_path
uint64 path_id
uint8 state
float64 progress
float64 remaining_distance
float64 endpoint_distance
string detail
