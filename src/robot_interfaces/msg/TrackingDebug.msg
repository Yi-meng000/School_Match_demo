# Temporary per-control-tick tracing telemetry. The header frame is the shared
# world/path frame. All planar Twist fields are expressed in that world frame,
# matching the values supplied to and returned by MpcController. Angular z is
# the planar yaw rate. /cmd_track is converted separately to the chassis frame.
uint8 NO_ACTIVE_PATH=0
uint8 READY=1
uint8 PATH_REJECTED=2
uint8 EMERGENCY_BRAKING=3
uint8 EMERGENCY_INFEASIBLE=4

std_msgs/Header header
uint8 trajectory_status
float64 progress
float64 remaining_distance

# Persistent nominal speed schedule: v_des(s). nominal_phase_* is the speed
# phase selected for this horizon; nominal_speed_at_progress is v_des at the
# current measured path projection.
float64 nominal_phase_arc_length
float64 nominal_phase_speed
float64 nominal_speed_at_progress

# First final reference point actually supplied to the MPC (dt in the future).
float64 reference_arc_length
float64 reference_speed
bool reachability_limited
bool spatial_safety_limited

geometry_msgs/Twist nominal_reference
geometry_msgs/Twist final_reference
geometry_msgs/Twist command
geometry_msgs/Twist measured
