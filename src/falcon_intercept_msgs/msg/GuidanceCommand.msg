uint8 STATE_INIT=0
uint8 STATE_SEARCH=1
uint8 STATE_TRACK=2
uint8 STATE_APPROACH=3
uint8 STATE_FIRE=4
uint8 STATE_LAND=5
uint8 STATE_DONE=6
uint8 STATE_ABORT=7
uint8 STATE_EGRESS=8
uint8 STATE_RECOVER=9

std_msgs/Header header
uint32 sequence
uint8 mission_state
# ROS body FLU: +X forward, +Y left, +Z up.
geometry_msgs/Vector3 velocity_body_flu
# Right-hand positive about FLU +Z (counter-clockwise viewed from above).
float32 yaw_rate_flu
# Receiver must replace the command with zero when this interval expires.
float32 valid_for_s
bool valid
bool land_request
string reason
