# Target position reconstructed by the perception workflow.
# Coordinate convention: optical frame, +X right, +Y down, +Z forward.
std_msgs/Header header
uint32 sequence
geometry_msgs/Point position_camera
float64[9] position_covariance
float32 confidence
bool valid
