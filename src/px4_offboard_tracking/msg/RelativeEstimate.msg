std_msgs/Header header
# All kinematic states are target minus pursuer, expressed in the common NED frame.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Relative acceleration only. Reconstruct target acceleration by adding the
# pursuer's measured common-frame acceleration, then apply confidence gating.
geometry_msgs/Vector3 acceleration
# Legacy 6x6 position/velocity covariance, row major.
float64[36] covariance
# Complete 9x9 position/velocity/acceleration covariance, row major.
float64[81] full_covariance
float64[3] innovation
# Deprecated compatibility alias for normalized_innovation_squared.
float64 innovation_mahalanobis
float64 normalized_innovation_squared
float64[3] model_probabilities  # [cv, ca, ct]
# Seconds from the last accepted depth measurement to the published state time.
float64 data_age
float64 coast_time_s
bool measurement_accepted
bool valid
