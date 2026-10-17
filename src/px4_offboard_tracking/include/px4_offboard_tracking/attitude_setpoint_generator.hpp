#pragma once

#include <array>

#include "px4_offboard_tracking/geometric_control.hpp"

namespace px4_offboard_tracking
{

struct AttitudeSetpointGeneratorConfig
{
  double hover_thrust{0.42};
  double minimum_thrust{0.1};
  double maximum_thrust{0.70};
  double smoothing_time_constant_s{0.25};
  double maximum_attitude_slew_rate_rad_s{1.5};
};

class AttitudeSetpointGenerator
{
public:
  explicit AttitudeSetpointGenerator(const AttitudeSetpointGeneratorConfig & config);

  AttitudeThrustCommand update(
    const std::array<double, 3> & desired_camera_axis_ned,
    double desired_yaw_rad,
    double desired_vertical_acceleration_mps2,
    const Quaternion & current_body_to_ned,
    double delta_time_s);

  void reset();

private:
  AttitudeSetpointGeneratorConfig config_;
  Quaternion previous_attitude_{};
  double previous_thrust_{};
  bool initialized_{false};
};

}  // namespace px4_offboard_tracking

