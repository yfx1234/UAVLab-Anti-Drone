#pragma once

#include <array>

#include "px4_offboard_tracking/geometric_control.hpp"

namespace px4_offboard_tracking
{

struct ZAxisAlignmentConfig
{
  double prediction_time_s{0.8};
  double maximum_tilt_rad{0.35};
  double minimum_target_above_m{0.5};
  double alignment_entry_horizontal_distance_m{2.0};
  double alignment_entry_relative_speed_mps{1.0};
};

struct ZAxisAlignmentResult
{
  std::array<double, 3> predicted_los_ned{};
  std::array<double, 3> desired_camera_axis_ned{};
  std::array<double, 3> current_camera_axis_ned{};
  double alignment_error_rad{};
  double alignment_weight{};
  bool tilt_limited{false};
  bool valid{false};
};

class ZAxisAlignment
{
public:
  explicit ZAxisAlignment(const ZAxisAlignmentConfig & config);

  ZAxisAlignmentResult update(
    const std::array<double, 3> & relative_position_ned,
    const std::array<double, 3> & relative_velocity_ned,
    const Quaternion & current_body_to_ned) const;

  static std::array<double, 3> body_minus_z_axis_ned(const Quaternion & body_to_ned);

private:
  ZAxisAlignmentConfig config_;
};

}  // namespace px4_offboard_tracking
