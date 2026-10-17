#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

using NedVector3 = std::array<double, 3>;

inline bool finite_ned(const NedVector3 & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

inline NedVector3 to_task_ned(
  const NedVector3 & px4_local_position,
  const NedVector3 & propeller_start_origin)
{
  if (!finite_ned(px4_local_position) || !finite_ned(propeller_start_origin)) {
    throw std::invalid_argument("invalid task-frame position");
  }
  return {
    px4_local_position[0] - propeller_start_origin[0],
    px4_local_position[1] - propeller_start_origin[1],
    px4_local_position[2] - propeller_start_origin[2]};
}

inline double height_above_task_origin_m(
  const NedVector3 & px4_local_position,
  const NedVector3 & propeller_start_origin)
{
  return -to_task_ned(px4_local_position, propeller_start_origin)[2];
}

inline void compensate_px4_local_reset(
  NedVector3 & px4_local_reference,
  const NedVector3 & reset_delta)
{
  if (!finite_ned(px4_local_reference) || !finite_ned(reset_delta)) {
    throw std::invalid_argument("invalid PX4 local reset compensation");
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    px4_local_reference[axis] += reset_delta[axis];
  }
}

}  // namespace px4_offboard_tracking
