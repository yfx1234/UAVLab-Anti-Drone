#pragma once

#include <array>

#include "px4_offboard_tracking/acceleration_guidance.hpp"

namespace px4_offboard_tracking
{

struct TerminalGuidanceConfig
{
  double navigation_constant{3.0};
  double minimum_closing_speed_mps{0.5};
  double line_of_sight_acceleration_mps2{0.6};
  double acceleration_feedforward_gain{0.5};
  double maximum_horizontal_acceleration_mps2{2.0};
  double maximum_vertical_acceleration_mps2{0.8};
};

struct TerminalGuidanceResult
{
  AccelerationCommand acceleration{};
  double closing_speed_mps{};
  double line_of_sight_rate_rad_s{};
  bool acceleration_limited{false};
  bool valid{false};
};

class TerminalGuidance
{
public:
  explicit TerminalGuidance(const TerminalGuidanceConfig & config);

  TerminalGuidanceResult update(
    const std::array<double, 3> & relative_position_ned,
    const std::array<double, 3> & relative_velocity_ned,
    const std::array<double, 3> & relative_acceleration_ned) const;

private:
  TerminalGuidanceConfig config_;
};

}  // namespace px4_offboard_tracking
