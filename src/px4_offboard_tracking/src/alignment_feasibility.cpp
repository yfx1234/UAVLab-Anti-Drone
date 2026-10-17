#include "px4_offboard_tracking/alignment_feasibility.hpp"

#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

AlignmentFeasibility::AlignmentFeasibility(const AlignmentFeasibilityConfig & config)
: config_(config)
{
  if (config_.maximum_tilt_rad <= 0.0 || config_.maximum_tilt_rad >= 1.4 ||
    config_.minimum_thrust_margin < 0.0 || config_.minimum_thrust_margin >= 1.0 ||
    config_.maximum_body_rate_rad_s <= 0.0 || config_.maximum_altitude_loss_m < 0.0 ||
    config_.minimum_range_m <= 0.0 || config_.maximum_alignment_time_s <= 0.0)
  {
    throw std::invalid_argument("invalid alignment feasibility configuration");
  }
}

AlignmentFeasibilityResult AlignmentFeasibility::evaluate(
  const AlignmentFeasibilityInput & input) const
{
  AlignmentFeasibilityResult output;
  const bool finite = std::isfinite(input.commanded_tilt_rad) &&
    std::isfinite(input.commanded_thrust) &&
    std::isfinite(input.maximum_available_thrust) &&
    std::isfinite(input.body_rate_rad_s) && std::isfinite(input.altitude_loss_m) &&
    std::isfinite(input.range_m) && std::isfinite(input.alignment_duration_s);
  if (!finite || input.maximum_available_thrust <= 0.0 || input.commanded_thrust < 0.0 ||
    input.body_rate_rad_s < 0.0 || input.altitude_loss_m < 0.0 || input.range_m < 0.0 ||
    input.alignment_duration_s < 0.0)
  {
    return output;
  }

  output.thrust_margin = input.maximum_available_thrust - input.commanded_thrust;
  output.tilt_ok = input.commanded_tilt_rad <= config_.maximum_tilt_rad;
  output.thrust_margin_ok = output.thrust_margin >= config_.minimum_thrust_margin;
  output.body_rate_ok = input.body_rate_rad_s <= config_.maximum_body_rate_rad_s;
  output.altitude_loss_ok = input.altitude_loss_m <= config_.maximum_altitude_loss_m;
  output.minimum_range_ok = input.range_m >= config_.minimum_range_m;
  output.alignment_time_ok = input.alignment_duration_s <= config_.maximum_alignment_time_s;
  output.feasible = output.tilt_ok && output.thrust_margin_ok && output.body_rate_ok &&
    output.altitude_loss_ok && output.minimum_range_ok && output.alignment_time_ok;
  return output;
}

}  // namespace px4_offboard_tracking
