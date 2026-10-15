#include "px4_offboard_tracking/capture_range_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

CaptureRangePolicy::CaptureRangePolicy(const CaptureRangePolicyConfig & config)
: config_(config)
{
  if (config_.minimum_range_m <= 0.0 || config_.maximum_range_m <= config_.minimum_range_m ||
    config_.preferred_range_m <= config_.minimum_range_m ||
    config_.preferred_range_m >= config_.maximum_range_m ||
    config_.range_gain_s_inv <= 0.0 ||
    config_.pre_attack_maximum_closing_speed_mps <= 0.0 ||
    config_.attack_maximum_closing_speed_mps <
    config_.pre_attack_maximum_closing_speed_mps)
  {
    throw std::invalid_argument("invalid capture range policy configuration");
  }
}

double CaptureRangePolicy::desired_closing_speed(
  double range_m, bool attack_enabled) const
{
  if (!std::isfinite(range_m)) {
    return 0.0;
  }

  const double maximum_speed = attack_enabled ?
    config_.attack_maximum_closing_speed_mps :
    config_.pre_attack_maximum_closing_speed_mps;
  return std::clamp(
    config_.range_gain_s_inv * (range_m - config_.preferred_range_m),
    -maximum_speed, maximum_speed);
}

const CaptureRangePolicyConfig & CaptureRangePolicy::config() const
{
  return config_;
}

}  // namespace px4_offboard_tracking
