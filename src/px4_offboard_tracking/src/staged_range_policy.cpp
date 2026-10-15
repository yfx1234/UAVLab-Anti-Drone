#include "px4_offboard_tracking/staged_range_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

StagedRangePolicy::StagedRangePolicy(const StagedRangePolicyConfig & config)
: config_(config)
{
  if (config_.shot_minimum_m <= 0.0 ||
    config_.intercept_minimum_m < config_.shot_minimum_m ||
    config_.intercept_maximum_m <= config_.intercept_minimum_m ||
    config_.acquisition_minimum_m < config_.intercept_minimum_m ||
    config_.acquisition_maximum_m <= config_.acquisition_minimum_m ||
    config_.shot_maximum_m < config_.intercept_maximum_m ||
    config_.soft_preferred_m < config_.shot_minimum_m ||
    config_.soft_preferred_m > config_.intercept_maximum_m ||
    config_.boundary_gain_s_inv <= 0.0 ||
    config_.soft_preference_gain_s_inv < 0.0 ||
    config_.reference_slew_mps <= 0.0 ||
    config_.maximum_closing_speed_mps <= 0.0)
  {
    throw std::invalid_argument("invalid staged range policy configuration");
  }
}

StagedRangePolicyResult StagedRangePolicy::update(
  RangeControlStage stage,
  double range_m,
  double dt_s,
  double fov_approach_scale)
{
  StagedRangePolicyResult output;
  if (!std::isfinite(range_m) || !std::isfinite(dt_s) || dt_s < 0.0 ||
    !std::isfinite(fov_approach_scale))
  {
    return output;
  }
  output.approach_scale = std::clamp(fov_approach_scale, 0.0, 1.0);
  if (stage == RangeControlStage::SEARCH_HANDOVER ||
    stage == RangeControlStage::DEPTH_ACQUIRE)
  {
    output.lower_bound_m = config_.acquisition_minimum_m;
    output.upper_bound_m = config_.acquisition_maximum_m;
  } else if (stage == RangeControlStage::INTERCEPT) {
    output.lower_bound_m = config_.intercept_minimum_m;
    output.upper_bound_m = config_.intercept_maximum_m;
  } else {
    output.lower_bound_m = config_.shot_minimum_m;
    output.upper_bound_m = config_.shot_maximum_m;
  }

  if (!initialized_ || stage != stage_) {
    stage_ = stage;
    reference_m_ = std::clamp(
      range_m, output.lower_bound_m, output.upper_bound_m);
    initialized_ = true;
  }
  if (stage == RangeControlStage::INTERCEPT ||
    stage == RangeControlStage::SHOT)
  {
    const double maximum_step = config_.reference_slew_mps * dt_s;
    reference_m_ += std::clamp(
      config_.soft_preferred_m - reference_m_,
      -maximum_step, maximum_step);
  }
  reference_m_ = std::clamp(
    reference_m_, output.lower_bound_m, output.upper_bound_m);
  output.active_reference_m = reference_m_;

  if (range_m < output.lower_bound_m) {
    output.desired_closing_speed_mps = std::max(
      -config_.maximum_closing_speed_mps,
      config_.boundary_gain_s_inv * (range_m - output.lower_bound_m));
  } else if (range_m > output.upper_bound_m) {
    output.desired_closing_speed_mps = output.approach_scale * std::min(
      config_.maximum_closing_speed_mps,
      config_.boundary_gain_s_inv * (range_m - output.upper_bound_m));
  } else if (stage == RangeControlStage::INTERCEPT ||
    stage == RangeControlStage::SHOT)
  {
    output.desired_closing_speed_mps = output.approach_scale * std::clamp(
      config_.soft_preference_gain_s_inv * (range_m - reference_m_),
      -0.5, 0.5);
  }
  output.approach_paused_for_fov =
    output.approach_scale <= 1.0e-6 && range_m >= output.lower_bound_m;
  return output;
}

void StagedRangePolicy::reset()
{
  initialized_ = false;
}

}  // namespace px4_offboard_tracking
