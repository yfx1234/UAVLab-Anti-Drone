#include "px4_offboard_tracking/tracking_quality_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

TrackingQualityGate::TrackingQualityGate(const TrackingQualityGateConfig & config)
: config_(config)
{
  if (config_.maximum_position_variance <= 0.0 ||
    config_.maximum_innovation_mahalanobis <= 0.0 ||
    config_.maximum_observation_age_s <= 0.0 || config_.maximum_estimate_age_s <= 0.0)
  {
    throw std::invalid_argument("invalid tracking quality gate configuration");
  }
}

TrackingQualityGateResult TrackingQualityGate::evaluate(
  const TrackingQualityGateInput & input) const
{
  TrackingQualityGateResult output;
  output.estimate_ok = input.estimate_valid;
  output.covariance_ok = std::isfinite(input.maximum_position_variance) &&
    input.maximum_position_variance >= 0.0 &&
    input.maximum_position_variance <= config_.maximum_position_variance;
  output.innovation_ok = std::isfinite(input.innovation_mahalanobis) &&
    input.innovation_mahalanobis >= 0.0 &&
    input.innovation_mahalanobis <= config_.maximum_innovation_mahalanobis;
  output.visibility_ok = input.observation_visible;
  output.timestamp_ok = std::isfinite(input.observation_age_s) &&
    std::isfinite(input.estimate_age_s) && input.observation_age_s >= 0.0 &&
    input.estimate_age_s >= 0.0 &&
    input.observation_age_s <= config_.maximum_observation_age_s &&
    input.estimate_age_s <= config_.maximum_estimate_age_s;
  output.attitude_ok = !input.attitude_saturated;
  output.alignment_ok = input.alignment_feasible;
  output.passed = output.estimate_ok && output.covariance_ok && output.innovation_ok &&
    output.visibility_ok && output.timestamp_ok && output.attitude_ok && output.alignment_ok;
  return output;
}

}  // namespace px4_offboard_tracking
