#pragma once

namespace px4_offboard_tracking
{

struct TrackingQualityGateConfig
{
  double maximum_position_variance{4.0};
  double maximum_innovation_mahalanobis{11.345};
  double maximum_observation_age_s{0.2};
  double maximum_estimate_age_s{0.2};
};

struct TrackingQualityGateInput
{
  bool estimate_valid{false};
  double maximum_position_variance{};
  double innovation_mahalanobis{};
  bool observation_visible{false};
  double observation_age_s{};
  double estimate_age_s{};
  bool attitude_saturated{false};
  bool alignment_feasible{false};
};

struct TrackingQualityGateResult
{
  bool estimate_ok{false};
  bool covariance_ok{false};
  bool innovation_ok{false};
  bool visibility_ok{false};
  bool timestamp_ok{false};
  bool attitude_ok{false};
  bool alignment_ok{false};
  bool passed{false};
};

class TrackingQualityGate
{
public:
  explicit TrackingQualityGate(const TrackingQualityGateConfig & config);

  TrackingQualityGateResult evaluate(const TrackingQualityGateInput & input) const;

private:
  TrackingQualityGateConfig config_;
};

}  // namespace px4_offboard_tracking
