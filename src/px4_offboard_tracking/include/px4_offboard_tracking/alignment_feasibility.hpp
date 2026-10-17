#pragma once

namespace px4_offboard_tracking
{

struct AlignmentFeasibilityConfig
{
  double maximum_tilt_rad{0.35};
  double minimum_thrust_margin{0.08};
  double maximum_body_rate_rad_s{2.0};
  double maximum_altitude_loss_m{1.0};
  double minimum_range_m{2.0};
  double maximum_alignment_time_s{8.0};
};

struct AlignmentFeasibilityInput
{
  double commanded_tilt_rad{};
  double commanded_thrust{};
  double maximum_available_thrust{1.0};
  double body_rate_rad_s{};
  double altitude_loss_m{};
  double range_m{};
  double alignment_duration_s{};
};

struct AlignmentFeasibilityResult
{
  bool tilt_ok{false};
  bool thrust_margin_ok{false};
  bool body_rate_ok{false};
  bool altitude_loss_ok{false};
  bool minimum_range_ok{false};
  bool alignment_time_ok{false};
  bool feasible{false};
  double thrust_margin{};
};

class AlignmentFeasibility
{
public:
  explicit AlignmentFeasibility(const AlignmentFeasibilityConfig & config);

  AlignmentFeasibilityResult evaluate(const AlignmentFeasibilityInput & input) const;

private:
  AlignmentFeasibilityConfig config_;
};

}  // namespace px4_offboard_tracking
