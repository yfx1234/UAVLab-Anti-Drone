#pragma once

namespace px4_offboard_tracking
{

struct CaptureRangePolicyConfig
{
  double minimum_range_m{15.0};
  double maximum_range_m{35.0};
  double preferred_range_m{25.0};
  double range_gain_s_inv{0.4};
  double pre_attack_maximum_closing_speed_mps{1.5};
  double attack_maximum_closing_speed_mps{4.0};
};

// Continuous mission-level radial speed schedule for the capture geometry.
//
// Positive speed closes on the target and negative speed opens the range. Before
// attack is enabled the controller settles at the preferred point inside the
// ideal window. Attack mode raises the permitted approach rate outside the
// window, but retains the same continuous standoff equilibrium instead of
// forcing a collision course.
class CaptureRangePolicy
{
public:
  explicit CaptureRangePolicy(const CaptureRangePolicyConfig & config);

  double desired_closing_speed(double range_m, bool attack_enabled) const;

  const CaptureRangePolicyConfig & config() const;

private:
  CaptureRangePolicyConfig config_;
};

}  // namespace px4_offboard_tracking
