#pragma once

#include <cstdint>

namespace px4_offboard_tracking
{

enum class RangeControlStage : std::uint8_t
{
  SEARCH_HANDOVER = 0,
  DEPTH_ACQUIRE = 1,
  INTERCEPT = 2,
  SHOT = 3,
  EGRESS = 4
};

struct StagedRangePolicyConfig
{
  double acquisition_minimum_m{20.0};
  double acquisition_maximum_m{30.0};
  double intercept_minimum_m{17.0};
  double intercept_maximum_m{25.0};
  double shot_minimum_m{15.0};
  double shot_maximum_m{35.0};
  double soft_preferred_m{17.0};
  double boundary_gain_s_inv{0.45};
  double soft_preference_gain_s_inv{0.08};
  double reference_slew_mps{0.25};
  double maximum_closing_speed_mps{4.0};
};

struct StagedRangePolicyResult
{
  double desired_closing_speed_mps{};
  double active_reference_m{};
  double lower_bound_m{};
  double upper_bound_m{};
  bool approach_paused_for_fov{false};
  double approach_scale{1.0};
};

class StagedRangePolicy
{
public:
  explicit StagedRangePolicy(const StagedRangePolicyConfig & config = {});

  StagedRangePolicyResult update(
    RangeControlStage stage,
    double range_m,
    double dt_s,
    double fov_approach_scale);

  void reset();

private:
  StagedRangePolicyConfig config_;
  RangeControlStage stage_{RangeControlStage::SEARCH_HANDOVER};
  double reference_m_{};
  bool initialized_{false};
};

}  // namespace px4_offboard_tracking
