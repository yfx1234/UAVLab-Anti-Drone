#pragma once

namespace px4_offboard_tracking
{

struct ShotTimingConfig
{
  double preferred_axis_error_rad{0.017453292519943295};
  double hard_axis_error_rad{0.03490658503988659};
  double prediction_horizon_s{0.04};
  double error_rate_filter_time_constant_s{0.08};
  double maximum_absolute_error_rate_rad_s{0.5};
  double minimum_ready_hold_s{0.04};
};

struct ShotTimingInput
{
  bool base_gate_passed{false};
  double axis_error_rad{};
  double now_s{};
};

struct ShotTimingResult
{
  bool valid{false};
  bool ready{false};
  double axis_error_rad{};
  double axis_error_rate_rad_s{};
  double predicted_axis_error_rad{};
  double score{};
};

// Selects a stable firing instant inside the preferred 1 degree window while
// retaining the existing 2 degree absolute safety gate.  This class never
// consumes target truth; its input is the controller's camera/IMM-derived
// predicted shot direction and the pursuer's measured attitude.
class ShotTimingOptimizer
{
public:
  explicit ShotTimingOptimizer(const ShotTimingConfig & config);

  ShotTimingResult update(const ShotTimingInput & input);
  void reset();

private:
  ShotTimingConfig config_;
  bool initialized_{false};
  double previous_time_s_{};
  double previous_error_rad_{};
  double filtered_error_rate_rad_s_{};
  double ready_since_s_{-1.0};
};

}  // namespace px4_offboard_tracking
