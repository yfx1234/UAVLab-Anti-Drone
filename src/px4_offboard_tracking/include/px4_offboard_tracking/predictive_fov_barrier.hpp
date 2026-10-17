#pragma once

#include <cstdint>

namespace px4_offboard_tracking
{

enum class FovBarrierZone : std::uint8_t
{
  NORMAL = 0,
  CENTERING = 1,
  RECOVERY = 2,
  IMM_COAST = 3,
  LOST = 4
};

struct PredictiveFovBarrierConfig
{
  double normal_boundary_rad{0.08726646259971647};      // 5 deg
  double centering_boundary_rad{0.13962634015954636};   // 8 deg
  double hard_boundary_rad{0.17453292519943295};        // 10 deg
  double recovery_exit_rad{0.10471975511965977};         // 6 deg
  double prediction_horizon_s{0.5};
  double angular_rate_filter_time_constant_s{0.12};
};

struct PredictiveFovBarrierInput
{
  double now_s{};
  bool depth_visible{false};
  bool imm_prediction_usable{false};
  double horizontal_error_rad{};
  double vertical_error_rad{};
  std::uint32_t observation_sequence{};
};

struct PredictiveFovBarrierResult
{
  FovBarrierZone zone{FovBarrierZone::LOST};
  double horizontal_rate_rad_s{};
  double vertical_rate_rad_s{};
  double predicted_horizontal_error_rad{};
  double predicted_vertical_error_rad{};
  double predicted_maximum_error_rad{};
  double centering_strength{};
  bool pause_approach{true};
  bool depth_admission_allowed{false};
  bool recovery_exit_allowed{false};
};

// A component-wise camera-FOV barrier. The measured image angles already
// contain both LOS motion and the actual Body -Z response, so extrapolating
// their filtered rates predicts the future actual camera/LOS error rather
// than an ideal commanded attack axis.
class PredictiveFovBarrier
{
public:
  explicit PredictiveFovBarrier(
    const PredictiveFovBarrierConfig & config = {});

  PredictiveFovBarrierResult update(
    const PredictiveFovBarrierInput & input);

  void reset();
  const PredictiveFovBarrierConfig & config() const;

private:
  PredictiveFovBarrierConfig config_;
  FovBarrierZone zone_{FovBarrierZone::LOST};
  double last_time_s_{};
  double last_horizontal_error_rad_{};
  double last_vertical_error_rad_{};
  double horizontal_rate_rad_s_{};
  double vertical_rate_rad_s_{};
  std::uint32_t last_sequence_{};
  bool initialized_{false};
};

}  // namespace px4_offboard_tracking
