#pragma once

namespace px4_offboard_tracking
{

struct CaptureWindowConfig
{
  double maximum_pixel_error_px{60.0};
  double maximum_alignment_error_rad{0.12};
  double maximum_los_rate_rad_s{0.25};
  double maximum_predicted_miss_distance_m{8.5};
  double maximum_time_to_closest_approach_s{4.0};
  double required_hold_time_s{0.4};
  int minimum_distinct_observation_frames{8};
};

struct CaptureWindowInput
{
  double timestamp_s{};
  bool observation_valid{false};
  double observation_timestamp_s{};
  bool quality_gate_passed{false};
  bool attack_geometry_valid{false};
  double pixel_error_px{};
  double alignment_error_rad{};
  double los_rate_rad_s{};
  double predicted_miss_distance_m{};
  double time_to_closest_approach_s{};
};

struct CaptureWindowResult
{
  bool quality_ok{false};
  bool pixel_ok{false};
  bool alignment_ok{false};
  bool range_ok{false};
  bool los_rate_ok{false};
  bool relative_speed_ok{false};
  bool prediction_ok{false};
  bool frame_count_ok{false};
  bool instantaneous_pass{false};
  bool held{false};
  int distinct_observation_frames{};
  double hold_elapsed_s{};
};

class CaptureWindow
{
public:
  explicit CaptureWindow(const CaptureWindowConfig & config);

  CaptureWindowResult update(const CaptureWindowInput & input);
  void reset();

private:
  void reset_hold();

  CaptureWindowConfig config_;
  double hold_started_s_{};
  double last_timestamp_s_{};
  double last_observation_timestamp_s_{};
  int distinct_observation_frames_{};
  bool holding_{false};
  bool timestamp_initialized_{false};
  bool observation_timestamp_initialized_{false};
};

}  // namespace px4_offboard_tracking
