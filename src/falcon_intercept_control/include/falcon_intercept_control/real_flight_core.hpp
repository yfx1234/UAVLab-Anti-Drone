#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace falcon_intercept_control
{

using Vector3 = std::array<double, 3>;

struct Quaterniond
{
  double w{1.0};
  double x{};
  double y{};
  double z{};
};

struct CameraToFluTransform
{
  // Rotation maps body FLU vectors into optical vectors.
  Quaterniond q_body_flu_to_optical{0.5, 0.5, -0.5, 0.5};
  Vector3 camera_translation_body_flu_m{};
};

Vector3 optical_to_body_flu(const Vector3 & optical, const CameraToFluTransform & transform);
Vector3 flu_to_frd(const Vector3 & flu);
Vector3 rotate_body_frd_to_ned(const Quaterniond & body_to_ned, const Vector3 & frd);
double wrap_pi(double angle);

struct RelativeKinematicsResult
{
  double range_m{};
  double closing_speed_mps{};
  Vector3 los_rate_rad_s{};
  double los_rate_norm_rad_s{};
  double relative_speed_mps{};
  double time_to_closest_approach_s{};
  double predicted_miss_distance_m{};
  bool valid{false};
};

RelativeKinematicsResult relative_kinematics(const Vector3 & position, const Vector3 & velocity);

struct SearchConfig
{
  double yaw_rate_flu_rps{0.2};
  double settle_time_s{0.5};
  double timeout_s{120.0};
  double cue_gain{0.004};
  double maximum_cue_yaw_rate_rps{0.2};
};

struct SearchInput
{
  double now_s{};
  double yaw_ned_rad{};
  bool enabled{false};
  bool vehicle_ready{false};
  bool cue_valid{false};
  double cue_horizontal_error_px{};
};

struct SearchResult
{
  double yaw_rate_flu_rps{};
  double progress{};
  std::uint32_t completed_turns{};
  bool active{false};
  bool settling{false};
  bool target_cued{false};
  bool timed_out{false};
  bool valid{false};
  std::string reason{"disabled"};
};

class SearchController
{
public:
  explicit SearchController(const SearchConfig & config);
  SearchResult update(const SearchInput & input);
  void reset();

private:
  SearchConfig config_;
  bool started_{false};
  bool yaw_initialized_{false};
  double started_s_{};
  double last_yaw_ned_{};
  double accumulated_angle_rad_{};
  double settle_until_s_{};
  std::uint32_t completed_turns_{};
};

struct GuidanceConfig
{
  double position_gain_forward{0.25};
  double position_gain_lateral{0.30};
  double position_gain_vertical{0.25};
  double velocity_feedforward{0.35};
  double desired_distance_m{1.7};
  double maximum_forward_speed_mps{0.30};
  double maximum_lateral_speed_mps{0.15};
  double maximum_vertical_speed_mps{0.10};
  double maximum_yaw_rate_rps{0.20};
  double yaw_gain{1.2};
  double maximum_acceleration_mps2{0.30};
};

struct GuidanceResult
{
  Vector3 velocity_body_flu_mps{};
  double yaw_rate_flu_rps{};
  RelativeKinematicsResult kinematics{};
  bool valid{false};
};

class GuidanceController
{
public:
  explicit GuidanceController(const GuidanceConfig & config);
  GuidanceResult update(
    const Vector3 & position_body_flu_m, const Vector3 & velocity_body_flu_mps,
    double delta_time_s, bool translation_enabled);
  void reset();

private:
  GuidanceConfig config_;
  Vector3 previous_velocity_{};
};

struct QualityConfig
{
  double maximum_target_age_s{0.20};
  double maximum_position_variance_m2{4.0};
  double maximum_innovation_nis{16.27};
};

struct QualityInput
{
  bool calibration_ready{false};
  bool target_valid{false};
  double target_age_s{};
  double maximum_position_variance_m2{};
  double innovation_nis{};
  bool px4_ready{false};
};

struct QualityResult
{
  bool calibration_ok{false};
  bool target_fresh{false};
  bool covariance_ok{false};
  bool innovation_ok{false};
  bool px4_state_ok{false};
  bool passed{false};
  std::string reason{"not evaluated"};
};

QualityResult evaluate_quality(const QualityConfig & config, const QualityInput & input);

struct CaptureConfig
{
  double minimum_range_m{1.5};
  double maximum_range_m{2.0};
  double maximum_alignment_error_rad{0.034906585};
  double maximum_los_rate_rad_s{0.20};
  double maximum_relative_speed_mps{1.0};
  double maximum_predicted_miss_distance_m{0.5};
  double maximum_time_to_closest_approach_s{2.0};
  double required_hold_time_s{0.3};
  std::uint32_t minimum_distinct_frames{6};
};

struct CaptureInput
{
  double now_s{};
  double observation_stamp_s{};
  std::uint32_t observation_sequence{};
  bool observation_valid{false};
  bool quality_passed{false};
  double alignment_error_rad{};
  RelativeKinematicsResult kinematics{};
  // Conservative range estimate + k*sigma for the terminal gate. A zero
  // value keeps legacy/unit-test callers compatible.
  double range_upper_m{};
};

struct CaptureResult
{
  bool quality_ok{false};
  bool alignment_ok{false};
  bool range_ok{false};
  bool los_rate_ok{false};
  bool relative_speed_ok{false};
  bool prediction_ok{false};
  bool frame_count_ok{false};
  bool instantaneous_pass{false};
  bool held{false};
  std::uint32_t distinct_frames{};
  double hold_elapsed_s{};
};

class CaptureWindow
{
public:
  explicit CaptureWindow(const CaptureConfig & config);
  CaptureResult update(const CaptureInput & input);
  void reset();

private:
  void reset_hold();
  CaptureConfig config_;
  bool holding_{false};
  bool time_initialized_{false};
  bool observation_initialized_{false};
  double last_now_s_{};
  double hold_started_s_{};
  double last_observation_stamp_s_{};
  std::uint32_t last_observation_sequence_{};
  std::uint32_t distinct_frames_{};
};

double prediction_decay(double age_s, double fresh_s = 0.20, double stop_s = 0.50);

}  // namespace falcon_intercept_control
