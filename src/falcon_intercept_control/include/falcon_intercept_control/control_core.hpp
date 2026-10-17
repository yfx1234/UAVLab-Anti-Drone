#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace falcon_intercept_control
{

// Public state values intentionally match falcon_intercept_msgs/GuidanceCommand.
enum class MissionState : std::uint8_t
{
  INIT = 0,
  SEARCH = 1,
  TRACK = 2,
  APPROACH = 3,
  FIRE = 4,
  LAND = 5,
  DONE = 6,
  ABORT = 7,
};

struct Quaternion
{
  double w{1.0};
  double x{};
  double y{};
  double z{};
};

struct RigidTransform
{
  // Rotation maps body FRD vectors into the camera optical frame.
  Quaternion q_body_to_optical{};
  // Camera origin expressed in body FRD, in metres.
  std::array<double, 3> camera_translation_body_frd_m{};
};

struct RelativeKinematics
{
  double range_m{};
  double closing_speed_mps{};
  std::array<double, 3> los_rate_rad_s{};
  double time_to_closest_approach_s{};
  double predicted_miss_distance_m{};
  bool valid{false};
};

struct ControlConfig
{
  double target_timeout_s{0.5};
  double maximum_position_variance_m2{4.0};
  double minimum_confidence{0.25};
  double fire_distance_m{2.0};
  double minimum_safe_range_m{1.5};
  double center_threshold_px{80.0};
  int fire_hold_frames{3};
  int track_confirm_frames{2};
  double land_hold_s{1.0};
  double max_task_time_s{120.0};
  double position_gain_xy{0.4};
  double position_gain_z{0.3};
  double maximum_speed_xy_mps{0.8};
  double maximum_speed_z_mps{0.5};
  double search_yaw_gain{0.004};
  double maximum_search_yaw_rate_rps{0.35};
  double scan_yaw_rate_rps{0.20};
  // Optional one-shot mission-start sweep adapted from anti-drone f0171ad0b0.
  // Disabled by default until SITL/bench validation is complete.
  bool startup_search_enabled{false};
  double startup_search_yaw_rate_rps{0.20};
  double startup_search_turns{1.0};
  double startup_search_settle_time_s{0.5};
};

struct ControlInput
{
  double now_s{};
  bool mission_enabled{false};
  bool abort_requested{false};
  bool landed{false};
  bool target_valid{false};
  std::array<double, 3> target_position_body_frd_m{};
  std::array<double, 3> target_velocity_body_frd_mps{};
  double maximum_position_variance_m2{};
  double confidence{};
  double target_age_s{};
  bool health_valid{false};
  bool left_centered{false};
  bool right_centered{false};
  bool search_cue_valid{false};
  double search_horizontal_error_px{};
};

struct ControlOutput
{
  MissionState state{MissionState::INIT};
  std::array<double, 3> velocity_body_frd_mps{};
  double yaw_rate_rps{};
  bool valid{false};
  bool land_request{false};
  bool fire_event{false};
  std::string reason{"initialized"};
  RelativeKinematics kinematics{};
};

std::array<double, 3> optical_position_to_body_frd(
  const std::array<double, 3> & position_optical_m,
  const RigidTransform & transform);

std::array<double, 9> optical_covariance_to_body_frd(
  const std::array<double, 9> & covariance_optical,
  const RigidTransform & transform);

RelativeKinematics compute_relative_kinematics(
  const std::array<double, 3> & relative_position_m,
  const std::array<double, 3> & relative_velocity_mps);

class ControlCore
{
public:
  explicit ControlCore(const ControlConfig & config);
  ControlOutput update(const ControlInput & input);
  void reset();
  MissionState state() const {return state_;}
  static const char * state_name(MissionState state);

private:
  void transition(MissionState next, const char * reason, double now_s);
  bool quality_passes(const ControlInput & input) const;
  std::array<double, 3> approach_velocity(const ControlInput & input) const;
  bool startup_search_pending(double now_s);
  double startup_search_yaw_rate(double now_s) const;

  ControlConfig config_;
  MissionState state_{MissionState::INIT};
  double state_entered_s_{};
  double mission_started_s_{};
  bool mission_time_initialized_{false};
  int track_frames_{};
  int fire_frames_{};
  bool fire_sent_{false};
  bool startup_search_started_{false};
  bool startup_search_completed_{false};
  double startup_search_started_s_{};
  std::string reason_{"initialized"};
};

}  // namespace falcon_intercept_control
