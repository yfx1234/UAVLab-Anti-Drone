#include <anti_drone_interfaces/msg/depth_observation.hpp>
#include <anti_drone_interfaces/msg/search_observation.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <virtual_interception_interfaces/msg/virtual_shot_event.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "px4_offboard_tracking/geometric_control.hpp"
#include "px4_offboard_tracking/bearing_handover.hpp"
#include "px4_offboard_tracking/common_frame_transform.hpp"
#include "px4_offboard_tracking/horizontal_speed_limiter.hpp"
#include "px4_offboard_tracking/launch_position_planner.hpp"
#include "px4_offboard_tracking/maneuver_feedforward.hpp"
#include "px4_offboard_tracking/vertical_speed_limiter.hpp"
#include "px4_offboard_tracking/minimum_height_constraint.hpp"
#include "px4_offboard_tracking/msg/pure_interceptor_status.hpp"
#include "px4_offboard_tracking/predictive_fov_barrier.hpp"
#include "px4_offboard_tracking/msg/relative_estimate.hpp"
#include "px4_offboard_tracking/predictive_shot.hpp"
#include "px4_offboard_tracking/pure_mission.hpp"
#include "px4_offboard_tracking/sample_time_sync.hpp"
#include "px4_offboard_tracking/shot_timing_optimizer.hpp"
#include "px4_offboard_tracking/staged_range_policy.hpp"
#include "px4_offboard_tracking/startup_search.hpp"
#include "px4_offboard_tracking/task_frame.hpp"
#include "px4_offboard_tracking/unified_3d_guidance.hpp"

namespace px4_offboard_tracking
{
namespace
{

using Vector = std::array<double, 3>;

enum class RadialControlSource : std::uint8_t
{
  FOV_HOLD = 0,
  SHOT_HOLD = 1,
  LAUNCH_PLANNER = 2,
  APPROACH_17M = 3,
  RECOIL = 4
};

const char * radial_control_source_name(RadialControlSource source)
{
  switch (source) {
    case RadialControlSource::FOV_HOLD: return "FOV_HOLD";
    case RadialControlSource::SHOT_HOLD: return "SHOT_HOLD";
    case RadialControlSource::LAUNCH_PLANNER: return "LAUNCH_PLANNER";
    case RadialControlSource::APPROACH_17M: return "APPROACH_17M";
    case RadialControlSource::RECOIL: return "RECOIL";
  }
  return "UNKNOWN";
}

double dot(const Vector & a, const Vector & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const Vector & value)
{
  return std::sqrt(dot(value, value));
}

Vector normalized(const Vector & value)
{
  const double length = norm(value);
  if (!std::isfinite(length) || length < 1e-9) {
    return {};
  }
  return {value[0] / length, value[1] / length, value[2] / length};
}

Vector add(const Vector & a, const Vector & b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vector scale(const Vector & value, double factor)
{
  return {factor * value[0], factor * value[1], factor * value[2]};
}

double angle_between(const Vector & a, const Vector & b)
{
  return std::acos(std::clamp(dot(normalized(a), normalized(b)), -1.0, 1.0));
}

Vector slerp_unit(const Vector & from, const Vector & to, double fraction)
{
  const Vector start = normalized(from);
  const Vector end = normalized(to);
  const double cosine = std::clamp(dot(start, end), -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (angle < 1e-8) {
    return start;
  }
  const double sine = std::sin(angle);
  if (std::abs(sine) < 1e-8) {
    return normalized(add(scale(start, 1.0 - fraction), scale(end, fraction)));
  }
  return normalized(add(
    scale(start, std::sin((1.0 - fraction) * angle) / sine),
    scale(end, std::sin(fraction * angle) / sine)));
}

Quaternion normalized_quaternion(const Quaternion & value)
{
  const double length = std::sqrt(
    value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
  if (!std::isfinite(length) || length < 1e-9) {
    return {};
  }
  return {
    value.w / length, value.x / length, value.y / length, value.z / length};
}

Vector rotate_body_to_ned(const Quaternion & input, const Vector & body)
{
  const Quaternion q = normalized_quaternion(input);
  const Vector qv{q.x, q.y, q.z};
  const Vector twice_cross{
    2.0 * (qv[1] * body[2] - qv[2] * body[1]),
    2.0 * (qv[2] * body[0] - qv[0] * body[2]),
    2.0 * (qv[0] * body[1] - qv[1] * body[0])};
  const Vector second_cross{
    qv[1] * twice_cross[2] - qv[2] * twice_cross[1],
    qv[2] * twice_cross[0] - qv[0] * twice_cross[2],
    qv[0] * twice_cross[1] - qv[1] * twice_cross[0]};
  return {
    body[0] + q.w * twice_cross[0] + second_cross[0],
    body[1] + q.w * twice_cross[1] + second_cross[1],
    body[2] + q.w * twice_cross[2] + second_cross[2]};
}

bool finite_vector(const Vector & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

}  // namespace

// Production controller boundary.
//
// Allowed external inputs:
//   1. SearchObservation from the real/search perception pipeline.
//   2. DepthObservation from the real/stereo perception pipeline.
//   3. RelativeEstimate derived from DepthObservation.
//   4. This vehicle's PX4 state.
//
// Simulation-only sources are intentionally absent from this translation unit.
class PureInterceptorController : public rclcpp::Node
{
public:
  PureInterceptorController()
  : Node("pure_interceptor_controller"),
    topic_prefix_(declare_parameter<std::string>("topic_prefix", "/px4_1/fmu")),
    search_topic_(declare_parameter<std::string>(
      "search_topic", "/perception/search/observation")),
    depth_topic_(declare_parameter<std::string>(
      "depth_topic", "/perception/depth/observation")),
    estimate_topic_(declare_parameter<std::string>(
      "estimate_topic", "/tracking/relative/estimate")),
    status_topic_(declare_parameter<std::string>(
      "status_topic", "/tracking/pure_interceptor/status")),
    shot_topic_(declare_parameter<std::string>(
      "shot_topic", "/tracking/virtual_shot_event")),
    completion_topic_(declare_parameter<std::string>(
      "completion_topic", "/tracking/mission/complete")),
    offboard_session_started_topic_(declare_parameter<std::string>(
      "offboard_session_started_topic", "/tracking/offboard_session_started")),
    enable_topic_(declare_parameter<std::string>(
      "enable_topic", "/tracking/pure_interceptor/enable")),
    task_origin_topic_(declare_parameter<std::string>(
      "task_origin_topic", "/tracking/task_origin_ned")),
    common_origin_topic_(declare_parameter<std::string>(
      "common_origin_topic", "/tracking/common_origin")),
    frame_id_(declare_parameter<std::string>("common_frame_id", "map_ned")),
    target_system_(declare_parameter<int>("target_system", 2)),
    control_enabled_(declare_parameter<bool>("control_enabled", true)),
    auto_engage_(declare_parameter<bool>("auto_engage", true)),
    takeoff_enabled_(declare_parameter<bool>("takeoff_enabled", true)),
    auto_disable_on_offboard_exit_(
      declare_parameter<bool>("auto_disable_on_offboard_exit", false)),
    offboard_rate_hz_(declare_parameter<double>("offboard_rate_hz", 50.0)),
    offboard_warmup_s_(declare_parameter<double>("offboard_warmup_s", 1.0)),
    preflight_stability_hold_s_(
      declare_parameter<double>("preflight_stability_hold_s", 5.0)),
    state_timeout_s_(declare_parameter<double>("state_timeout_s", 0.5)),
    status_timeout_s_(declare_parameter<double>("status_timeout_s", 1.0)),
    observation_timeout_s_(declare_parameter<double>("observation_timeout_s", 0.25)),
    estimate_timeout_s_(declare_parameter<double>("estimate_timeout_s", 0.5)),
    prediction_coast_timeout_s_(
      declare_parameter<double>("prediction_coast_timeout_s", 1.0)),
    prediction_maximum_position_variance_(
      declare_parameter<double>("prediction_maximum_position_variance", 25.0)),
    attitude_history_s_(declare_parameter<double>("attitude_history_s", 2.0)),
    attitude_sync_tolerance_s_(
      declare_parameter<double>("attitude_sync_tolerance_s", 0.05)),
    attitude_history_(attitude_history_s_, attitude_sync_tolerance_s_),
    takeoff_height_m_(declare_parameter<double>("takeoff_height_m", 5.0)),
    takeoff_tolerance_m_(declare_parameter<double>("takeoff_tolerance_m", 0.6)),
    takeoff_speed_tolerance_mps_(
      declare_parameter<double>("takeoff_speed_tolerance_mps", 0.5)),
    search_yaw_rate_rad_s_(declare_parameter<double>("search_yaw_rate_rad_s", 0.35)),
    search_prediction_hold_s_(
      declare_parameter<double>("search_prediction_hold_s", 1.5)),
    known_target_direction_enabled_(
      declare_parameter<bool>("known_target_direction_enabled", false)),
    known_target_bearing_ned_rad_(
      declare_parameter<double>("known_target_bearing_ned_rad", 0.0)),
    known_target_search_hold_s_(
      declare_parameter<double>("known_target_search_hold_s", 6.0)),
    known_target_search_acceleration_mps2_(
      declare_parameter<double>("known_target_search_acceleration_mps2", 1.5)),
    known_target_search_acceleration_duration_s_(
      declare_parameter<double>("known_target_search_acceleration_duration_s", 1.5)),
    known_target_course_enabled_(
      declare_parameter<bool>("known_target_course_enabled", false)),
    known_target_course_ned_rad_(
      declare_parameter<double>("known_target_course_ned_rad", 0.0)),
    known_target_search_velocity_mps_(
      declare_parameter<double>("known_target_search_velocity_mps", 0.0)),
    known_target_velocity_gain_(
      declare_parameter<double>("known_target_velocity_gain", 1.0)),
    known_target_course_minimum_speed_fraction_(
      declare_parameter<double>(
        "known_target_course_minimum_speed_fraction", 0.35)),
    search_pattern_(StartupSearchConfig{
      declare_parameter<bool>("search_pattern_enabled", true),
      search_yaw_rate_rad_s_,
      declare_parameter<double>("search_pattern_turns", 1.0),
      0.0,
      false,
      declare_parameter<bool>("search_repeat_until_acquired", false),
      0.0}),
    handover_maximum_tilt_rad_(
      declare_parameter<double>("handover_maximum_tilt_rad", 0.52)),
    handover_maximum_acceleration_mps2_(
      declare_parameter<double>("handover_maximum_acceleration_mps2", 2.5)),
    maximum_jerk_mps3_(declare_parameter<double>("maximum_jerk_mps3", 8.0)),
    invalid_input_braking_gain_(
      declare_parameter<double>("invalid_input_braking_gain", 1.2)),
    maximum_braking_acceleration_mps2_(
      declare_parameter<double>("maximum_braking_acceleration_mps2", 3.0)),
    search_coast_timeout_s_(
      declare_parameter<double>("search_coast_timeout_s", 30.0)),
    search_velocity_hold_s_(
      declare_parameter<double>("search_velocity_hold_s", 5.0)),
    search_velocity_decay_time_constant_s_(
      declare_parameter<double>("search_velocity_decay_time_constant_s", 10.0)),
    search_velocity_gain_(
      declare_parameter<double>("search_velocity_gain", 1.0)),
    attack_enable_delay_s_(declare_parameter<double>("attack_enable_delay_s", 10.0)),
    first_attack_tracking_hold_s_(
      declare_parameter<double>("first_attack_tracking_hold_s", 3.0)),
    first_attack_depth_gap_tolerance_s_(
      declare_parameter<double>("first_attack_depth_gap_tolerance_s", 1.0)),
    terminal_alignment_tracking_hold_s_(
      declare_parameter<double>("terminal_alignment_tracking_hold_s", 2.0)),
    terminal_alignment_ramp_s_(
      declare_parameter<double>("terminal_alignment_ramp_s", 1.0)),
    capture_minimum_range_m_(
      declare_parameter<double>("capture_minimum_range_m", 15.0)),
    capture_maximum_range_m_(
      declare_parameter<double>("capture_maximum_range_m", 35.0)),
    capture_maximum_alignment_rad_(
      declare_parameter<double>("capture_maximum_alignment_rad", 0.03490658503988659)),
    // 网枪链（方案 v4 §1）：置 false 时本节点不再发布 VirtualShotEvent
    // （开火决定权移交 netgun_intercept 的 netgun_strike_node），制导/
    // 瞄准/任务完成逻辑不变；默认 true 保持旧行为。
    enable_native_fire_(
      declare_parameter<bool>("enable_native_fire", true)),
    projectile_speed_mps_(
      declare_parameter<double>("projectile_speed_mps", 400.0)),
    shot_maximum_time_of_flight_s_(
      declare_parameter<double>("shot_maximum_time_of_flight_s", 0.25)),
    shot_maximum_lead_angle_rad_(
      declare_parameter<double>("shot_maximum_lead_angle_rad", 0.06981317007977318)),
    shot_aim_tolerance_rad_(
      declare_parameter<double>("shot_aim_tolerance_rad", 0.017453292519943295)),
    shot_maximum_lateral_sigma_m_(
      declare_parameter<double>("shot_maximum_lateral_sigma_m", 0.75)),
    shot_hold_stationary_relative_speed_mps_(
      declare_parameter<double>("shot_hold_stationary_relative_speed_mps", 0.75)),
    recoil_opening_speed_mps_(
      declare_parameter<double>("recoil_opening_speed_mps", 1.0)),
    // Retained as declared, ignored compatibility parameters so existing
    // launch/config files do not fail while migrating to the time-bounded
    // recoil interface.
    egress_distance_m_(declare_parameter<double>("egress_distance_m", 5.0)),
    egress_speed_mps_(declare_parameter<double>("egress_speed_mps", 1.0)),
    safety_minimum_depth_m_(declare_parameter<double>("safety_minimum_depth_m", 3.0)),
    minimum_height_constraint_(MinimumHeightConstraintConfig{
      declare_parameter<double>("minimum_flight_height_m", 3.0),
      declare_parameter<double>(
        "minimum_height_prediction_horizon_s", 1.0),
      declare_parameter<double>(
        "minimum_height_maximum_upward_acceleration_mps2", 3.0),
      declare_parameter<double>("minimum_height_recovery_m", 3.3),
      declare_parameter<double>("minimum_height_recovery_gain_s2", 1.0),
      declare_parameter<double>("minimum_height_recovery_damping_s_inv", 1.0)}),
    safety_maximum_speed_mps_(
      declare_parameter<double>("safety_maximum_speed_mps", 14.0)),
    safety_maximum_tilt_rad_(
      declare_parameter<double>("safety_maximum_tilt_rad", 0.75)),
    recovery_maximum_speed_mps_(
      declare_parameter<double>("recovery_maximum_speed_mps", 0.8)),
    recovery_maximum_tilt_rad_(
      declare_parameter<double>("recovery_maximum_tilt_rad", 0.18)),
    hover_thrust_(declare_parameter<double>("hover_thrust", 0.42)),
    maximum_thrust_(declare_parameter<double>("maximum_thrust", 0.70)),
    maximum_tilt_rad_(declare_parameter<double>("maximum_tilt_rad", 0.65)),
    launch_position_planner_enabled_(
      declare_parameter<bool>("launch_position_planner_enabled", true)),
    launch_position_planner_blend_(
      declare_parameter<double>("launch_position_planner_blend", 1.0)),
    launch_planner_minimum_geometry_blend_(
      declare_parameter<double>("launch_planner_minimum_geometry_blend", 0.55)),
    launch_planner_full_inbound_speed_mps_(
      declare_parameter<double>("launch_planner_full_inbound_speed_mps", 4.0)),
    geometric_control_(
      hover_thrust_, maximum_thrust_, maximum_tilt_rad_,
      declare_parameter<double>("attitude_gain", 3.0),
      declare_parameter<double>("maximum_body_rate_rad_s", 3.0)),
    command_speed_limiter_(HorizontalSpeedLimiterConfig{
      declare_parameter<double>("command_maximum_speed_mps", 12.0),
      declare_parameter<double>("command_speed_soft_zone_mps", 2.5),
      declare_parameter<double>("command_overspeed_braking_gain", 4.0),
      declare_parameter<double>("command_maximum_braking_acceleration_mps2", 5.0)}),
    vertical_speed_limiter_(VerticalSpeedLimiterConfig{
      declare_parameter<double>("command_maximum_vertical_speed_mps", 1.0),
      declare_parameter<double>("command_vertical_speed_soft_zone_mps", 0.2),
      declare_parameter<double>("command_vertical_overspeed_braking_gain", 2.0),
      declare_parameter<double>(
        "command_vertical_maximum_braking_acceleration_mps2", 2.0)}),
    bearing_handover_(BearingHandoverConfig{
      declare_parameter<double>("handover_maximum_axis_rate_rad_s", 0.8),
      declare_parameter<double>("handover_los_rate_filter_time_constant_s", 0.15),
      declare_parameter<double>("handover_los_rate_lead_s", 0.10),
      observation_timeout_s_}),
    fov_barrier_(PredictiveFovBarrierConfig{
      declare_parameter<double>("fov_normal_boundary_rad", 0.08726646259971647),
      declare_parameter<double>("fov_centering_boundary_rad", 0.13962634015954636),
      declare_parameter<double>("fov_hard_boundary_rad", 0.17453292519943295),
      declare_parameter<double>("fov_recovery_exit_rad", 0.10471975511965977),
      declare_parameter<double>("fov_prediction_horizon_s", 0.2),
      declare_parameter<double>("fov_rate_filter_time_constant_s", 0.12)}),
    staged_range_policy_(StagedRangePolicyConfig{
      declare_parameter<double>("acquisition_range_minimum_m", 20.0),
      declare_parameter<double>("acquisition_range_maximum_m", 30.0),
      declare_parameter<double>("intercept_range_minimum_m", 17.0),
      declare_parameter<double>("intercept_range_maximum_m", 25.0),
      capture_minimum_range_m_,
      capture_maximum_range_m_,
      declare_parameter<double>("preferred_capture_range_m", 17.0),
      declare_parameter<double>("range_boundary_gain_s_inv", 0.45),
      declare_parameter<double>("range_soft_preference_gain_s_inv", 0.08),
      declare_parameter<double>("range_reference_slew_mps", 0.25),
      declare_parameter<double>("capture_maximum_approach_speed_mps", 4.0)}),
    maneuver_feedforward_(ManeuverFeedforwardConfig{
      declare_parameter<double>(
        "target_acceleration_feedforward_minimum_ca_probability", 0.25),
      declare_parameter<double>(
        "target_acceleration_feedforward_full_ca_probability", 0.75),
      declare_parameter<double>(
        "target_acceleration_feedforward_maximum_stddev_mps2", 5.0),
      declare_parameter<double>(
        "target_acceleration_feedforward_filter_time_constant_s", 0.20),
      declare_parameter<double>(
        "target_acceleration_feedforward_maximum_mps2", 4.0)}),
    launch_position_planner_(LaunchPositionPlannerConfig{
      declare_parameter<double>("launch_planner_minimum_horizon_s", 0.5),
      declare_parameter<double>("launch_planner_maximum_horizon_s", 8.0),
      declare_parameter<double>("launch_planner_horizon_step_s", 0.1),
      capture_minimum_range_m_,
      capture_maximum_range_m_,
      declare_parameter<double>("launch_planner_preferred_range_m", 25.0),
      declare_parameter<double>("launch_planner_range_step_m", 1.0),
      declare_parameter<double>(
        "launch_planner_maximum_horizontal_acceleration_mps2", 2.5),
      declare_parameter<double>(
        "launch_planner_maximum_vertical_acceleration_mps2", 1.0),
      declare_parameter<double>("launch_planner_maximum_speed_mps", 12.0),
      maximum_tilt_rad_,
      capture_maximum_alignment_rad_,
      declare_parameter<double>(
        "launch_planner_maximum_target_acceleration_mps2", 4.0),
      declare_parameter<double>("launch_planner_time_weight", 0.05),
      declare_parameter<double>("launch_planner_acceleration_weight", 1.0),
      declare_parameter<double>(
        "launch_planner_acceleration_change_weight", 0.2),
      declare_parameter<double>("launch_planner_range_weight", 0.01),
      declare_parameter<double>(
        "launch_planner_terminal_relative_speed_weight", 0.0),
      declare_parameter<double>(
        "launch_planner_terminal_relative_speed_scale_mps", 4.0),
      declare_parameter<double>("launch_planner_closing_speed_reserve_mps", 4.0),
      declare_parameter<double>("launch_planner_excess_speed_weight", 0.2),
      declare_parameter<double>("launch_planner_excess_speed_scale_mps", 2.0)}),
    guidance_(Unified3dGuidanceConfig{
      declare_parameter<double>("navigation_constant", 3.5),
      declare_parameter<double>("radial_acceleration_mps2", 1.5),
      declare_parameter<double>("radial_velocity_gain", 0.35),
      declare_parameter<double>("desired_closing_speed_mps", 4.0),
      declare_parameter<double>("target_acceleration_feedforward_gain", 0.2),
      declare_parameter<double>("maximum_acceleration_mps2", 5.0),
      maximum_tilt_rad_,
      declare_parameter<double>("depth_fov_constraint_rad", 0.13962634015954636),
      declare_parameter<double>("terminal_alignment_start_range_m", 45.0),
      declare_parameter<double>("capture_alignment_range_m", 35.0),
      declare_parameter<double>("capture_alignment_target_rad", 0.017453292519943295),
      declare_parameter<double>("transverse_velocity_gain", 1.0)}),
    shot_solver_(PredictiveShotConfig{
      projectile_speed_mps_,
      9.80665,
      0.001,
      shot_maximum_time_of_flight_s_,
      declare_parameter<double>("shot_maximum_target_acceleration_mps2", 20.0),
      1e-7,
      12}),
    shot_timing_optimizer_(ShotTimingConfig{
      shot_aim_tolerance_rad_,
      capture_maximum_alignment_rad_,
      declare_parameter<double>("shot_timing_prediction_horizon_s", 0.04),
      declare_parameter<double>(
        "shot_timing_error_rate_filter_time_constant_s", 0.08),
      declare_parameter<double>(
        "shot_timing_maximum_absolute_error_rate_rad_s", 0.5),
      declare_parameter<double>("shot_timing_minimum_ready_hold_s", 0.04)}),
    mission_(PureMissionConfig{
      declare_parameter<double>("search_acquisition_hold_s", 0.2),
      declare_parameter<double>("depth_acquisition_hold_s", 0.2),
      declare_parameter<double>("camera_loss_hysteresis_s", 0.6),
      declare_parameter<double>("depth_loss_hysteresis_s", 0.2),
      static_cast<std::uint32_t>(
        declare_parameter<int>("minimum_depth_acquisition_frames", 4)),
      declare_parameter<double>("estimate_stability_hold_s", 1.0),
      0.0,
      declare_parameter<double>("fov_recovery_entry_hold_s", 0.15),
      declare_parameter<double>("tracking_invalid_hysteresis_s", 0.2),
      declare_parameter<double>("fov_recovery_exit_hold_s", 0.5),
      declare_parameter<double>("recoil_duration_s", 0.2),
      declare_parameter<double>("reattack_cooldown_s", 3.0),
      static_cast<std::uint32_t>(declare_parameter<int>("required_captures", 2))}),
    completion_braking_hold_s_(
      declare_parameter<double>("completion_braking_hold_s", 0.5)),
    completion_position_hold_maximum_speed_mps_(
      declare_parameter<double>("completion_position_hold_maximum_speed_mps", 0.5)),
    completion_position_hold_settle_s_(
      declare_parameter<double>("completion_position_hold_settle_s", 0.5)),
    completion_safe_hover_maximum_speed_mps_(
      declare_parameter<double>("completion_safe_hover_maximum_speed_mps", 0.3)),
    completion_safe_hover_hold_s_(
      declare_parameter<double>("completion_safe_hover_hold_s", 2.0))
  {
    if (offboard_rate_hz_ < 2.0 || offboard_warmup_s_ < 1.0 ||
      preflight_stability_hold_s_ < 0.0 || state_timeout_s_ <= 0.0 ||
      status_timeout_s_ <= 0.0 ||
      observation_timeout_s_ <= 0.0 || estimate_timeout_s_ <= 0.0 ||
      prediction_coast_timeout_s_ < estimate_timeout_s_ ||
      prediction_maximum_position_variance_ <= 0.0 ||
      takeoff_height_m_ <= 0.0 || takeoff_tolerance_m_ <= 0.0 ||
      takeoff_speed_tolerance_mps_ <= 0.0 || handover_maximum_tilt_rad_ <= 0.0 ||
      handover_maximum_acceleration_mps2_ <= 0.0 ||
      handover_maximum_tilt_rad_ > maximum_tilt_rad_ || maximum_jerk_mps3_ <= 0.0 ||
      search_velocity_hold_s_ < 0.0 || search_velocity_decay_time_constant_s_ <= 0.0 ||
      search_coast_timeout_s_ <= 0.0 || search_velocity_gain_ <= 0.0 ||
      search_prediction_hold_s_ < 0.0 ||
      !std::isfinite(known_target_bearing_ned_rad_) ||
      known_target_search_hold_s_ < 0.0 ||
      known_target_search_acceleration_mps2_ < 0.0 ||
      known_target_search_acceleration_mps2_ > handover_maximum_acceleration_mps2_ ||
      known_target_search_acceleration_duration_s_ < 0.0 ||
      known_target_search_acceleration_duration_s_ > known_target_search_hold_s_ ||
      !std::isfinite(known_target_course_ned_rad_) ||
      known_target_search_velocity_mps_ < 0.0 ||
      known_target_velocity_gain_ <= 0.0 ||
      known_target_course_minimum_speed_fraction_ < 0.0 ||
      known_target_course_minimum_speed_fraction_ > 1.0 ||
      launch_position_planner_blend_ < 0.0 ||
      launch_position_planner_blend_ > 1.0 ||
      launch_planner_minimum_geometry_blend_ < 0.0 ||
      launch_planner_minimum_geometry_blend_ > 1.0 ||
      launch_planner_full_inbound_speed_mps_ <= 0.0 ||
      attack_enable_delay_s_ < 0.0 || first_attack_tracking_hold_s_ < 0.0 ||
      first_attack_depth_gap_tolerance_s_ < 0.0 ||
      first_attack_depth_gap_tolerance_s_ > prediction_coast_timeout_s_ ||
      terminal_alignment_tracking_hold_s_ < 0.0 ||
      terminal_alignment_ramp_s_ <= 0.0 ||
      capture_minimum_range_m_ <= safety_minimum_depth_m_ ||
      capture_maximum_range_m_ <= capture_minimum_range_m_ ||
      capture_maximum_alignment_rad_ <= 0.0 ||
      projectile_speed_mps_ <= 0.0 || shot_maximum_time_of_flight_s_ <= 0.0 ||
      shot_maximum_lead_angle_rad_ <= 0.0 || shot_aim_tolerance_rad_ <= 0.0 ||
      shot_aim_tolerance_rad_ > capture_maximum_alignment_rad_ ||
      shot_maximum_lateral_sigma_m_ <= 0.0 ||
      shot_hold_stationary_relative_speed_mps_ < 0.0 ||
      recoil_opening_speed_mps_ <= 0.0 ||
      egress_distance_m_ < 0.0 || egress_speed_mps_ <= 0.0 ||
      safety_maximum_speed_mps_ <= 0.0 || completion_braking_hold_s_ < 0.0 ||
      completion_position_hold_maximum_speed_mps_ <= 0.0 ||
      completion_position_hold_settle_s_ < 0.0 ||
      completion_safe_hover_maximum_speed_mps_ <= 0.0 ||
      completion_safe_hover_hold_s_ < 0.0)
    {
      throw std::invalid_argument("invalid pure interceptor configuration");
    }

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    auto origin_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    origin_qos.reliable().transient_local();
    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      topic("in/offboard_control_mode"), px4_qos);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic("in/trajectory_setpoint"), px4_qos);
    attitude_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      topic("in/vehicle_attitude_setpoint"), px4_qos);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      topic("in/vehicle_command"), px4_qos);
    local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topic("out/vehicle_local_position"), px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
        on_local_position(*message);
      });
    global_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      topic("out/vehicle_global_position"), px4_qos,
      [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr message) {
        const GeodeticPosition position{
          message->lat, message->lon, message->alt_ellipsoid};
        if (!CommonFrameTransform::is_valid(position)) {
          return;
        }
        latest_global_position_ = position;
        global_valid_ = true;
        if (local_valid_) {
          local_position_at_global_ = local_position_;
          local_at_global_valid_ = true;
        }
        publish_common_origin_if_ready();
      });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      topic("out/vehicle_attitude"), px4_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr message) {
        attitude_ = {
          message->q[0], message->q[1], message->q[2], message->q[3]};
        attitude_valid_ = finite_attitude(attitude_);
        attitude_update_ = std::chrono::steady_clock::now();
        // Keep a stamped history so observations can be rotated with the
        // attitude at their own capture time (search_los_ned/depth_los_ned).
        const double receipt_time_s = now().seconds();
        attitude_history_.add(
          SampleTimeAttitudeBuffer::px4_sample_or_receipt_time(
            message->timestamp, message->timestamp_sample, receipt_time_s,
            attitude_sync_tolerance_s_),
          Quaterniond{
            message->q[0], message->q[1], message->q[2], message->q[3]});
      });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      topic("out/vehicle_status"), px4_qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr message) {
        vehicle_status_ = *message;
        vehicle_status_valid_ = true;
        vehicle_status_update_ = now();
        if (message->arming_state ==
          px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
        {
          lock_task_origin("PX4 armed transition");
        }
      });
    search_sub_ = create_subscription<anti_drone_interfaces::msg::SearchObservation>(
      search_topic_, rclcpp::SensorDataQoS(),
      [this](anti_drone_interfaces::msg::SearchObservation::UniquePtr message) {
        search_observation_ = *message;
        search_update_ = rclcpp::Time(message->header.stamp);
      });
    depth_sub_ = create_subscription<anti_drone_interfaces::msg::DepthObservation>(
      depth_topic_, rclcpp::SensorDataQoS(),
      [this](anti_drone_interfaces::msg::DepthObservation::UniquePtr message) {
        depth_observation_ = *message;
        depth_update_ = rclcpp::Time(message->header.stamp);
      });
    estimate_sub_ = create_subscription<px4_offboard_tracking::msg::RelativeEstimate>(
      estimate_topic_, 10,
      [this](px4_offboard_tracking::msg::RelativeEstimate::UniquePtr message) {
        relative_estimate_ = *message;
        estimate_update_ = rclcpp::Time(message->header.stamp);
      });
    auto control_enable_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    control_enable_qos.reliable().transient_local();
    control_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, control_enable_qos,
      [this](std_msgs::msg::Bool::UniquePtr message) {
        on_control_enable(*message);
      });
    require_mission_enable_ =
      declare_parameter<bool>("require_mission_enable", false);
    mission_enable_topic_ = declare_parameter<std::string>(
      "mission_enable_topic", "/simulation/target_motion_ready");
    pursuer_takeoff_ready_topic_ = declare_parameter<std::string>(
      "pursuer_takeoff_ready_topic", "/simulation/pursuer_takeoff_ready");
    mission_enabled_ = !require_mission_enable_;
    if (require_mission_enable_) {
      auto enable_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      enable_qos.reliable().transient_local();
      pursuer_takeoff_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
        pursuer_takeoff_ready_topic_, enable_qos);
      mission_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        mission_enable_topic_, enable_qos,
        [this](std_msgs::msg::Bool::UniquePtr message) {
          if (message->data && !mission_enabled_) {
            mission_enabled_ = true;
            // The no-attack interval is defined relative to the target's
            // stabilized trajectory start, not controller construction or
            // local-origin acquisition during preflight.
            mission_start_ = now();
            RCLCPP_INFO(
              get_logger(),
              "mission enabled by external orchestration; search may start");
          }
        });
    }
    status_pub_ = create_publisher<px4_offboard_tracking::msg::PureInterceptorStatus>(
      status_topic_, 10);
    shot_pub_ =
      create_publisher<virtual_interception_interfaces::msg::VirtualShotEvent>(
      shot_topic_, 10);
    auto completion_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    completion_qos.reliable().transient_local();
    completion_pub_ = create_publisher<std_msgs::msg::Bool>(
      completion_topic_, completion_qos);
    offboard_session_started_pub_ = create_publisher<std_msgs::msg::Bool>(
      offboard_session_started_topic_, completion_qos);
    task_origin_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      task_origin_topic_, origin_qos);
    common_origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      common_origin_topic_, origin_qos);

    warmup_setpoint_count_ = static_cast<std::size_t>(
      std::ceil(offboard_rate_hz_ * offboard_warmup_s_));
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / offboard_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {on_timer();});
    RCLCPP_INFO(
      get_logger(),
      "pure interceptor ready: camera topics [%s, %s], estimate [%s], PX4 [%s], "
      "auto_engage=%s, enabled=%s, takeoff=%s, stop_on_exit=%s",
      search_topic_.c_str(), depth_topic_.c_str(), estimate_topic_.c_str(),
      topic_prefix_.c_str(), auto_engage_ ? "true" : "false",
      control_enabled_ ? "true" : "false", takeoff_enabled_ ? "true" : "false",
      auto_disable_on_offboard_exit_ ? "true" : "false");
  }

private:
  std::string topic(const std::string & suffix) const
  {
    std::string prefix = topic_prefix_;
    while (prefix.size() > 1 && prefix.back() == '/') {
      prefix.pop_back();
    }
    if (prefix.empty() || prefix.front() != '/') {
      prefix.insert(prefix.begin(), '/');
    }
    return prefix + "/" + suffix;
  }

  std::uint64_t timestamp_us()
  {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  static bool finite_attitude(const Quaternion & value)
  {
    return std::isfinite(value.w) && std::isfinite(value.x) &&
           std::isfinite(value.y) && std::isfinite(value.z);
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition & message)
  {
    // PX4 1.15 reports heading_good_for_control only after the final
    // in-flight magnetometer alignment. Requiring it before arming creates a
    // ground-state deadlock. Heading health before arming is instead covered
    // by the continuous VehicleStatus pre_flight_checks_pass gate together
    // with a fresh finite attitude below.
    if (!message.xy_valid || !message.z_valid ||
      !message.v_xy_valid || !message.v_z_valid)
    {
      return;
    }
    if (!local_reset_counters_initialized_) {
      local_xy_reset_counter_ = message.xy_reset_counter;
      local_z_reset_counter_ = message.z_reset_counter;
      local_reset_counters_initialized_ = true;
    } else {
      if (message.xy_reset_counter != local_xy_reset_counter_ &&
        std::isfinite(message.delta_xy[0]) &&
        std::isfinite(message.delta_xy[1]))
      {
        if (local_origin_set_ && task_origin_locked_) {
          local_origin_[0] += message.delta_xy[0];
          local_origin_[1] += message.delta_xy[1];
        }
        position_hold_[0] += message.delta_xy[0];
        position_hold_[1] += message.delta_xy[1];
        if (local_at_global_valid_) {
          local_position_at_global_[0] += message.delta_xy[0];
          local_position_at_global_[1] += message.delta_xy[1];
        }
        local_xy_reset_counter_ = message.xy_reset_counter;
        RCLCPP_WARN(
          get_logger(),
          "PX4 local XY reset compensated in task frame: dN=%.3f dE=%.3f m",
          message.delta_xy[0], message.delta_xy[1]);
        publish_task_origin();
      }
      if (message.z_reset_counter != local_z_reset_counter_ &&
        std::isfinite(message.delta_z))
      {
        if (local_origin_set_ && task_origin_locked_) {
          local_origin_[2] += message.delta_z;
        }
        position_hold_[2] += message.delta_z;
        if (local_at_global_valid_) {
          local_position_at_global_[2] += message.delta_z;
        }
        local_z_reset_counter_ = message.z_reset_counter;
        RCLCPP_WARN(
          get_logger(),
          "PX4 local Z reset compensated in task frame: dD=%.3f m",
          message.delta_z);
        publish_task_origin();
      }
    }
    local_position_ = {message.x, message.y, message.z};
    local_velocity_ = {message.vx, message.vy, message.vz};
    local_acceleration_ = {message.ax, message.ay, message.az};
    local_valid_ = finite_vector(local_position_) && finite_vector(local_velocity_);
    local_update_ = std::chrono::steady_clock::now();
    if (local_valid_ && !local_origin_set_) {
      local_origin_ = local_position_;
      position_hold_ = local_position_;
      position_hold_[2] = local_origin_[2] - takeoff_height_m_;
      local_origin_set_ = true;
      if (!require_mission_enable_) {
        mission_start_ = now();
      }
      search_yaw_rad_ = message.heading;
      RCLCPP_INFO(
        get_logger(), "initialized provisional takeoff origin [%.2f, %.2f, %.2f]",
        local_origin_[0], local_origin_[1], local_origin_[2]);
    } else if (local_valid_ && !task_origin_locked_ && !armed()) {
      // Do not freeze the frame at process startup.  Pre-arm estimator drift
      // follows this provisional origin and therefore cannot bias the task
      // frame or the 3 m clearance boundary.
      local_origin_ = local_position_;
      if (takeoff_enabled_) {
        position_hold_ = {
          local_origin_[0], local_origin_[1],
          local_origin_[2] - takeoff_height_m_};
      }
    }
    publish_common_origin_if_ready();
  }

  Vector task_position() const
  {
    return to_task_ned(local_position_, local_origin_);
  }

  void publish_task_origin()
  {
    if (!task_origin_locked_ || !task_origin_pub_) {
      return;
    }
    geometry_msgs::msg::PointStamped output;
    output.header.stamp = now();
    output.header.frame_id = "px4_local_ned";
    output.point.x = local_origin_[0];
    output.point.y = local_origin_[1];
    output.point.z = local_origin_[2];
    task_origin_pub_->publish(output);
  }

  void publish_common_origin_if_ready()
  {
    if (!task_origin_locked_ || common_origin_published_ || !global_valid_ ||
      !local_at_global_valid_ || !common_origin_pub_)
    {
      return;
    }
    const CommonFrameTransform current_global_frame(latest_global_position_);
    common_geodetic_origin_ = current_global_frame.from_ned({
      local_origin_[0] - local_position_at_global_[0],
      local_origin_[1] - local_position_at_global_[1],
      local_origin_[2] - local_position_at_global_[2]});
    sensor_msgs::msg::NavSatFix output;
    output.header.stamp = now();
    output.header.frame_id = frame_id_ + "_origin";
    output.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    output.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    output.latitude = common_geodetic_origin_.latitude_deg;
    output.longitude = common_geodetic_origin_.longitude_deg;
    output.altitude = common_geodetic_origin_.altitude_ellipsoid_m;
    common_origin_pub_->publish(output);
    common_origin_published_ = true;
    RCLCPP_INFO(
      get_logger(),
      "published task common NED origin lat=%.8f lon=%.8f alt=%.2f m",
      output.latitude, output.longitude, output.altitude);
  }

  void lock_task_origin(const char * reason)
  {
    if (task_origin_locked_ || !local_valid_) {
      return;
    }
    local_origin_ = local_position_;
    task_origin_locked_ = true;
    if (!require_mission_enable_) {
      mission_start_ = now();
    }
    publish_task_origin();
    publish_common_origin_if_ready();
    RCLCPP_INFO(
      get_logger(),
      "task NED origin locked at propeller start [%.2f, %.2f, %.2f]: %s",
      local_origin_[0], local_origin_[1], local_origin_[2], reason);
  }

  double message_age(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() <= 0) {
      return std::numeric_limits<double>::infinity();
    }
    const double age = (now() - stamp).seconds();
    return age >= 0.0 ? age : std::numeric_limits<double>::infinity();
  }

  bool local_fresh() const
  {
    return local_valid_ &&
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - local_update_).count() <= state_timeout_s_;
  }

  bool attitude_fresh() const
  {
    return attitude_valid_ &&
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - attitude_update_).count() <= state_timeout_s_;
  }

  bool status_fresh() const
  {
    return vehicle_status_valid_ &&
      message_age(vehicle_status_update_) <= status_timeout_s_;
  }

  bool armed() const
  {
    return status_fresh() &&
      vehicle_status_.arming_state ==
      px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool offboard_active() const
  {
    return status_fresh() &&
      vehicle_status_.nav_state ==
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool heartbeat_streaming() const
  {
    return control_enabled_ || auto_engage_;
  }

  bool setpoint_streaming() const
  {
    // ROS 2 Offboard liveness is carried by OffboardControlMode. Do not send
    // position/velocity/attitude setpoints until a fresh VehicleStatus sample
    // confirms that PX4 has actually entered Offboard. Keep streaming through
    // a later transient status gap; update_manual_offboard_session() stops the
    // manual controller only on a fresh, explicit Offboard exit.
    return heartbeat_streaming() && (offboard_active() || offboard_session_seen_);
  }

  bool flight_control_active() const
  {
    return auto_engage_ || (control_enabled_ && offboard_active());
  }

  void on_control_enable(const std_msgs::msg::Bool & message)
  {
    const bool enabling = message.data && !control_enabled_;
    if (enabling && stopped_after_offboard_exit_) {
      RCLCPP_ERROR(
        get_logger(),
        "rejected enable after Offboard exit; restart the process for a new sortie");
      return;
    }
    if (enabling && !auto_engage_ && !local_fresh()) {
      control_enabled_ = false;
      RCLCPP_ERROR(
        get_logger(), "rejected manual enable: PX4 local state is not fresh");
      return;
    }
    if (enabling && !auto_engage_ && offboard_active()) {
      control_enabled_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "rejected manual enable while already in Offboard; "
        "enable heartbeat pre-stream before switching modes");
      return;
    }
    control_enabled_ = message.data;
    if (enabling) {
      warmup_count_ = 0;
      offboard_session_seen_ = false;
      if (!takeoff_enabled_ && local_fresh()) {
        position_hold_ = local_position_;
        search_yaw_rad_ = current_yaw();
        if (!require_mission_enable_) {
          mission_start_ = now();
        }
        RCLCPP_INFO(
          get_logger(),
          "manual flight enable: latched hold [%.2f, %.2f, %.2f] NED",
          position_hold_[0], position_hold_[1], position_hold_[2]);
      }
    }
    if (!control_enabled_) {
      smoothed_acceleration_initialized_ = false;
      maneuver_feedforward_.reset();
    }
    RCLCPP_WARN(
      get_logger(), "pure interceptor control %s by enable topic",
      control_enabled_ ? "enabled" : "disabled");
  }

  void begin_first_offboard_session()
  {
    const auto session_start = now();
    capture_count_ = 0;
    attack_complete_ = false;
    safe_to_exit_offboard_ = false;
    completion_position_hold_active_ = false;
    completion_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    completion_braking_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    completion_hover_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    first_intercept_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    search_reacquisition_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    continuous_tracking_duration_s_ = 0.0;
    last_depth_guidance_ok_s_ = 0.0;
    attack_enable_delay_remaining_s_ = attack_enable_delay_s_;
    first_attack_tracking_hold_remaining_s_ = first_attack_tracking_hold_s_;
    tracking_ready_ = false;
    terminal_alignment_strength_ = 0.0;
    fire_authorized_ = false;
    attack_height_gate_passed_ = false;
    height_recovery_active_ = false;
    predictive_aim_active_ = false;
    shot_hard_gate_passed_ = false;
    intercept_range_latched_ = false;
    pursuer_takeoff_ready_published_ = false;
    last_target_velocity_valid_ = false;
    last_commanded_attack_axis_valid_ = false;
    safety_abort_latched_ = false;
    smoothed_acceleration_initialized_ = false;
    launch_planner_result_ = {};
    launch_planner_active_ = false;
    radial_control_source_ = RadialControlSource::FOV_HOLD;
    shot_timing_optimizer_.reset();
    shot_timing_result_ = {};
    maneuver_feedforward_.reset();
    staged_range_policy_.reset();
    bearing_handover_.reset();
    fov_barrier_.reset();
    search_pattern_.reset();
    mission_.reset(session_start.seconds());
    if (!require_mission_enable_ || mission_enabled_) {
      mission_start_ = session_start;
    } else {
      mission_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (local_fresh()) {
      position_hold_ = local_position_;
      search_yaw_rad_ = current_yaw();
    }
    std_msgs::msg::Bool session_started;
    session_started.data = true;
    offboard_session_started_pub_->publish(session_started);
    RCLCPP_WARN(
      get_logger(),
      "first Offboard entry: reset captures, attack timers and completion state");
  }

  void update_manual_offboard_session()
  {
    if (setpoint_streaming() && offboard_active()) {
      if (!offboard_session_seen_) {
        begin_first_offboard_session();
      }
      offboard_session_seen_ = true;
      return;
    }
    if (auto_engage_ || !auto_disable_on_offboard_exit_) {
      return;
    }
    // A delayed VehicleStatus sample is not evidence that the pilot changed
    // flight mode. Treating status staleness as an Offboard exit permanently
    // disabled the manual controller during a transient DDS/CPU scheduling
    // gap even though PX4 itself remained in Offboard. Stop only on a fresh,
    // explicit non-Offboard state; the next fresh RC/QGC mode sample still
    // cuts setpoints immediately.
    const bool explicit_offboard_exit =
      status_fresh() &&
      vehicle_status_.nav_state !=
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (control_enabled_ && offboard_session_seen_ && explicit_offboard_exit) {
      control_enabled_ = false;
      stopped_after_offboard_exit_ = true;
      smoothed_acceleration_initialized_ = false;
      maneuver_feedforward_.reset();
      RCLCPP_WARN(
        get_logger(),
        "manual Offboard exit detected: stopped all pure-interceptor setpoints");
    }
  }

  bool search_visible() const
  {
    return message_age(search_update_) <= observation_timeout_s_ &&
           search_observation_.interface_version ==
           anti_drone_interfaces::msg::SearchObservation::INTERFACE_VERSION_V1 &&
           search_observation_.visible &&
           search_observation_.angles_valid &&
           search_observation_.unit_los_valid &&
           std::isfinite(search_observation_.horizontal_angle_rad) &&
           std::isfinite(search_observation_.vertical_angle_rad) &&
           std::isfinite(search_observation_.unit_los_optical.x) &&
           std::isfinite(search_observation_.unit_los_optical.y) &&
           std::isfinite(search_observation_.unit_los_optical.z);
  }

  bool depth_visible() const
  {
    return message_age(depth_update_) <= observation_timeout_s_ &&
           depth_observation_.interface_version ==
           anti_drone_interfaces::msg::DepthObservation::INTERFACE_VERSION_V1 &&
           depth_observation_.visible &&
           depth_observation_.angles_valid &&
           depth_observation_.depth_valid &&
           std::isfinite(depth_observation_.position_optical.x) &&
           std::isfinite(depth_observation_.position_optical.y) &&
           std::isfinite(depth_observation_.position_optical.z) &&
           std::isfinite(depth_observation_.depth_z_m) &&
           depth_observation_.depth_z_m > 0.0 &&
           std::isfinite(depth_observation_.range_m) &&
           depth_observation_.range_m > 0.0;
  }

  bool estimate_valid() const
  {
    return message_age(estimate_update_) <= estimate_timeout_s_ &&
           relative_estimate_.valid &&
           estimate_state_finite();
  }

  bool estimate_state_finite() const
  {
    return
           std::isfinite(relative_estimate_.position.x) &&
           std::isfinite(relative_estimate_.position.y) &&
           std::isfinite(relative_estimate_.position.z) &&
           std::isfinite(relative_estimate_.velocity.x) &&
           std::isfinite(relative_estimate_.velocity.y) &&
           std::isfinite(relative_estimate_.velocity.z);
  }

  double estimate_maximum_position_variance() const
  {
    return std::max({
      relative_estimate_.covariance[0],
      relative_estimate_.covariance[7],
      relative_estimate_.covariance[14]});
  }

  bool estimate_usable_for_guidance() const
  {
    return message_age(estimate_update_) <= estimate_timeout_s_ &&
           estimate_state_finite() &&
           std::isfinite(relative_estimate_.data_age) &&
           relative_estimate_.data_age >= 0.0 &&
           relative_estimate_.data_age <= prediction_coast_timeout_s_ &&
           estimate_maximum_position_variance() <=
           prediction_maximum_position_variance_;
  }

  Vector current_attack_axis_ned() const
  {
    return normalized(rotate_body_to_ned(attitude_, {0.0, 0.0, -1.0}));
  }

  double current_tilt_rad() const
  {
    return angle_between({0.0, 0.0, -1.0}, current_attack_axis_ned());
  }

  // Attitude valid at a past observation stamp.  Rotating a capture-time
  // camera measurement with the latest attitude instead injects the full body
  // rotation accumulated during the observation latency (body_rate * ~0.3 s,
  // i.e. tens of degrees in a tracking turn).  Falls back to the latest
  // attitude only when the stamp falls outside the retained history.
  Quaternion attitude_at(const rclcpp::Time & stamp) const
  {
    Quaterniond sampled;
    if (attitude_history_.sample(stamp.seconds(), sampled)) {
      return {sampled.w, sampled.x, sampled.y, sampled.z};
    }
    return attitude_;
  }

  Vector search_los_ned() const
  {
    // R_body_from_search_optical from the approved camera definition.
    constexpr double sine = 0.17364817766693033;
    constexpr double cosine = 0.98480775301220802;
    const Vector optical{
      search_observation_.unit_los_optical.x,
      search_observation_.unit_los_optical.y,
      search_observation_.unit_los_optical.z};
    const Vector body{
      cosine * optical[1] + sine * optical[2],
      optical[0],
      sine * optical[1] - cosine * optical[2]};
    return normalized(rotate_body_to_ned(attitude_at(search_update_), body));
  }

  Vector depth_los_ned() const
  {
    const Vector optical{
      depth_observation_.position_optical.x,
      depth_observation_.position_optical.y,
      depth_observation_.position_optical.z};
    // R_body_from_depth_optical: body = [optical_y, optical_x, -optical_z].
    return normalized(rotate_body_to_ned(
      attitude_at(depth_update_), {optical[1], optical[0], -optical[2]}));
  }

  AccelerationCommand handover_acceleration(const Vector & requested_axis) const
  {
    constexpr double gravity = 9.80665;
    const Vector vertical_up{0.0, 0.0, -1.0};
    Vector axis = normalized(requested_axis);
    const double tilt = angle_between(vertical_up, axis);
    // Camera handover happens before unified guidance is valid. It still must
    // obey a physical acceleration limit: hover-compensated horizontal
    // acceleration is g*tan(tilt). The old tilt-only limit allowed ~5.7 m/s²
    // at 30 degrees and could trip the safety supervisor before interception.
    const double acceleration_limited_tilt = std::atan2(
      handover_maximum_acceleration_mps2_, gravity);
    const double allowed_tilt = std::min(
      handover_maximum_tilt_rad_, acceleration_limited_tilt);
    if (tilt > allowed_tilt) {
      axis = slerp_unit(
        vertical_up, axis, allowed_tilt / tilt);
    }
    const double upward_component = std::max(0.35, -axis[2]);
    const double specific_force = gravity / upward_component;
    const Vector acceleration = add({0.0, 0.0, gravity}, scale(axis, specific_force));
    AccelerationCommand output;
    output.ax = acceleration[0];
    output.ay = acceleration[1];
    output.az = acceleration[2];
    // Yaw does not change the body -Z attack axis. Preserve the acquisition
    // heading so the mixer does not create a large yaw transient (and extra
    // quadratic motor lift) while roll/pitch perform the camera handover.
    output.yaw = search_yaw_rad_;
    return output;
  }

  Vector handover_axis()
  {
    const bool use_depth = depth_visible();
    if (use_depth != handover_using_depth_) {
      bearing_handover_.reset();
      handover_using_depth_ = use_depth;
    }
    if (!use_depth && !search_visible()) {
      return current_attack_axis_ned();
    }
    const auto & header = use_depth ?
      depth_observation_.header : search_observation_.header;
    const std::uint32_t sequence = use_depth ?
      depth_observation_.sequence : search_observation_.sequence;
    const Vector measured_los = use_depth ? depth_los_ned() : search_los_ned();
    const auto result = bearing_handover_.update({
      measured_los, current_attack_axis_ned(), rclcpp::Time(header.stamp).seconds(),
      sequence, true});
    return result.valid ? result.commanded_axis_common : current_attack_axis_ned();
  }

  void update_fov_barrier()
  {
    const bool raw_depth_visible = depth_visible();
    fov_barrier_result_ = fov_barrier_.update({
      raw_depth_visible ? rclcpp::Time(depth_observation_.header.stamp).seconds() :
      now().seconds(),
      raw_depth_visible,
      estimate_usable_for_guidance(),
      raw_depth_visible ? depth_observation_.horizontal_angle_rad : 0.0,
      raw_depth_visible ? depth_observation_.vertical_angle_rad : 0.0,
      raw_depth_visible ? depth_observation_.sequence : 0U});
  }

  AccelerationCommand braking_acceleration(double altitude_reference_z) const
  {
    AccelerationCommand output;
    output.ax = std::clamp(
      -invalid_input_braking_gain_ * local_velocity_[0],
      -maximum_braking_acceleration_mps2_, maximum_braking_acceleration_mps2_);
    output.ay = std::clamp(
      -invalid_input_braking_gain_ * local_velocity_[1],
      -maximum_braking_acceleration_mps2_, maximum_braking_acceleration_mps2_);
    output.az = std::clamp(
      0.8 * (altitude_reference_z - local_position_[2]) -
      invalid_input_braking_gain_ * local_velocity_[2],
      -maximum_braking_acceleration_mps2_, maximum_braking_acceleration_mps2_);
    output.yaw = search_yaw_rad_;
    return output;
  }

  AccelerationCommand acquisition_limited_acceleration(
    const AccelerationCommand & requested) const
  {
    AccelerationCommand output = requested;
    const double horizontal = std::hypot(output.ax, output.ay);
    if (horizontal > handover_maximum_acceleration_mps2_) {
      const double scale = handover_maximum_acceleration_mps2_ / horizontal;
      output.ax *= scale;
      output.ay *= scale;
    }
    return output;
  }

  AccelerationCommand search_reacquisition_acceleration() const
  {
    const double coast_elapsed_s =
      search_reacquisition_started_.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() :
      (now() - search_reacquisition_started_).seconds();
    if (!last_target_velocity_valid_ ||
      search_reacquisition_started_.nanoseconds() == 0 ||
      coast_elapsed_s > search_coast_timeout_s_)
    {
      return braking_acceleration(position_hold_[2]);
    }
    // A constant-velocity coast sent the aircraft tens of metres along the
    // stale tangent when a circular/figure-eight target turned after camera
    // loss. Decay that camera/estimator-derived velocity during a full search
    // scan so reacquisition remains local without assuming target truth.
    const double velocity_decay = std::exp(
      -std::max(0.0, coast_elapsed_s - search_velocity_hold_s_) /
      search_velocity_decay_time_constant_s_);
    const Vector search_velocity = scale(last_target_velocity_, velocity_decay);
    AccelerationCommand output;
    output.ax = std::clamp(
      search_velocity_gain_ *
      (search_velocity[0] - local_velocity_[0]),
      -maximum_braking_acceleration_mps2_,
      maximum_braking_acceleration_mps2_);
    output.ay = std::clamp(
      search_velocity_gain_ *
      (search_velocity[1] - local_velocity_[1]),
      -maximum_braking_acceleration_mps2_,
      maximum_braking_acceleration_mps2_);
    output.az = std::clamp(
      0.8 * (position_hold_[2] - local_position_[2]) -
      search_velocity_gain_ * local_velocity_[2],
      -maximum_braking_acceleration_mps2_,
      maximum_braking_acceleration_mps2_);
    output.yaw = search_yaw_rad_;
    return output;
  }

  AccelerationCommand known_direction_search_acceleration(
    double search_elapsed_s) const
  {
    if (search_elapsed_s >= known_target_search_acceleration_duration_s_ ||
      known_target_search_acceleration_mps2_ <= 0.0)
    {
      return braking_acceleration(position_hold_[2]);
    }
    // A finite initial tilt brings an out-of-cone target into the upward
    // camera without using range or target truth. It is deliberately
    // time-bounded; after the short slew the normal braking/search policy
    // resumes even if the operator-provided bearing was wrong.
    AccelerationCommand output;
    output.ax = known_target_search_acceleration_mps2_ *
      std::cos(known_target_bearing_ned_rad_);
    output.ay = known_target_search_acceleration_mps2_ *
      std::sin(known_target_bearing_ned_rad_);
    output.az = std::clamp(
      0.8 * (position_hold_[2] - local_position_[2]) -
      invalid_input_braking_gain_ * local_velocity_[2],
      -maximum_braking_acceleration_mps2_,
      maximum_braking_acceleration_mps2_);
    output.yaw = known_target_bearing_ned_rad_;
    return output;
  }

  AccelerationCommand known_course_velocity_acceleration() const
  {
    // The course cue is range-free. Map the encounter angle continuously:
    // inbound gets the configured floor, a perpendicular crossing gets the
    // midpoint, and an escaping target gets full course matching. Clamping the
    // cosine to [0, 1] used to collapse inbound and crossing encounters into
    // the same slow cue, so an 8 m/s crossing target left the search FOV before
    // stereo could become stable.
    const double course_alignment = std::clamp(
      std::cos(known_target_course_ned_rad_ - known_target_bearing_ned_rad_),
      -1.0, 1.0);
    const double normalized_encounter_geometry = 0.5 * (course_alignment + 1.0);
    const double course_speed_fraction =
      known_target_course_minimum_speed_fraction_ +
      (1.0 - known_target_course_minimum_speed_fraction_) *
      normalized_encounter_geometry;
    const double geometry_safe_speed =
      known_target_search_velocity_mps_ * course_speed_fraction;
    const double desired_north = geometry_safe_speed *
      std::cos(known_target_course_ned_rad_);
    const double desired_east = geometry_safe_speed *
      std::sin(known_target_course_ned_rad_);
    AccelerationCommand output;
    output.ax = std::clamp(
      known_target_velocity_gain_ * (desired_north - local_velocity_[0]),
      -handover_maximum_acceleration_mps2_,
      handover_maximum_acceleration_mps2_);
    output.ay = std::clamp(
      known_target_velocity_gain_ * (desired_east - local_velocity_[1]),
      -handover_maximum_acceleration_mps2_,
      handover_maximum_acceleration_mps2_);
    output.az = std::clamp(
      0.8 * (position_hold_[2] - local_position_[2]) -
      invalid_input_braking_gain_ * local_velocity_[2],
      -maximum_braking_acceleration_mps2_,
      maximum_braking_acceleration_mps2_);
    output.yaw = known_target_bearing_ned_rad_;
    return output;
  }

  AccelerationCommand smooth_acceleration(const AccelerationCommand & requested)
  {
    const double maximum_step = maximum_jerk_mps3_ / offboard_rate_hz_;
    if (!smoothed_acceleration_initialized_) {
      // Position control hands over near hover, whose inertial acceleration is
      // zero. Start there and apply the same slew limit as every later sample
      // instead of stepping directly to the first attitude-mode command.
      smoothed_acceleration_ = {};
      smoothed_acceleration_.yaw = requested.yaw;
      smoothed_acceleration_initialized_ = true;
    }
    smoothed_acceleration_.ax += std::clamp(
      requested.ax - smoothed_acceleration_.ax, -maximum_step, maximum_step);
    smoothed_acceleration_.ay += std::clamp(
      requested.ay - smoothed_acceleration_.ay, -maximum_step, maximum_step);
    smoothed_acceleration_.az += std::clamp(
      requested.az - smoothed_acceleration_.az, -maximum_step, maximum_step);
    smoothed_acceleration_.yaw = requested.yaw;
    return smoothed_acceleration_;
  }

  AccelerationCommand preserve_guidance_axis_constraint(
    const AccelerationCommand & requested,
    const Unified3dGuidanceResult & guidance) const
  {
    if (!guidance.valid) {
      return requested;
    }
    constexpr double gravity = 9.80665;
    const Vector gravity_ned{0.0, 0.0, gravity};
    const Vector acceleration{requested.ax, requested.ay, requested.az};
    const Vector specific_force{
      acceleration[0],
      acceleration[1],
      acceleration[2] - gravity};
    const double magnitude = norm(specific_force);
    if (!std::isfinite(magnitude) || magnitude < 1e-3) {
      return requested;
    }

    Vector axis = normalized(specific_force);
    const bool use_predictive_aim =
      predictive_aim_active_ && current_shot_solution_.valid;
    const Vector reference_axis = use_predictive_aim ?
      current_shot_solution_.shot_direction_ned : guidance.line_of_sight_ned;
    const double allowed_error = use_predictive_aim ?
      shot_aim_tolerance_rad_ : guidance.allowed_axis_error_rad;
    const double reference_error = angle_between(axis, reference_axis);
    if (reference_error > allowed_error) {
      axis = slerp_unit(
        reference_axis, axis,
        allowed_error / reference_error);
    }
    const Vector vertical_up{0.0, 0.0, -1.0};
    const double tilt = angle_between(vertical_up, axis);
    if (tilt > maximum_tilt_rad_) {
      axis = slerp_unit(vertical_up, axis, maximum_tilt_rad_ / tilt);
    }

    const Vector constrained = add(gravity_ned, scale(axis, magnitude));
    AccelerationCommand output = requested;
    output.ax = constrained[0];
    output.ay = constrained[1];
    output.az = constrained[2];
    return output;
  }

  AccelerationCommand apply_vertical_acceleration_preserving_axis(
    const AccelerationCommand & requested,
    double desired_acceleration_down_mps2) const
  {
    constexpr double gravity = 9.80665;
    AccelerationCommand output = requested;
    output.az = desired_acceleration_down_mps2;
    const Vector axis = normalized({
      requested.ax, requested.ay, requested.az - gravity});
    if (!finite_vector(axis) || axis[2] >= -1.0e-3) {
      return output;
    }
    const double specific_force_magnitude =
      (desired_acceleration_down_mps2 - gravity) / axis[2];
    if (!std::isfinite(specific_force_magnitude) ||
      specific_force_magnitude <= 1.0e-3)
    {
      return output;
    }
    output.ax = axis[0] * specific_force_magnitude;
    output.ay = axis[1] * specific_force_magnitude;
    return output;
  }

  RangeControlStage active_range_stage() const
  {
    // Once stable three-dimensional interception has been established, short
    // camera/FOV recovery must not restore the old 20--30 m acquisition band.
    if (intercept_range_latched_ &&
      mission_.state() != PureMissionState::SHOT_READY &&
      mission_.state() != PureMissionState::EGRESS_REATTACK)
    {
      return RangeControlStage::INTERCEPT;
    }
    switch (mission_.state()) {
      case PureMissionState::SEARCH:
      case PureMissionState::BEARING_TRACK:
      case PureMissionState::DEPTH_HANDOVER:
        return RangeControlStage::SEARCH_HANDOVER;
      case PureMissionState::DEPTH_ACQUIRE:
        return RangeControlStage::DEPTH_ACQUIRE;
      case PureMissionState::SHOT_READY:
        return RangeControlStage::SHOT;
      case PureMissionState::EGRESS_REATTACK:
        return RangeControlStage::EGRESS;
      default:
        return RangeControlStage::INTERCEPT;
    }
  }

  double active_fov_axis_constraint_rad() const
  {
    const auto & config = fov_barrier_.config();
    if (fov_barrier_result_.zone == FovBarrierZone::CENTERING) {
      return config.centering_boundary_rad -
             fov_barrier_result_.centering_strength *
             (config.centering_boundary_rad - config.normal_boundary_rad);
    }
    if (fov_barrier_result_.zone == FovBarrierZone::RECOVERY ||
      fov_barrier_result_.zone == FovBarrierZone::IMM_COAST ||
      fov_barrier_result_.zone == FovBarrierZone::LOST)
    {
      return config.normal_boundary_rad;
    }
    return config.centering_boundary_rad;
  }

  double fov_approach_scale() const
  {
    if (fov_barrier_result_.zone == FovBarrierZone::NORMAL) {
      return 1.0;
    }
    if (fov_barrier_result_.zone == FovBarrierZone::CENTERING) {
      return std::clamp(1.0 - fov_barrier_result_.centering_strength, 0.0, 1.0);
    }
    return 0.0;
  }

  double selected_closing_speed(RadialControlSource source)
  {
    if (!estimate_usable_for_guidance()) {
      range_policy_result_ = {};
      return 0.0;
    }
    const Vector position{
      relative_estimate_.position.x,
      relative_estimate_.position.y,
      relative_estimate_.position.z};
    const double range_m = norm(position);
    if (source == RadialControlSource::APPROACH_17M) {
      range_policy_result_ = staged_range_policy_.update(
        RangeControlStage::INTERCEPT, range_m,
        1.0 / offboard_rate_hz_, 1.0);
      return range_policy_result_.desired_closing_speed_mps;
    }

    range_policy_result_ = {};
    range_policy_result_.active_reference_m =
      source == RadialControlSource::LAUNCH_PLANNER &&
      launch_planner_result_.valid ?
      launch_planner_result_.launch_range_m : range_m;
    range_policy_result_.lower_bound_m = capture_minimum_range_m_;
    range_policy_result_.upper_bound_m = capture_maximum_range_m_;
    range_policy_result_.approach_paused_for_fov =
      source == RadialControlSource::FOV_HOLD;
    range_policy_result_.approach_scale =
      source == RadialControlSource::FOV_HOLD ? 0.0 : 1.0;
    if (source == RadialControlSource::RECOIL) {
      range_policy_result_.desired_closing_speed_mps =
        -recoil_opening_speed_mps_;
    }
    return range_policy_result_.desired_closing_speed_mps;
  }

  Unified3dGuidanceResult guidance_result(double terminal_alignment_strength)
  {
    if (!estimate_usable_for_guidance()) {
      return {};
    }
    const Vector relative_acceleration{
      relative_estimate_.acceleration.x,
      relative_estimate_.acceleration.y,
      relative_estimate_.acceleration.z};
    Vector target_acceleration{};
    if (finite_vector(relative_acceleration) && finite_vector(local_acceleration_)) {
      target_acceleration = add(relative_acceleration, local_acceleration_);
    }
    const double maximum_acceleration_variance = std::max({
      relative_estimate_.full_covariance[60],
      relative_estimate_.full_covariance[70],
      relative_estimate_.full_covariance[80]});
    const double maximum_acceleration_stddev =
      maximum_acceleration_variance >= 0.0 ?
      std::sqrt(maximum_acceleration_variance) :
      std::numeric_limits<double>::quiet_NaN();
    // Acceleration feed-forward is useful only while stereo is fresh and the
    // target is already inside the normal camera cone.  Suppress it during
    // centering/recovery/coast: there the primary objective is restoring the
    // optical axis, and an uncertain maneuver estimate can otherwise amplify
    // the very excursion that caused the recovery state.
    const bool maneuver_prediction_allowed =
      depth_visible() &&
      (mission_.state() == PureMissionState::INTERCEPT ||
      mission_.state() == PureMissionState::SHOT_READY) &&
      fov_barrier_result_.zone == FovBarrierZone::NORMAL;
    const auto feedforward = maneuver_feedforward_.update(
      target_acceleration,
      relative_estimate_.model_probabilities[1],
      maximum_acceleration_stddev,
      estimate_valid() && relative_estimate_.measurement_accepted &&
      maneuver_prediction_allowed,
      1.0 / offboard_rate_hz_);
    Unified3dGuidanceInput input;
    input.relative_position_ned = {
      relative_estimate_.position.x,
      relative_estimate_.position.y,
      relative_estimate_.position.z};
    input.relative_velocity_ned = {
      relative_estimate_.velocity.x,
      relative_estimate_.velocity.y,
      relative_estimate_.velocity.z};
    input.target_acceleration_ned =
      feedforward.filtered_target_acceleration_ned;
    input.target_acceleration_confidence = feedforward.confidence;
    launch_planner_result_ = {};
    launch_planner_active_ = false;
    const bool planner_state =
      mission_.state() == PureMissionState::DEPTH_ACQUIRE ||
      mission_.state() == PureMissionState::INTERCEPT ||
      mission_.state() == PureMissionState::SHOT_READY;
    if (launch_position_planner_enabled_ && planner_state &&
      depth_visible() && estimate_valid() &&
      relative_estimate_.measurement_accepted)
    {
      LaunchPositionPlannerInput planner_input;
      planner_input.relative_position_ned = input.relative_position_ned;
      planner_input.relative_velocity_ned = input.relative_velocity_ned;
      planner_input.target_acceleration_ned = target_acceleration;
      planner_input.pursuer_velocity_ned = local_velocity_;
      if (smoothed_acceleration_initialized_) {
        planner_input.previous_acceleration_ned = {
          smoothed_acceleration_.ax,
          smoothed_acceleration_.ay,
          smoothed_acceleration_.az};
      }
      planner_input.target_acceleration_confidence = feedforward.confidence;
      planner_input.maximum_executable_horizontal_speed_mps =
        command_speed_limiter_.config().maximum_speed_mps;
      planner_input.maximum_executable_vertical_speed_mps =
        vertical_speed_limiter_.config().maximum_speed_mps;
      launch_planner_result_ = launch_position_planner_.plan(planner_input);
      if (launch_planner_result_.valid) {
        const Vector target_velocity{
          local_velocity_[0] + input.relative_velocity_ned[0],
          local_velocity_[1] + input.relative_velocity_ned[1],
          local_velocity_[2] + input.relative_velocity_ned[2]};
        const Vector line_of_sight = scale(
          input.relative_position_ned,
          1.0 / std::max(1.0e-6, norm(input.relative_position_ned)));
        const double target_inbound_speed = -dot(target_velocity, line_of_sight);
        const double inbound_fraction = std::clamp(
          target_inbound_speed / launch_planner_full_inbound_speed_mps_,
          0.0, 1.0);
        const double geometry_blend =
          launch_planner_minimum_geometry_blend_ +
          (1.0 - launch_planner_minimum_geometry_blend_) * inbound_fraction;
        // During centering the planner fades continuously instead of fighting
        // the camera-axis recovery. Incoming targets receive the full planner
        // command so excessive natural closure is not mistaken for overshoot;
        // crossing and escaping targets retain more PN/FOV stabilization.
        input.launch_planner_blend = launch_position_planner_blend_ *
          geometry_blend;
      }
    }
    const bool fov_holds_range =
      mission_.state() == PureMissionState::FOV_RECOVERY ||
      mission_.state() == PureMissionState::IMM_COAST ||
      fov_approach_scale() <= 1.0e-6;
    if (mission_.state() == PureMissionState::EGRESS_REATTACK) {
      radial_control_source_ = RadialControlSource::RECOIL;
      input.launch_planner_blend = 0.0;
    } else if (fov_holds_range) {
      radial_control_source_ = RadialControlSource::FOV_HOLD;
      input.launch_planner_blend = 0.0;
    } else if (high_quality_shot_geometry(current_shot_solution_)) {
      radial_control_source_ = RadialControlSource::SHOT_HOLD;
      input.launch_planner_blend = 0.0;
    } else if (launch_planner_result_.valid &&
      input.launch_planner_blend > 1.0e-6)
    {
      radial_control_source_ = RadialControlSource::LAUNCH_PLANNER;
      input.planned_acceleration_ned = launch_planner_result_.acceleration_ned;
      launch_planner_active_ = true;
    } else {
      radial_control_source_ = RadialControlSource::APPROACH_17M;
      input.launch_planner_blend = 0.0;
    }
    input.desired_closing_speed_mps = selected_closing_speed(
      radial_control_source_);
    input.maximum_axis_error_rad = active_fov_axis_constraint_rad();
    input.terminal_alignment_required = terminal_alignment_strength > 0.0;
    input.terminal_alignment_strength = terminal_alignment_strength;
    return guidance_.update(input);
  }

  PredictiveShotResult predictive_shot_solution() const
  {
    if (!depth_visible() || !estimate_valid()) {
      return {};
    }
    const Vector relative_acceleration{
      relative_estimate_.acceleration.x,
      relative_estimate_.acceleration.y,
      relative_estimate_.acceleration.z};
    Vector target_acceleration{};
    if (finite_vector(relative_acceleration) && finite_vector(local_acceleration_)) {
      target_acceleration = add(relative_acceleration, local_acceleration_);
    }
    PredictiveShotInput input;
    input.relative_position_ned = {
      relative_estimate_.position.x,
      relative_estimate_.position.y,
      relative_estimate_.position.z};
    input.relative_velocity_ned = {
      relative_estimate_.velocity.x,
      relative_estimate_.velocity.y,
      relative_estimate_.velocity.z};
    input.estimated_target_acceleration_ned = target_acceleration;
    input.state_covariance = relative_estimate_.full_covariance;
    return shot_solver_.solve(input);
  }

  bool high_quality_shot_geometry(const PredictiveShotResult & shot) const
  {
    if (!depth_visible() || !estimate_valid() ||
      !relative_estimate_.measurement_accepted || !shot.valid ||
      depth_observation_.range_m < capture_minimum_range_m_ ||
      depth_observation_.range_m > capture_maximum_range_m_ ||
      shot.time_of_flight_s > shot_maximum_time_of_flight_s_ ||
      shot.lead_angle_rad > shot_maximum_lead_angle_rad_ ||
      shot.predicted_lateral_sigma_m > shot_maximum_lateral_sigma_m_)
    {
      return false;
    }
    const double actual_shot_axis_error = angle_between(
      current_attack_axis_ned(), shot.shot_direction_ned);
    const Vector relative_velocity{
      relative_estimate_.velocity.x,
      relative_estimate_.velocity.y,
      relative_estimate_.velocity.z};
    // A moving target is held only after it is already inside the accepted
    // firing cone. A genuinely stationary target that starts at 25 m is the
    // explicit exception: preserve range while the attitude loop aligns.
    return actual_shot_axis_error <= capture_maximum_alignment_rad_ ||
           norm(relative_velocity) <= shot_hold_stationary_relative_speed_mps_;
  }

  bool shot_base_gate(
    const Unified3dGuidanceResult & guidance,
    const PredictiveShotResult & shot) const
  {
    if (!depth_visible() || !estimate_valid() ||
      !relative_estimate_.measurement_accepted ||
      !guidance.valid || !shot.valid)
    {
      return false;
    }
    const double actual_shot_axis_error = angle_between(
      current_attack_axis_ned(), shot.shot_direction_ned);
    return depth_observation_.range_m >= capture_minimum_range_m_ &&
           depth_observation_.range_m <= capture_maximum_range_m_ &&
           shot.time_of_flight_s <= shot_maximum_time_of_flight_s_ &&
           shot.lead_angle_rad <= shot_maximum_lead_angle_rad_ &&
           actual_shot_axis_error <= capture_maximum_alignment_rad_ &&
           shot.predicted_lateral_sigma_m <= shot_maximum_lateral_sigma_m_;
  }

  bool safety_abort_requested(
    bool local_ok, bool attitude_ok, std::string & reason) const
  {
    if (!local_ok) {
      reason = "PX4 local state stale or invalid";
      return true;
    }
    if (!attitude_ok) {
      reason = "PX4 attitude stale or invalid";
      return true;
    }
    if (norm(local_velocity_) > safety_maximum_speed_mps_) {
      reason = "vehicle speed exceeded safety limit";
      return true;
    }
    if (current_tilt_rad() > safety_maximum_tilt_rad_) {
      reason = "vehicle tilt exceeded safety limit";
      return true;
    }
    if (depth_visible() && depth_observation_.range_m < safety_minimum_depth_m_) {
      reason = "depth range violated minimum safety distance";
      return true;
    }
    reason.clear();
    return false;
  }

  bool recovery_ready() const
  {
    return local_fresh() && attitude_fresh() &&
           norm(local_velocity_) <= recovery_maximum_speed_mps_ &&
           current_tilt_rad() <= recovery_maximum_tilt_rad_;
  }

  bool position_control_state(PureMissionState state) const
  {
    if (height_recovery_active_) {
      return false;
    }
    return state == PureMissionState::INIT ||
           state == PureMissionState::TAKEOFF_SAFE ||
           state == PureMissionState::SAFETY_ABORT ||
           (state == PureMissionState::COMPLETE && completion_position_hold_active_);
  }

  bool velocity_braking_state(PureMissionState state) const
  {
    return !height_recovery_active_ &&
           state == PureMissionState::COMPLETE &&
           !completion_position_hold_active_;
  }

  void publish_offboard_mode(PureMissionState state)
  {
    px4_msgs::msg::OffboardControlMode output;
    output.timestamp = timestamp_us();
    output.position = position_control_state(state);
    output.velocity = velocity_braking_state(state);
    output.attitude = !output.position && !output.velocity;
    offboard_pub_->publish(output);
  }

  void publish_zero_velocity_setpoint()
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint output;
    output.timestamp = timestamp_us();
    output.position = {nan, nan, nan};
    output.velocity = {0.0F, 0.0F, 0.0F};
    output.acceleration = {nan, nan, nan};
    output.jerk = {nan, nan, nan};
    output.yaw = static_cast<float>(search_yaw_rad_);
    output.yawspeed = nan;
    trajectory_pub_->publish(output);
  }

  void publish_position_setpoint(PureMissionState state)
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint output;
    output.timestamp = timestamp_us();
    output.position = {nan, nan, nan};
    output.velocity = {nan, nan, nan};
    output.acceleration = {nan, nan, nan};
    output.jerk = {nan, nan, nan};
    output.yaw = nan;
    output.yawspeed = nan;
    Vector setpoint = position_hold_;
    if (takeoff_enabled_ &&
      (state == PureMissionState::INIT || state == PureMissionState::TAKEOFF_SAFE))
    {
      setpoint = {
        local_origin_[0], local_origin_[1], local_origin_[2] - takeoff_height_m_};
    }
    output.position = {
      static_cast<float>(setpoint[0]),
      static_cast<float>(setpoint[1]),
      static_cast<float>(setpoint[2])};
    output.yaw = static_cast<float>(search_yaw_rad_);
    trajectory_pub_->publish(output);
  }

  void publish_attitude_setpoint(
    const AccelerationCommand & raw_acceleration,
    const Unified3dGuidanceResult * guidance = nullptr)
  {
    auto requested = command_speed_limiter_.apply(
      raw_acceleration, local_velocity_[0], local_velocity_[1]).acceleration;
    requested.az = vertical_speed_limiter_.apply(
      requested.az, local_velocity_[2]).acceleration_down_mps2;
    const double height_m = height_above_task_origin_m(
      local_position_, local_origin_);
    const auto requested_height_constraint = minimum_height_constraint_.apply({
      height_m, local_velocity_[2], requested.az});
    requested.az = requested_height_constraint.acceleration_down_mps2;
    // The upward attack axis is fully defined by roll/pitch. Keeping yaw fixed
    // removes the LOS-heading wrap and the old alternating lateral/yaw motion.
    requested.yaw = search_yaw_rad_;
    auto acceleration = smooth_acceleration(requested);
    // The horizontal speed limiter and jerk limiter act after unified guidance.
    // Without this final projection they can rotate the actually commanded
    // body -Z axis outside the depth-camera cone even though the guidance
    // diagnostic still reports a one-degree-constrained axis.
    if (guidance != nullptr && guidance->valid) {
      acceleration = preserve_guidance_axis_constraint(acceleration, *guidance);
    }
    // Safety has the final authority. Reapply both speed bounds after jerk
    // smoothing and the attack-axis/FOV projection. Otherwise the smoother
    // can retain outward acceleration from the previous sample and defeat the
    // horizontal speed cap while the vehicle is already near its limit.
    acceleration = command_speed_limiter_.apply(
      acceleration, local_velocity_[0], local_velocity_[1]).acceleration;
    const double final_vertical_acceleration = vertical_speed_limiter_.apply(
      acceleration.az, local_velocity_[2]).acceleration_down_mps2;
    const auto final_height_constraint = minimum_height_constraint_.apply({
      height_m, local_velocity_[2], final_vertical_acceleration});
    acceleration = apply_vertical_acceleration_preserving_axis(
      acceleration, final_height_constraint.acceleration_down_mps2);
    minimum_height_constraint_active_ =
      requested_height_constraint.active || final_height_constraint.active;
    minimum_height_acceleration_limit_down_mps2_ =
      final_height_constraint.maximum_allowed_acceleration_down_mps2;
    const auto thrust_priority =
      geometric_control_.prioritize_vertical_acceleration(acceleration);
    acceleration = thrust_priority.acceleration;
    smoothed_acceleration_ = acceleration;
    auto desired = geometric_control_.attitude_from_acceleration(acceleration);
    last_commanded_attack_axis_ = normalized({
      acceleration.ax, acceleration.ay, acceleration.az - 9.80665});
    last_commanded_attack_axis_valid_ =
      norm(last_commanded_attack_axis_) > 0.5;
    const Vector current_axis = current_attack_axis_ned();
    requested_thrust_before_saturation_ =
      geometric_control_.transient_compensated_thrust_request(
      acceleration.az, -current_axis[2]);
    desired.thrust = geometric_control_.transient_compensated_thrust(
      acceleration.az, -current_axis[2]);
    commanded_thrust_ = desired.thrust;
    thrust_saturated_ = thrust_priority.saturated ||
      requested_thrust_before_saturation_ > maximum_thrust_ + 1.0e-6;
    px4_msgs::msg::VehicleAttitudeSetpoint output;
    output.timestamp = timestamp_us();
    output.roll_body = static_cast<float>(desired.roll);
    output.pitch_body = static_cast<float>(desired.pitch);
    output.yaw_body = static_cast<float>(desired.yaw);
    output.q_d = {
      static_cast<float>(desired.attitude.w),
      static_cast<float>(desired.attitude.x),
      static_cast<float>(desired.attitude.y),
      static_cast<float>(desired.attitude.z)};
    output.thrust_body = {0.0F, 0.0F, static_cast<float>(-desired.thrust)};
    attitude_setpoint_pub_->publish(output);
  }

  void publish_control_setpoint(
    PureMissionState state, const Unified3dGuidanceResult & guidance)
  {
    if (height_recovery_active_) {
      const double recovery_z = local_origin_[2] -
        minimum_height_constraint_.config().recovery_height_m;
      publish_attitude_setpoint(braking_acceleration(recovery_z));
      return;
    }
    if (velocity_braking_state(state)) {
      smoothed_acceleration_initialized_ = false;
      smoothed_acceleration_ = {};
      commanded_thrust_ = 0.0;
      requested_thrust_before_saturation_ = 0.0;
      thrust_saturated_ = false;
      last_commanded_attack_axis_valid_ = false;
      publish_zero_velocity_setpoint();
      return;
    }
    if (position_control_state(state)) {
      smoothed_acceleration_initialized_ = false;
      smoothed_acceleration_ = {};
      commanded_thrust_ = 0.0;
      requested_thrust_before_saturation_ = 0.0;
      thrust_saturated_ = false;
      last_commanded_attack_axis_valid_ = false;
      publish_position_setpoint(state);
      return;
    }
    if (state == PureMissionState::SEARCH) {
      const double search_elapsed = search_reacquisition_started_.nanoseconds() > 0 ?
        std::max(0.0, (now() - search_reacquisition_started_).seconds()) : 0.0;
      const bool initial_known_direction_search =
        known_target_direction_enabled_ && !last_target_velocity_valid_ &&
        search_elapsed < known_target_search_hold_s_;
      if (initial_known_direction_search) {
        // An operator/RTK cue supplies only a fixed NED bearing, never target
        // truth.  Point the long axis of the wide search camera toward that
        // sector and hold position while an incoming target enters its cone.
        // If acquisition does not occur, fall back to the normal yaw scan.
        search_yaw_rad_ = known_target_bearing_ned_rad_;
        search_pattern_result_ = {};
        search_pattern_result_.pending = true;
        search_pattern_result_.completed = false;
        search_pattern_result_.desired_yaw_rad = search_yaw_rad_;
      } else if (search_elapsed < search_prediction_hold_s_) {
        // First search locally around the predicted target motion. Keeping the
        // last acquisition yaw prevents a wide scan from throwing the camera
        // away from a target that only disappeared for a few frames.
        search_pattern_result_ = {};
        search_pattern_result_.pending = true;
        search_pattern_result_.completed = false;
        search_pattern_result_.desired_yaw_rad = search_yaw_rad_;
      } else {
        search_pattern_result_ = search_pattern_.update(
          true, search_visible(), now().seconds(), current_yaw());
        search_yaw_rad_ = search_pattern_result_.desired_yaw_rad;
      }
      const bool direction_slew_active =
        initial_known_direction_search && !known_direction_bearing_seen_ &&
        search_elapsed < known_target_search_acceleration_duration_s_;
      if (direction_slew_active) {
        publish_attitude_setpoint(
          known_direction_search_acceleration(search_elapsed));
      } else if (known_target_course_enabled_ && !last_target_velocity_valid_) {
        // Once a bearing has been seen, use only the course component that
        // carries the target away along the bearing. Incoming/crossing cues
        // hold instead of making the pursuer retreat. The estimate replaces
        // this range-free cue as soon as depth is stable.
        publish_attitude_setpoint(known_course_velocity_acceleration());
      } else {
        publish_attitude_setpoint(search_reacquisition_acceleration());
      }
      return;
    }
    if (state == PureMissionState::BEARING_TRACK ||
      state == PureMissionState::DEPTH_HANDOVER)
    {
      if (known_target_course_enabled_ && !last_target_velocity_valid_) {
        publish_attitude_setpoint(known_course_velocity_acceleration());
        return;
      }
      if (intercept_range_latched_ && last_target_velocity_valid_) {
        // A wide-camera bearing is a buffer around depth tracking, not a
        // request to discard the already-estimated target velocity. Continue
        // matching that velocity while constraining the attack axis with the
        // available bearing; otherwise every brief reacquisition brakes the
        // pursuer and lets a 5 m/s target leave the search camera again.
        Unified3dGuidanceResult buffered_bearing;
        buffered_bearing.valid = true;
        buffered_bearing.line_of_sight_ned = handover_axis();
        buffered_bearing.allowed_axis_error_rad =
          fov_barrier_.config().normal_boundary_rad;
        publish_attitude_setpoint(
          search_reacquisition_acceleration(), &buffered_bearing);
        return;
      }
      publish_attitude_setpoint(handover_acceleration(handover_axis()));
      return;
    }
    if (state == PureMissionState::DEPTH_ACQUIRE) {
      if (guidance.valid) {
        publish_attitude_setpoint(
          acquisition_limited_acceleration(guidance.acceleration), &guidance);
        return;
      }
      publish_attitude_setpoint(handover_acceleration(handover_axis()));
      return;
    }
    if (state == PureMissionState::FOV_RECOVERY) {
      // Recovery is a constraint mode of the same unified guidance law, not a
      // second controller.  Keeping guidance active avoids the old
      // guidance->handover->guidance command-cycle that repeatedly pushed the
      // camera back across the 8 degree boundary.  The range scheduler has
      // already paused positive closure and the guidance FOV override is 5
      // degrees in this state.
      if (guidance.valid) {
        auto buffered_guidance = guidance;
        if (!depth_visible() && search_visible()) {
          buffered_guidance.line_of_sight_ned = search_los_ned();
          buffered_guidance.allowed_axis_error_rad =
            fov_barrier_.config().normal_boundary_rad;
        }
        publish_attitude_setpoint(
          intercept_range_latched_ ? guidance.acceleration :
          acquisition_limited_acceleration(guidance.acceleration),
          &buffered_guidance);
      } else if (depth_visible()) {
        publish_attitude_setpoint(handover_acceleration(handover_axis()));
      } else {
        publish_attitude_setpoint(braking_acceleration(position_hold_[2]));
      }
      return;
    }
    if (state == PureMissionState::IMM_COAST && guidance.valid &&
      !depth_visible() && search_visible())
    {
      // Preserve IMM range/velocity prediction while the wide search camera
      // corrects the pointing axis. Search bearing buffers depth loss; it does
      // not invent a replacement depth measurement.
      auto buffered_guidance = guidance;
      buffered_guidance.line_of_sight_ned = search_los_ned();
      buffered_guidance.allowed_axis_error_rad =
        fov_barrier_.config().normal_boundary_rad;
      publish_attitude_setpoint(guidance.acceleration, &buffered_guidance);
      return;
    }
    if ((state == PureMissionState::INTERCEPT ||
      state == PureMissionState::SHOT_READY ||
      state == PureMissionState::IMM_COAST ||
      state == PureMissionState::EGRESS_REATTACK) && guidance.valid)
    {
      publish_attitude_setpoint(guidance.acceleration, &guidance);
      return;
    }
    publish_attitude_setpoint(braking_acceleration(position_hold_[2]));
  }

  void publish_vehicle_command(std::uint32_t command, float param1, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand output;
    output.timestamp = timestamp_us();
    output.command = command;
    output.param1 = param1;
    output.param2 = param2;
    output.target_system = static_cast<std::uint8_t>(target_system_);
    output.target_component = 1;
    output.source_system = 255;
    output.source_component = 0;
    output.from_external = true;
    command_pub_->publish(output);
  }

  void manage_offboard_and_arm(bool local_ok)
  {
    if (!auto_engage_) {
      return;
    }
    if (warmup_count_ < warmup_setpoint_count_) {
      ++warmup_count_;
      return;
    }
    const bool status_fresh = vehicle_status_valid_ &&
      message_age(vehicle_status_update_) <= status_timeout_s_;
    const bool armed = status_fresh &&
      vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    if (!armed) {
      const bool preflight_ready = status_fresh &&
        vehicle_status_.pre_flight_checks_pass && local_ok && attitude_fresh();
      if (!preflight_ready) {
        preflight_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        return;
      }
      if (preflight_stable_since_.nanoseconds() == 0) {
        preflight_stable_since_ = now();
      }
      if ((now() - preflight_stable_since_).seconds() < preflight_stability_hold_s_) {
        return;
      }
    }
    if (message_age(last_command_) <= 1.0) {
      return;
    }
    if (vehicle_status_.nav_state !=
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      last_command_ = now();
    } else if (!armed) {
      lock_task_origin("automatic arm command");
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      last_command_ = now();
    }
  }

  void publish_shot_event(const PredictiveShotResult & shot)
  {
    virtual_interception_interfaces::msg::VirtualShotEvent output;
    output.interface_version =
      virtual_interception_interfaces::msg::VirtualShotEvent::INTERFACE_VERSION_V1;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    // capture_count_ is incremented immediately before this publisher is called.
    output.shot_id = capture_count_;
    output.projectile_speed_mps = projectile_speed_mps_;
    output.predicted_time_of_flight_s = shot.time_of_flight_s;
    output.shot_direction_ned.x = shot.shot_direction_ned[0];
    output.shot_direction_ned.y = shot.shot_direction_ned[1];
    output.shot_direction_ned.z = shot.shot_direction_ned[2];
    const Vector pursuer_task_position = task_position();
    output.pursuer_position_ned.x = pursuer_task_position[0];
    output.pursuer_position_ned.y = pursuer_task_position[1];
    output.pursuer_position_ned.z = pursuer_task_position[2];
    output.pursuer_velocity_ned.x = local_velocity_[0];
    output.pursuer_velocity_ned.y = local_velocity_[1];
    output.pursuer_velocity_ned.z = local_velocity_[2];
    output.estimated_target_position_ned.x =
      pursuer_task_position[0] + relative_estimate_.position.x;
    output.estimated_target_position_ned.y =
      pursuer_task_position[1] + relative_estimate_.position.y;
    output.estimated_target_position_ned.z =
      pursuer_task_position[2] + relative_estimate_.position.z;
    const Vector target_velocity{
      local_velocity_[0] + relative_estimate_.velocity.x,
      local_velocity_[1] + relative_estimate_.velocity.y,
      local_velocity_[2] + relative_estimate_.velocity.z};
    output.estimated_target_velocity_ned.x = target_velocity[0];
    output.estimated_target_velocity_ned.y = target_velocity[1];
    output.estimated_target_velocity_ned.z = target_velocity[2];
    const Vector relative_acceleration{
      relative_estimate_.acceleration.x,
      relative_estimate_.acceleration.y,
      relative_estimate_.acceleration.z};
    const Vector target_acceleration =
      finite_vector(relative_acceleration) && finite_vector(local_acceleration_) ?
      add(relative_acceleration, local_acceleration_) : Vector{};
    output.estimated_target_acceleration_ned.x = target_acceleration[0];
    output.estimated_target_acceleration_ned.y = target_acceleration[1];
    output.estimated_target_acceleration_ned.z = target_acceleration[2];
    const double time = shot.time_of_flight_s;
    output.predicted_intercept_point_ned.x =
      output.estimated_target_position_ned.x +
      target_velocity[0] * time + 0.5 * target_acceleration[0] * time * time;
    output.predicted_intercept_point_ned.y =
      output.estimated_target_position_ned.y +
      target_velocity[1] * time + 0.5 * target_acceleration[1] * time * time;
    output.predicted_intercept_point_ned.z =
      output.estimated_target_position_ned.z +
      target_velocity[2] * time + 0.5 * target_acceleration[2] * time * time;
    output.estimated_range_m = shot.range_m;
    output.actual_attack_axis_error_rad = angle_between(
      current_attack_axis_ned(), shot.shot_direction_ned);
    output.predicted_lateral_sigma_m = shot.predicted_lateral_sigma_m;
    output.maximum_position_variance_m2 = std::max({
      relative_estimate_.covariance[0],
      relative_estimate_.covariance[7],
      relative_estimate_.covariance[14]});
    output.solution_valid = shot.valid;
    shot_pub_->publish(output);
  }

  void publish_status(
    const PureMissionResult & mission, const Unified3dGuidanceResult & guidance,
    const PredictiveShotResult & shot,
    bool local_ok, bool attitude_ok, bool capture_gate)
  {
    px4_offboard_tracking::msg::PureInterceptorStatus output;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    output.state = static_cast<std::uint8_t>(mission.state);
    output.state_name = mission.state_name;
    output.transition_reason = mission.transition_reason;
    output.control_enabled = control_enabled_;
    output.auto_engage = auto_engage_;
    output.takeoff_enabled = takeoff_enabled_;
    output.setpoint_streaming = setpoint_streaming();
    output.offboard_session_seen = offboard_session_seen_;
    output.stopped_after_offboard_exit = stopped_after_offboard_exit_;
    output.offboard_active = offboard_active();
    output.armed = armed();
    output.local_state_valid = local_ok;
    output.attitude_valid = attitude_ok;
    output.search_visible = search_visible();
    output.depth_visible = depth_visible();
    output.estimate_valid = estimate_valid();
    output.guidance_valid = guidance.valid;
    output.capture_gate_passed = capture_gate;
    output.tracking_ready = tracking_ready_;
    output.continuous_tracking_duration_s = continuous_tracking_duration_s_;
    output.terminal_alignment_active = terminal_alignment_strength_ > 0.0;
    output.terminal_alignment_strength = terminal_alignment_strength_;
    output.fire_authorized = fire_authorized_;
    output.attack_height_gate_passed = attack_height_gate_passed_;
    output.attack_enable_delay_remaining_s = attack_enable_delay_remaining_s_;
    output.first_attack_tracking_hold_remaining_s =
      first_attack_tracking_hold_remaining_s_;
    output.recoil_active = mission.recoil_active;
    output.reattack_cooldown_active = mission.reattack_cooldown_active;
    output.reattack_cooldown_remaining_s =
      mission.reattack_cooldown_remaining_s;
    output.search_pattern_active = search_pattern_result_.active;
    output.search_commanded_yaw_rad = search_yaw_rad_;
    output.search_pattern_progress = search_pattern_result_.progress;
    output.capture_count = capture_count_;
    output.required_capture_count = mission_required_captures();
    output.attack_complete = attack_complete_;
    output.completion_velocity_braking_active = velocity_braking_state(mission.state);
    output.completion_position_hold_active = completion_position_hold_active_;
    output.safe_to_exit_offboard = safe_to_exit_offboard_;
    output.range_m = guidance.valid ? guidance.range_m :
      std::numeric_limits<double>::quiet_NaN();
    output.closing_speed_mps = guidance.valid ? guidance.closing_speed_mps :
      std::numeric_limits<double>::quiet_NaN();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double actual_attack_axis_error = depth_visible() ?
      std::hypot(
      depth_observation_.horizontal_angle_rad,
      depth_observation_.vertical_angle_rad) : nan;
    const double commanded_attack_axis_error =
      guidance.valid && last_commanded_attack_axis_valid_ ?
      angle_between(guidance.line_of_sight_ned, last_commanded_attack_axis_) : nan;
    const double tracking_error = last_commanded_attack_axis_valid_ ?
      angle_between(current_attack_axis_ned(), last_commanded_attack_axis_) : nan;
    output.attack_axis_error_rad = actual_attack_axis_error;
    output.search_observation_age_s = message_age(search_update_);
    output.depth_observation_age_s = message_age(depth_update_);
    output.estimate_age_s = message_age(estimate_update_);
    output.commanded_attack_axis_error_rad = commanded_attack_axis_error;
    output.actual_attack_axis_error_rad = actual_attack_axis_error;
    output.attack_axis_tracking_error_rad = tracking_error;
    output.estimate_coasting = estimate_usable_for_guidance() && !estimate_valid();
    output.shot_solution_valid = shot.valid;
    output.predicted_shot_time_of_flight_s = shot.valid ? shot.time_of_flight_s : nan;
    output.predicted_shot_lead_angle_rad = shot.valid ? shot.lead_angle_rad : nan;
    output.actual_shot_axis_error_rad = shot.valid ?
      angle_between(current_attack_axis_ned(), shot.shot_direction_ned) : nan;
    output.predicted_shot_lateral_sigma_m =
      shot.valid ? shot.predicted_lateral_sigma_m : nan;
    output.shot_hard_gate_passed = shot_hard_gate_passed_;
    output.shot_timing_gate_passed = shot_timing_result_.ready;
    output.shot_timing_predicted_axis_error_rad =
      shot_timing_result_.valid ?
      shot_timing_result_.predicted_axis_error_rad : nan;
    output.shot_timing_axis_error_rate_rad_s =
      shot_timing_result_.valid ?
      shot_timing_result_.axis_error_rate_rad_s : nan;
    output.shot_timing_score =
      shot_timing_result_.valid ? shot_timing_result_.score : nan;
    output.launch_planner_enabled = launch_position_planner_enabled_;
    output.launch_planner_active = launch_planner_active_;
    output.launch_planner_blend = guidance.valid ?
      guidance.launch_planner_blend : 0.0;
    output.planned_launch_horizon_s = launch_planner_result_.valid ?
      launch_planner_result_.horizon_s : nan;
    output.planned_launch_range_m = launch_planner_result_.valid ?
      launch_planner_result_.launch_range_m : nan;
    output.planned_launch_cost = launch_planner_result_.valid ?
      launch_planner_result_.cost : nan;
    output.planned_acceleration_ned.x = launch_planner_result_.valid ?
      launch_planner_result_.acceleration_ned[0] : nan;
    output.planned_acceleration_ned.y = launch_planner_result_.valid ?
      launch_planner_result_.acceleration_ned[1] : nan;
    output.planned_acceleration_ned.z = launch_planner_result_.valid ?
      launch_planner_result_.acceleration_ned[2] : nan;
    output.planned_relative_velocity_at_launch_ned.x =
      launch_planner_result_.valid ?
      launch_planner_result_.predicted_relative_velocity_ned[0] : nan;
    output.planned_relative_velocity_at_launch_ned.y =
      launch_planner_result_.valid ?
      launch_planner_result_.predicted_relative_velocity_ned[1] : nan;
    output.planned_relative_velocity_at_launch_ned.z =
      launch_planner_result_.valid ?
      launch_planner_result_.predicted_relative_velocity_ned[2] : nan;
    output.radial_control_source =
      static_cast<std::uint8_t>(radial_control_source_);
    output.radial_control_source_name =
      radial_control_source_name(radial_control_source_);
    output.height_m = height_above_task_origin_m(local_position_, local_origin_);
    output.minimum_flight_height_m =
      minimum_height_constraint_.config().minimum_height_m;
    output.minimum_height_constraint_active =
      minimum_height_constraint_active_;
    output.height_recovery_active = height_recovery_active_;
    output.minimum_height_acceleration_limit_down_mps2 =
      minimum_height_acceleration_limit_down_mps2_;
    output.hover_thrust = hover_thrust_;
    output.maximum_thrust = maximum_thrust_;
    output.commanded_thrust = commanded_thrust_;
    output.requested_thrust_before_saturation =
      requested_thrust_before_saturation_;
    output.thrust_saturated = thrust_saturated_;
    output.commanded_acceleration_ned.x = smoothed_acceleration_.ax;
    output.commanded_acceleration_ned.y = smoothed_acceleration_.ay;
    output.commanded_acceleration_ned.z = smoothed_acceleration_.az;
    output.local_velocity_ned.x = local_velocity_[0];
    output.local_velocity_ned.y = local_velocity_[1];
    output.local_velocity_ned.z = local_velocity_[2];
    output.fov_barrier_zone =
      static_cast<std::uint8_t>(fov_barrier_result_.zone);
    output.fov_horizontal_error_rad = depth_visible() ?
      depth_observation_.horizontal_angle_rad : nan;
    output.fov_vertical_error_rad = depth_visible() ?
      depth_observation_.vertical_angle_rad : nan;
    output.fov_predicted_horizontal_error_rad = depth_visible() ?
      fov_barrier_result_.predicted_horizontal_error_rad : nan;
    output.fov_predicted_vertical_error_rad = depth_visible() ?
      fov_barrier_result_.predicted_vertical_error_rad : nan;
    output.fov_predicted_maximum_error_rad = depth_visible() ?
      fov_barrier_result_.predicted_maximum_error_rad : nan;
    // Report the range scheduler action that was actually sent to guidance.
    // The raw barrier request can remain asserted before mission start or
    // while no usable range exists, which is not an approach command pause.
    output.fov_approach_paused =
      range_policy_result_.approach_paused_for_fov;
    output.fov_approach_scale = range_policy_result_.approach_scale;
    output.active_range_reference_m = range_policy_result_.active_reference_m;
    output.active_range_lower_bound_m = range_policy_result_.lower_bound_m;
    output.active_range_upper_bound_m = range_policy_result_.upper_bound_m;
    status_pub_->publish(output);
  }

  std::uint32_t mission_required_captures() const
  {
    return static_cast<std::uint32_t>(
      get_parameter("required_captures").as_int());
  }

  void on_timer()
  {
    const bool local_ok = local_fresh();
    const bool attitude_ok = attitude_fresh();
    if (!local_origin_set_) {
      return;
    }
    update_manual_offboard_session();
    if (local_ok && control_enabled_ && !auto_engage_ &&
      auto_disable_on_offboard_exit_ && !offboard_session_seen_ &&
      !offboard_active())
    {
      // Manual nodes start before takeoff and heartbeat throughout the
      // pilot-controlled climb. Follow the live pose until RC enters
      // Offboard, then freeze this hold before the first control setpoint.
      position_hold_ = local_position_;
      search_yaw_rad_ = current_yaw();
    }
    if (estimate_usable_for_guidance()) {
      const Vector target_velocity{
        local_velocity_[0] + relative_estimate_.velocity.x,
        local_velocity_[1] + relative_estimate_.velocity.y,
        local_velocity_[2] + relative_estimate_.velocity.z};
      if (finite_vector(target_velocity)) {
        last_target_velocity_ = target_velocity;
        last_target_velocity_update_ = now();
        last_target_velocity_valid_ = true;
      }
    }
    if (known_target_direction_enabled_ && search_visible()) {
      known_direction_bearing_seen_ = true;
    }

    const auto state_before = mission_.state();
    update_fov_barrier();
    const double altitude = height_above_task_origin_m(
      local_position_, local_origin_);
    const bool height_recovery_was_active = height_recovery_active_;
    if (armed() && flight_control_active() &&
      altitude < minimum_height_constraint_.config().minimum_height_m)
    {
      height_recovery_active_ = true;
    } else if (height_recovery_active_ &&
      altitude >= minimum_height_constraint_.config().recovery_height_m)
    {
      height_recovery_active_ = false;
    }
    if (height_recovery_active_ != height_recovery_was_active) {
      RCLCPP_WARN(
        get_logger(), "minimum-height recovery %s at %.2f m",
        height_recovery_active_ ? "entered" : "cleared", altitude);
    }
    // Depth-guidance freshness for the first-shot continuity.  The IMM coast
    // path exists precisely to ride through short stereo dropouts, so the
    // ramp must not restart for a gap the coast machinery itself would have
    // bridged: only an outage longer than first_attack_depth_gap_tolerance_s
    // (bounded by prediction_coast_timeout_s) resets the timer.  Without this
    // a recurring >observation_timeout dropout keeps zeroing the ramp and the
    // first shot is postponed indefinitely.
    const bool depth_guidance_ok = depth_visible() && estimate_valid();
    const double continuity_now_s = now().seconds();
    if (depth_guidance_ok) {
      last_depth_guidance_ok_s_ = continuity_now_s;
    }
    const bool depth_guidance_recent = last_depth_guidance_ok_s_ > 0.0 &&
      continuity_now_s - last_depth_guidance_ok_s_ <=
      first_attack_depth_gap_tolerance_s_;
    const bool continuous_first_attack_tracking =
      (state_before == PureMissionState::DEPTH_ACQUIRE ||
      state_before == PureMissionState::INTERCEPT ||
      state_before == PureMissionState::SHOT_READY ||
      state_before == PureMissionState::FOV_RECOVERY ||
      state_before == PureMissionState::IMM_COAST) &&
      (depth_guidance_ok || depth_guidance_recent);
    // Start the smooth terminal-axis ramp as soon as the depth estimate is
    // valid during DEPTH_ACQUIRE.  The mission state still cannot emit a shot
    // before INTERCEPT, so this uses otherwise idle estimator-stability time
    // to remove attitude lag without weakening any firing gate.
    if (capture_count_ == 0) {
      if (!continuous_first_attack_tracking) {
        first_intercept_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      } else if (first_intercept_stable_since_.nanoseconds() == 0) {
        first_intercept_stable_since_ = now();
      }
    }
    const double mission_elapsed = mission_start_.nanoseconds() > 0 ?
      std::max(0.0, (now() - mission_start_).seconds()) : 0.0;
    continuous_tracking_duration_s_ =
      first_intercept_stable_since_.nanoseconds() > 0 ?
      std::max(0.0, (now() - first_intercept_stable_since_).seconds()) : 0.0;
    tracking_ready_ = capture_count_ > 0 ||
      continuous_tracking_duration_s_ >= terminal_alignment_tracking_hold_s_;
    terminal_alignment_strength_ = capture_count_ > 0 ? 1.0 : std::clamp(
      (continuous_tracking_duration_s_ - terminal_alignment_tracking_hold_s_) /
      terminal_alignment_ramp_s_, 0.0, 1.0);
    const bool first_attack_tracking_ready =
      capture_count_ > 0 || first_attack_tracking_hold_s_ == 0.0 ||
      continuous_tracking_duration_s_ >= first_attack_tracking_hold_s_;
    attack_enable_delay_remaining_s_ = std::max(
      0.0, attack_enable_delay_s_ - mission_elapsed);
    first_attack_tracking_hold_remaining_s_ = capture_count_ > 0 ? 0.0 :
      std::max(
      0.0, first_attack_tracking_hold_s_ - continuous_tracking_duration_s_);
    // Tracking and terminal alignment start from sensor readiness, while the
    // fire gate remains an independent mission-time authorization. The ramp
    // prevents a command-axis step when the authorization timer expires.
    attack_height_gate_passed_ =
      armed() && offboard_active() && task_origin_locked_ &&
      altitude >= minimum_height_constraint_.config().minimum_height_m &&
      !height_recovery_active_;
    fire_authorized_ =
      flight_control_active() &&
      attack_height_gate_passed_ &&
      attack_enable_delay_remaining_s_ <= 0.0 &&
      first_attack_tracking_ready &&
      terminal_alignment_strength_ >= 1.0 - 1.0e-6;
    current_shot_solution_ = predictive_shot_solution();
    const auto guidance = guidance_result(terminal_alignment_strength_);
    predictive_aim_active_ =
      terminal_alignment_strength_ > 0.0 &&
      current_shot_solution_.valid &&
      current_shot_solution_.lead_angle_rad <= shot_maximum_lead_angle_rad_;
    shot_hard_gate_passed_ =
      shot_base_gate(guidance, current_shot_solution_);
    const double actual_shot_axis_error =
      current_shot_solution_.valid ?
      angle_between(
      current_attack_axis_ned(),
      current_shot_solution_.shot_direction_ned) :
      std::numeric_limits<double>::quiet_NaN();
    shot_timing_result_ = shot_timing_optimizer_.update({
      shot_hard_gate_passed_, actual_shot_axis_error, now().seconds()});
    const bool capture_gate =
      shot_hard_gate_passed_ && shot_timing_result_.ready;
    // INIT is the acquisition state for the first coherent local-position and
    // attitude samples. Treating that expected startup gap as an in-flight
    // safety failure creates a race into SAFETY_ABORT (and an invalid recovery
    // directly to SEARCH). Safety monitoring is fully active after INIT.
    std::string safety_abort_reason;
    const bool safety_abort =
      state_before != PureMissionState::INIT &&
      state_before != PureMissionState::COMPLETE &&
      state_before != PureMissionState::SAFETY_ABORT &&
      safety_abort_requested(local_ok, attitude_ok, safety_abort_reason);
    if (safety_abort && !safety_abort_latched_) {
      RCLCPP_ERROR(
        get_logger(), "safety abort requested: %s", safety_abort_reason.c_str());
    }
    safety_abort_latched_ = safety_abort;
    const bool height_ready = takeoff_enabled_ ?
      std::abs(altitude - takeoff_height_m_) <= takeoff_tolerance_m_ :
      altitude >= minimum_height_constraint_.config().minimum_height_m;
    const bool takeoff_ready =
      flight_control_active() && offboard_active() && armed() &&
      task_origin_locked_ && height_ready &&
      norm(local_velocity_) <= takeoff_speed_tolerance_mps_;
    if (require_mission_enable_ && takeoff_ready &&
      !pursuer_takeoff_ready_published_ && pursuer_takeoff_ready_pub_)
    {
      std_msgs::msg::Bool ready;
      ready.data = true;
      pursuer_takeoff_ready_pub_->publish(ready);
      pursuer_takeoff_ready_published_ = true;
      RCLCPP_INFO(
        get_logger(),
        "pursuer safe takeoff complete; target trajectory may start");
    }

    PureMissionInput input;
    input.system_ready = local_ok && attitude_ok;
    input.takeoff_ready = takeoff_ready;
    input.mission_enabled = mission_enabled_ && flight_control_active();
    input.search_visible = search_visible();
    input.depth_visible = depth_visible();
    input.depth_admission_ready =
      input.depth_visible && fov_barrier_result_.depth_admission_allowed;
    input.estimate_valid = estimate_usable_for_guidance();
    input.guidance_valid = guidance.valid;
    input.fov_recovery_required =
      fov_barrier_result_.zone == FovBarrierZone::RECOVERY;
    input.fov_hard_violation = input.depth_visible &&
      std::max(
      std::abs(depth_observation_.horizontal_angle_rad),
      std::abs(depth_observation_.vertical_angle_rad)) >=
      fov_barrier_.config().hard_boundary_rad;
    input.fov_recovery_complete =
      fov_barrier_result_.recovery_exit_allowed;
    input.imm_coast_active =
      fov_barrier_result_.zone == FovBarrierZone::IMM_COAST &&
      input.estimate_valid;
    input.attack_enabled = fire_authorized_;
    input.capture_instantaneous_pass = capture_gate;
    input.safety_abort = safety_abort;
    input.recovery_ready = recovery_ready();
    input.search_sequence = search_observation_.sequence;
    input.depth_sequence = depth_observation_.sequence;
    input.capture_count = capture_count_;
    const auto mission = mission_.update(input, now().seconds());

    if (mission.transitioned) {
      RCLCPP_INFO(
        get_logger(), "mission -> %s: %s",
        mission.state_name.c_str(), mission.transition_reason.c_str());
      if (state_before == PureMissionState::SEARCH &&
        mission.state != PureMissionState::SEARCH)
      {
        search_pattern_result_.active = false;
        search_pattern_result_.pending = false;
        search_pattern_result_.completed = true;
      }
      if (mission.state == PureMissionState::SEARCH ||
        mission.state == PureMissionState::COMPLETE ||
        mission.state == PureMissionState::SAFETY_ABORT)
      {
        position_hold_ = local_position_;
        search_yaw_rad_ =
          known_target_direction_enabled_ && !last_target_velocity_valid_ ?
          known_target_bearing_ned_rad_ : current_yaw();
        if (mission.state == PureMissionState::SEARCH) {
          // Give every reacquisition attempt a complete yaw scan while
          // matching the last camera/IMM-derived target velocity. Measuring
          // this window from the last depth sample consumed most of it during
          // egress and handover, so a moving target escaped as SEARCH braked.
          search_reacquisition_started_ = now();
          search_pattern_.reset();
          search_pattern_result_ = {};
          search_pattern_result_.completed = false;
          shot_timing_optimizer_.reset();
          shot_timing_result_ = {};
        }
      }
      if (mission.state == PureMissionState::INTERCEPT) {
        intercept_range_latched_ = true;
      }
      if (mission.state == PureMissionState::COMPLETE) {
        if (completion_started_.nanoseconds() == 0) {
          completion_started_ = now();
        }
      }
      if (mission.state == PureMissionState::EGRESS_REATTACK) {
        // Keep the existing jerk-limited acceleration history. Egress is a
        // signed radial command in the same unified guidance loop.
        position_hold_ = local_position_;
      }
      if (mission.state == PureMissionState::SEARCH ||
        mission.state == PureMissionState::COMPLETE ||
        mission.state == PureMissionState::SAFETY_ABORT)
      {
        // Preserve the acceleration history across airborne perception-state
        // changes. Reset only when entering the first attitude-controlled
        // search after takeoff, or a true safety recovery.
        if (state_before == PureMissionState::TAKEOFF_SAFE ||
          mission.state == PureMissionState::SAFETY_ABORT)
        {
          smoothed_acceleration_initialized_ = false;
        }
        bearing_handover_.reset();
        fov_barrier_.reset();
        if (!intercept_range_latched_) {
          staged_range_policy_.reset();
        }
      }
    }
    if (mission.capture_event && flight_control_active()) {
      ++capture_count_;
      if (enable_native_fire_) {
        publish_shot_event(current_shot_solution_);
      }
      shot_timing_optimizer_.reset();
      shot_timing_result_ = {};
      if (capture_count_ >= mission_required_captures()) {
        attack_complete_ = true;
        completion_started_ = now();
        completion_braking_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        completion_hover_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    }

    if (mission.state == PureMissionState::COMPLETE &&
      !completion_position_hold_active_ &&
      completion_started_.nanoseconds() > 0 &&
      flight_control_active())
    {
      if (norm(local_velocity_) <= completion_position_hold_maximum_speed_mps_) {
        if (completion_braking_stable_since_.nanoseconds() == 0) {
          completion_braking_stable_since_ = now();
        }
      } else {
        completion_braking_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      const bool braking_time_passed =
        (now() - completion_started_).seconds() >= completion_braking_hold_s_;
      const bool braking_stable =
        completion_braking_stable_since_.nanoseconds() > 0 &&
        (now() - completion_braking_stable_since_).seconds() >=
        completion_position_hold_settle_s_;
      if (braking_time_passed && braking_stable) {
        position_hold_ = local_position_;
        completion_position_hold_active_ = true;
        completion_hover_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_INFO(
          get_logger(),
          "completion zero-velocity braking settled; holding current position "
          "[%.2f, %.2f, %.2f]",
          position_hold_[0], position_hold_[1], position_hold_[2]);
      }
    }
    if (mission.state == PureMissionState::COMPLETE &&
      completion_position_hold_active_ && !safe_to_exit_offboard_ &&
      flight_control_active())
    {
      if (norm(local_velocity_) <= completion_safe_hover_maximum_speed_mps_) {
        if (completion_hover_stable_since_.nanoseconds() == 0) {
          completion_hover_stable_since_ = now();
        }
      } else {
        completion_hover_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      if (completion_hover_stable_since_.nanoseconds() > 0 &&
        (now() - completion_hover_stable_since_).seconds() >=
        completion_safe_hover_hold_s_)
      {
        safe_to_exit_offboard_ = true;
        RCLCPP_WARN(
          get_logger(),
          "completion hover stable; safe to exit Offboard manually");
      }
    }

    if (heartbeat_streaming()) {
      publish_offboard_mode(mission.state);
    }
    if (setpoint_streaming()) {
      publish_control_setpoint(mission.state, guidance);
    }
    publish_status(
      mission, guidance, current_shot_solution_,
      local_ok, attitude_ok, capture_gate);
    std_msgs::msg::Bool complete;
    complete.data = safe_to_exit_offboard_;
    completion_pub_->publish(complete);
    manage_offboard_and_arm(local_ok);
  }

  double current_yaw() const
  {
    return std::atan2(
      2.0 * (attitude_.w * attitude_.z + attitude_.x * attitude_.y),
      1.0 - 2.0 * (attitude_.y * attitude_.y + attitude_.z * attitude_.z));
  }

  std::string topic_prefix_;
  std::string search_topic_;
  std::string depth_topic_;
  std::string estimate_topic_;
  std::string status_topic_;
  std::string shot_topic_;
  std::string completion_topic_;
  std::string offboard_session_started_topic_;
  std::string enable_topic_;
  std::string task_origin_topic_;
  std::string common_origin_topic_;
  std::string frame_id_;
  int target_system_;
  bool control_enabled_;
  bool auto_engage_;
  bool takeoff_enabled_;
  bool auto_disable_on_offboard_exit_;
  double offboard_rate_hz_;
  double offboard_warmup_s_;
  double preflight_stability_hold_s_;
  double state_timeout_s_;
  double status_timeout_s_;
  double observation_timeout_s_;
  double estimate_timeout_s_;
  double prediction_coast_timeout_s_;
  double prediction_maximum_position_variance_;
  double attitude_history_s_;
  double attitude_sync_tolerance_s_;
  SampleTimeAttitudeBuffer attitude_history_;
  double takeoff_height_m_;
  double takeoff_tolerance_m_;
  double takeoff_speed_tolerance_mps_;
  double search_yaw_rate_rad_s_;
  double search_prediction_hold_s_;
  bool known_target_direction_enabled_;
  double known_target_bearing_ned_rad_;
  double known_target_search_hold_s_;
  double known_target_search_acceleration_mps2_;
  double known_target_search_acceleration_duration_s_;
  bool known_target_course_enabled_;
  double known_target_course_ned_rad_;
  double known_target_search_velocity_mps_;
  double known_target_velocity_gain_;
  double known_target_course_minimum_speed_fraction_;
  StartupSearch search_pattern_;
  double handover_maximum_tilt_rad_;
  double handover_maximum_acceleration_mps2_;
  double maximum_jerk_mps3_;
  double invalid_input_braking_gain_;
  double maximum_braking_acceleration_mps2_;
  double search_coast_timeout_s_;
  double search_velocity_hold_s_;
  double search_velocity_decay_time_constant_s_;
  double search_velocity_gain_;
  double attack_enable_delay_s_;
  double first_attack_tracking_hold_s_;
  double first_attack_depth_gap_tolerance_s_;
  double terminal_alignment_tracking_hold_s_;
  double terminal_alignment_ramp_s_;
  double capture_minimum_range_m_;
  double capture_maximum_range_m_;
  double capture_maximum_alignment_rad_;
  bool enable_native_fire_;
  double projectile_speed_mps_;
  double shot_maximum_time_of_flight_s_;
  double shot_maximum_lead_angle_rad_;
  double shot_aim_tolerance_rad_;
  double shot_maximum_lateral_sigma_m_;
  double shot_hold_stationary_relative_speed_mps_;
  double recoil_opening_speed_mps_;
  double egress_distance_m_;
  double egress_speed_mps_;
  double safety_minimum_depth_m_;
  MinimumHeightConstraint minimum_height_constraint_;
  double safety_maximum_speed_mps_;
  double safety_maximum_tilt_rad_;
  double recovery_maximum_speed_mps_;
  double recovery_maximum_tilt_rad_;
  double hover_thrust_;
  double maximum_thrust_;
  double maximum_tilt_rad_;
  bool launch_position_planner_enabled_;
  double launch_position_planner_blend_;
  double launch_planner_minimum_geometry_blend_;
  double launch_planner_full_inbound_speed_mps_;
  GeometricControl geometric_control_;
  HorizontalSpeedLimiter command_speed_limiter_;
  VerticalSpeedLimiter vertical_speed_limiter_;
  BearingHandover bearing_handover_;
  PredictiveFovBarrier fov_barrier_;
  StagedRangePolicy staged_range_policy_;
  ManeuverFeedforward maneuver_feedforward_;
  LaunchPositionPlanner launch_position_planner_;
  Unified3dGuidance guidance_;
  PredictiveShotSolver shot_solver_;
  ShotTimingOptimizer shot_timing_optimizer_;
  PureMission mission_;
  double completion_braking_hold_s_;
  double completion_position_hold_maximum_speed_mps_;
  double completion_position_hold_settle_s_;
  double completion_safe_hover_maximum_speed_mps_;
  double completion_safe_hover_hold_s_;
  Vector local_origin_{};
  Vector local_position_{};
  Vector local_position_at_global_{};
  Vector local_velocity_{};
  Vector local_acceleration_{};
  Vector position_hold_{};
  Vector last_target_velocity_{};
  Vector last_commanded_attack_axis_{};
  Quaternion attitude_{};
  anti_drone_interfaces::msg::SearchObservation search_observation_{};
  anti_drone_interfaces::msg::DepthObservation depth_observation_{};
  px4_offboard_tracking::msg::RelativeEstimate relative_estimate_{};
  px4_msgs::msg::VehicleStatus vehicle_status_{};
  GeodeticPosition latest_global_position_{};
  GeodeticPosition common_geodetic_origin_{};
  AccelerationCommand smoothed_acceleration_{};
  PredictiveFovBarrierResult fov_barrier_result_{};
  StagedRangePolicyResult range_policy_result_{};
  PredictiveShotResult current_shot_solution_{};
  StartupSearchResult search_pattern_result_{};
  ShotTimingResult shot_timing_result_{};
  LaunchPositionPlannerResult launch_planner_result_{};
  RadialControlSource radial_control_source_{RadialControlSource::FOV_HOLD};
  bool local_origin_set_{false};
  bool task_origin_locked_{false};
  bool global_valid_{false};
  bool local_at_global_valid_{false};
  bool common_origin_published_{false};
  bool local_reset_counters_initialized_{false};
  bool local_valid_{false};
  bool attitude_valid_{false};
  bool vehicle_status_valid_{false};
  bool smoothed_acceleration_initialized_{false};
  bool handover_using_depth_{false};
  bool safety_abort_latched_{false};
  bool last_target_velocity_valid_{false};
  bool known_direction_bearing_seen_{false};
  bool last_commanded_attack_axis_valid_{false};
  bool predictive_aim_active_{false};
  bool shot_hard_gate_passed_{false};
  bool tracking_ready_{false};
  bool fire_authorized_{false};
  bool attack_height_gate_passed_{false};
  bool attack_complete_{false};
  bool safe_to_exit_offboard_{false};
  bool launch_planner_active_{false};
  bool minimum_height_constraint_active_{false};
  bool height_recovery_active_{false};
  bool intercept_range_latched_{false};
  bool completion_position_hold_active_{false};
  bool thrust_saturated_{false};
  bool require_mission_enable_{false};
  bool mission_enabled_{true};
  bool pursuer_takeoff_ready_published_{false};
  bool offboard_session_seen_{false};
  bool stopped_after_offboard_exit_{false};
  std::string mission_enable_topic_;
  std::string pursuer_takeoff_ready_topic_;
  double search_yaw_rad_{};
  double continuous_tracking_duration_s_{};
  double terminal_alignment_strength_{};
  double attack_enable_delay_remaining_s_{};
  double first_attack_tracking_hold_remaining_s_{};
  double minimum_height_acceleration_limit_down_mps2_{
    std::numeric_limits<double>::quiet_NaN()};
  double commanded_thrust_{};
  double requested_thrust_before_saturation_{};
  std::size_t warmup_count_{0};
  std::size_t warmup_setpoint_count_{0};
  std::uint32_t capture_count_{0};
  std::uint8_t local_xy_reset_counter_{};
  std::uint8_t local_z_reset_counter_{};
  std::chrono::steady_clock::time_point local_update_{};
  std::chrono::steady_clock::time_point attitude_update_{};
  rclcpp::Time vehicle_status_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time search_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time depth_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time estimate_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_velocity_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time search_reacquisition_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_intercept_stable_since_{0, 0, RCL_ROS_TIME};
  double last_depth_guidance_ok_s_{0.0};
  rclcpp::Time completion_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time completion_braking_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time completion_hover_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mission_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time preflight_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_enable_sub_;
  rclcpp::Subscription<anti_drone_interfaces::msg::SearchObservation>::SharedPtr search_sub_;
  rclcpp::Subscription<anti_drone_interfaces::msg::DepthObservation>::SharedPtr depth_sub_;
  rclcpp::Subscription<px4_offboard_tracking::msg::RelativeEstimate>::SharedPtr estimate_sub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::PureInterceptorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<
    virtual_interception_interfaces::msg::VirtualShotEvent>::SharedPtr shot_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr offboard_session_started_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr task_origin_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr common_origin_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pursuer_takeoff_ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::PureInterceptorController>());
  rclcpp::shutdown();
  return 0;
}
