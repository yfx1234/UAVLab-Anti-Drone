#include "falcon_intercept_control/real_flight_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace falcon_intercept_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite3(const Vector3 & value)
{
  return std::all_of(value.begin(), value.end(), [](double item) {return std::isfinite(item);});
}

double norm3(const Vector3 & value)
{
  return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

Quaterniond normalized(const Quaterniond & input)
{
  const double norm = std::sqrt(
    input.w * input.w + input.x * input.x + input.y * input.y + input.z * input.z);
  if (!std::isfinite(norm) || norm < 1e-9) {throw std::invalid_argument("invalid quaternion");}
  return {input.w / norm, input.x / norm, input.y / norm, input.z / norm};
}

std::array<double, 9> rotation(const Quaterniond & raw)
{
  const auto q = normalized(raw);
  return {
    1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y - q.z * q.w),
    2.0 * (q.x * q.z + q.y * q.w), 2.0 * (q.x * q.y + q.z * q.w),
    1.0 - 2.0 * (q.x * q.x + q.z * q.z), 2.0 * (q.y * q.z - q.x * q.w),
    2.0 * (q.x * q.z - q.y * q.w), 2.0 * (q.y * q.z + q.x * q.w),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
}
}  // namespace

Vector3 optical_to_body_flu(const Vector3 & optical, const CameraToFluTransform & transform)
{
  if (!finite3(optical) || !finite3(transform.camera_translation_body_flu_m)) {
    throw std::invalid_argument("non-finite transform input");
  }
  const auto r = rotation(transform.q_body_flu_to_optical);
  Vector3 output = transform.camera_translation_body_flu_m;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      output[row] += r[column * 3 + row] * optical[column];
    }
  }
  return output;
}

Vector3 flu_to_frd(const Vector3 & flu) {return {flu[0], -flu[1], -flu[2]};}

Vector3 rotate_body_frd_to_ned(const Quaterniond & q, const Vector3 & value)
{
  if (!finite3(value)) {throw std::invalid_argument("non-finite vector");}
  const auto r = rotation(q);
  return {
    r[0] * value[0] + r[1] * value[1] + r[2] * value[2],
    r[3] * value[0] + r[4] * value[1] + r[5] * value[2],
    r[6] * value[0] + r[7] * value[1] + r[8] * value[2]};
}

double wrap_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

RelativeKinematicsResult relative_kinematics(const Vector3 & r, const Vector3 & v)
{
  RelativeKinematicsResult output;
  if (!finite3(r) || !finite3(v)) {return output;}
  const double range2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
  if (range2 < 1e-8) {return output;}
  output.range_m = std::sqrt(range2);
  const double dot = r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
  output.closing_speed_mps = -dot / output.range_m;
  output.los_rate_rad_s = {
    (r[1] * v[2] - r[2] * v[1]) / range2,
    (r[2] * v[0] - r[0] * v[2]) / range2,
    (r[0] * v[1] - r[1] * v[0]) / range2};
  output.los_rate_norm_rad_s = norm3(output.los_rate_rad_s);
  output.relative_speed_mps = norm3(v);
  const double speed2 = output.relative_speed_mps * output.relative_speed_mps;
  output.time_to_closest_approach_s = speed2 > 1e-8 ? std::max(0.0, -dot / speed2) : 0.0;
  Vector3 closest{};
  for (int axis = 0; axis < 3; ++axis) {
    closest[axis] = r[axis] + v[axis] * output.time_to_closest_approach_s;
  }
  // With effectively zero relative speed there is no velocity ray from which to
  // infer a miss. Treat it as zero predicted miss and let range/alignment gates decide.
  output.predicted_miss_distance_m = speed2 > 1e-8 ? norm3(closest) : 0.0;
  output.valid = true;
  return output;
}

SearchController::SearchController(const SearchConfig & config) : config_(config)
{
  if (!std::isfinite(config_.yaw_rate_flu_rps) || config_.yaw_rate_flu_rps == 0.0 ||
    config_.settle_time_s < 0.0 || config_.timeout_s <= 0.0 || config_.cue_gain < 0.0 ||
    config_.maximum_cue_yaw_rate_rps <= 0.0)
  {
    throw std::invalid_argument("invalid search configuration");
  }
}

void SearchController::reset()
{
  started_ = false;
  yaw_initialized_ = false;
  started_s_ = 0.0;
  last_yaw_ned_ = 0.0;
  accumulated_angle_rad_ = 0.0;
  settle_until_s_ = 0.0;
  completed_turns_ = 0;
}

SearchResult SearchController::update(const SearchInput & input)
{
  SearchResult output;
  if (!input.enabled) {reset(); return output;}
  if (!std::isfinite(input.now_s) || !std::isfinite(input.yaw_ned_rad)) {
    output.reason = "invalid search input";
    return output;
  }
  if (!started_) {started_ = true; started_s_ = input.now_s;}
  if (!yaw_initialized_) {
    yaw_initialized_ = true;
    last_yaw_ned_ = input.yaw_ned_rad;
  } else {
    accumulated_angle_rad_ += std::abs(wrap_pi(input.yaw_ned_rad - last_yaw_ned_));
    last_yaw_ned_ = input.yaw_ned_rad;
  }
  const auto measured_turns = static_cast<std::uint32_t>(accumulated_angle_rad_ / (2.0 * kPi));
  if (measured_turns > completed_turns_) {
    completed_turns_ = measured_turns;
    settle_until_s_ = input.now_s + config_.settle_time_s;
  }
  output.completed_turns = completed_turns_;
  output.progress = std::fmod(accumulated_angle_rad_, 2.0 * kPi) / (2.0 * kPi);
  output.timed_out = input.now_s - started_s_ >= config_.timeout_s;
  output.target_cued = input.cue_valid;
  output.valid = input.vehicle_ready && !output.timed_out;
  output.active = output.valid;
  if (output.timed_out) {output.reason = "search timeout"; return output;}
  if (!input.vehicle_ready) {output.reason = "vehicle not ready"; return output;}
  if (input.cue_valid) {
    output.yaw_rate_flu_rps = std::clamp(
      -config_.cue_gain * input.cue_horizontal_error_px,
      -config_.maximum_cue_yaw_rate_rps, config_.maximum_cue_yaw_rate_rps);
    output.reason = "search cue alignment";
  } else if (input.now_s < settle_until_s_) {
    output.settling = true;
    output.reason = "search turn settle";
  } else {
    output.yaw_rate_flu_rps = config_.yaw_rate_flu_rps;
    output.reason = "search scan";
  }
  return output;
}

GuidanceController::GuidanceController(const GuidanceConfig & config) : config_(config)
{
  if (config_.desired_distance_m <= 0.0 || config_.maximum_forward_speed_mps <= 0.0 ||
    config_.maximum_lateral_speed_mps <= 0.0 || config_.maximum_vertical_speed_mps <= 0.0 ||
    config_.maximum_yaw_rate_rps <= 0.0 || config_.maximum_acceleration_mps2 <= 0.0)
  {
    throw std::invalid_argument("invalid guidance configuration");
  }
}

void GuidanceController::reset() {previous_velocity_ = {};}

GuidanceResult GuidanceController::update(
  const Vector3 & position, const Vector3 & velocity, double dt, bool translation_enabled)
{
  GuidanceResult output;
  output.kinematics = relative_kinematics(position, velocity);
  if (!output.kinematics.valid || !std::isfinite(dt) || dt <= 0.0) {return output;}
  const double yaw_error = std::atan2(position[1], std::max(1e-6, position[0]));
  output.yaw_rate_flu_rps = std::clamp(
    config_.yaw_gain * yaw_error, -config_.maximum_yaw_rate_rps, config_.maximum_yaw_rate_rps);
  if (translation_enabled) {
    const double remaining = std::max(0.0, output.kinematics.range_m - config_.desired_distance_m);
    const double scale = remaining / output.kinematics.range_m;
    Vector3 desired{
      config_.position_gain_forward * position[0] * scale + config_.velocity_feedforward * velocity[0],
      config_.position_gain_lateral * position[1] * scale + config_.velocity_feedforward * velocity[1],
      config_.position_gain_vertical * position[2] * scale + config_.velocity_feedforward * velocity[2]};
    desired[0] = std::clamp(desired[0], 0.0, config_.maximum_forward_speed_mps);
    desired[1] = std::clamp(
      desired[1], -config_.maximum_lateral_speed_mps, config_.maximum_lateral_speed_mps);
    desired[2] = std::clamp(
      desired[2], -config_.maximum_vertical_speed_mps, config_.maximum_vertical_speed_mps);
    const double maximum_step = config_.maximum_acceleration_mps2 * dt;
    for (int axis = 0; axis < 3; ++axis) {
      desired[axis] = std::clamp(
        desired[axis], previous_velocity_[axis] - maximum_step,
        previous_velocity_[axis] + maximum_step);
    }
    previous_velocity_ = desired;
    output.velocity_body_flu_mps = desired;
  } else {
    reset();
  }
  output.valid = true;
  return output;
}

QualityResult evaluate_quality(const QualityConfig & config, const QualityInput & input)
{
  QualityResult output;
  output.calibration_ok = input.calibration_ready;
  output.target_fresh = input.target_valid && std::isfinite(input.target_age_s) &&
    input.target_age_s >= 0.0 && input.target_age_s <= config.maximum_target_age_s;
  output.covariance_ok = std::isfinite(input.maximum_position_variance_m2) &&
    input.maximum_position_variance_m2 >= 0.0 &&
    input.maximum_position_variance_m2 <= config.maximum_position_variance_m2;
  output.innovation_ok = std::isfinite(input.innovation_nis) && input.innovation_nis >= 0.0 &&
    input.innovation_nis <= config.maximum_innovation_nis;
  output.px4_state_ok = input.px4_ready;
  output.passed = output.calibration_ok && output.target_fresh && output.covariance_ok &&
    output.innovation_ok && output.px4_state_ok;
  if (!output.calibration_ok) {output.reason = "calibration not ready";}
  else if (!output.target_fresh) {output.reason = "target stale or invalid";}
  else if (!output.covariance_ok) {output.reason = "position covariance rejected";}
  else if (!output.innovation_ok) {output.reason = "innovation rejected";}
  else if (!output.px4_state_ok) {output.reason = "PX4 state not ready";}
  else {output.reason = "tracking quality passed";}
  return output;
}

CaptureWindow::CaptureWindow(const CaptureConfig & config) : config_(config)
{
  if (config_.minimum_range_m <= 0.0 || config_.maximum_range_m <= config_.minimum_range_m ||
    config_.required_hold_time_s < 0.0 || config_.minimum_distinct_frames < 1)
  {
    throw std::invalid_argument("invalid capture configuration");
  }
}

void CaptureWindow::reset_hold()
{
  holding_ = false;
  hold_started_s_ = 0.0;
  observation_initialized_ = false;
  last_observation_stamp_s_ = 0.0;
  last_observation_sequence_ = 0;
  distinct_frames_ = 0;
}

void CaptureWindow::reset()
{
  reset_hold();
  time_initialized_ = false;
  last_now_s_ = 0.0;
}

CaptureResult CaptureWindow::update(const CaptureInput & input)
{
  CaptureResult output;
  if (!std::isfinite(input.now_s) || (time_initialized_ && input.now_s < last_now_s_)) {
    reset();
    return output;
  }
  time_initialized_ = true;
  last_now_s_ = input.now_s;
  output.quality_ok = input.quality_passed && input.observation_valid;
  output.alignment_ok = std::isfinite(input.alignment_error_rad) &&
    input.alignment_error_rad <= config_.maximum_alignment_error_rad;
  const double range_for_gate = std::isfinite(input.range_upper_m) && input.range_upper_m > 0.0 ?
    input.range_upper_m : input.kinematics.range_m;
  output.range_ok = input.kinematics.valid &&
    range_for_gate >= config_.minimum_range_m &&
    range_for_gate <= config_.maximum_range_m;
  output.los_rate_ok = input.kinematics.valid &&
    input.kinematics.los_rate_norm_rad_s <= config_.maximum_los_rate_rad_s;
  output.relative_speed_ok = input.kinematics.valid &&
    input.kinematics.relative_speed_mps <= config_.maximum_relative_speed_mps;
  output.prediction_ok = input.kinematics.valid &&
    input.kinematics.predicted_miss_distance_m <= config_.maximum_predicted_miss_distance_m;
  output.instantaneous_pass = output.quality_ok && output.alignment_ok && output.range_ok &&
    output.los_rate_ok && output.relative_speed_ok && output.prediction_ok;
  if (!output.instantaneous_pass) {reset_hold(); return output;}
  if (observation_initialized_ &&
    (input.observation_stamp_s < last_observation_stamp_s_ ||
    input.observation_sequence < last_observation_sequence_))
  {
    reset_hold();
    output = {};
    return output;
  }
  if (!holding_) {holding_ = true; hold_started_s_ = input.now_s;}
  const bool distinct = !observation_initialized_ ||
    (input.observation_stamp_s > last_observation_stamp_s_ &&
    input.observation_sequence > last_observation_sequence_);
  if (distinct) {
    observation_initialized_ = true;
    last_observation_stamp_s_ = input.observation_stamp_s;
    last_observation_sequence_ = input.observation_sequence;
    ++distinct_frames_;
  }
  output.distinct_frames = distinct_frames_;
  output.hold_elapsed_s = std::max(0.0, input.now_s - hold_started_s_);
  output.frame_count_ok = distinct_frames_ >= config_.minimum_distinct_frames;
  output.held = output.frame_count_ok && output.hold_elapsed_s >= config_.required_hold_time_s;
  return output;
}

double prediction_decay(double age_s, double fresh_s, double stop_s)
{
  if (!std::isfinite(age_s) || stop_s <= fresh_s || age_s >= stop_s) {return 0.0;}
  if (age_s <= fresh_s) {return 1.0;}
  return (stop_s - age_s) / (stop_s - fresh_s);
}

}  // namespace falcon_intercept_control
