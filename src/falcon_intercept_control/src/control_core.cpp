#include "falcon_intercept_control/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace falcon_intercept_control
{
namespace
{

bool finite_vector(const std::array<double, 3> & value)
{
  return std::all_of(value.begin(), value.end(), [](double item) {return std::isfinite(item);});
}

Quaternion normalized(const Quaternion & input)
{
  const double norm = std::sqrt(
    input.w * input.w + input.x * input.x + input.y * input.y + input.z * input.z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("camera quaternion must be finite and non-zero");
  }
  return {input.w / norm, input.x / norm, input.y / norm, input.z / norm};
}

std::array<double, 9> rotation_body_to_optical(const Quaternion & raw)
{
  const auto q = normalized(raw);
  return {
    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    2.0 * (q.x * q.y - q.z * q.w),
    2.0 * (q.x * q.z + q.y * q.w),
    2.0 * (q.x * q.y + q.z * q.w),
    1.0 - 2.0 * (q.x * q.x + q.z * q.z),
    2.0 * (q.y * q.z - q.x * q.w),
    2.0 * (q.x * q.z - q.y * q.w),
    2.0 * (q.y * q.z + q.x * q.w),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
}

double vector_norm(const std::array<double, 3> & value)
{
  return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

}  // namespace

std::array<double, 3> optical_position_to_body_frd(
  const std::array<double, 3> & position_optical_m,
  const RigidTransform & transform)
{
  if (!finite_vector(position_optical_m) ||
    !finite_vector(transform.camera_translation_body_frd_m))
  {
    throw std::invalid_argument("position and translation must be finite");
  }
  const auto rotation = rotation_body_to_optical(transform.q_body_to_optical);
  // p_body = t_body_camera + R_body_optical * p_optical = t + R^T * p_optical.
  std::array<double, 3> result{};
  for (int row = 0; row < 3; ++row) {
    result[row] = transform.camera_translation_body_frd_m[row];
    for (int column = 0; column < 3; ++column) {
      result[row] += rotation[column * 3 + row] * position_optical_m[column];
    }
  }
  return result;
}

std::array<double, 9> optical_covariance_to_body_frd(
  const std::array<double, 9> & covariance_optical,
  const RigidTransform & transform)
{
  if (!std::all_of(
      covariance_optical.begin(), covariance_optical.end(),
      [](double item) {return std::isfinite(item);}))
  {
    throw std::invalid_argument("covariance must be finite");
  }
  const auto rotation = rotation_body_to_optical(transform.q_body_to_optical);
  std::array<double, 9> output{};
  // C_body = R^T C_optical R.
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int left = 0; left < 3; ++left) {
        for (int right = 0; right < 3; ++right) {
          output[row * 3 + column] += rotation[left * 3 + row] *
            covariance_optical[left * 3 + right] * rotation[right * 3 + column];
        }
      }
    }
  }
  return output;
}

RelativeKinematics compute_relative_kinematics(
  const std::array<double, 3> & relative_position_m,
  const std::array<double, 3> & relative_velocity_mps)
{
  // Adapted from the validated pure-numeric anti-drone implementation.
  RelativeKinematics output;
  if (!finite_vector(relative_position_m) || !finite_vector(relative_velocity_mps)) {
    return output;
  }
  const auto & r = relative_position_m;
  const auto & v = relative_velocity_mps;
  const double range_squared = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
  if (range_squared < 1e-8) {return output;}
  output.range_m = std::sqrt(range_squared);
  const double radial_dot = r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
  output.closing_speed_mps = -radial_dot / output.range_m;
  output.los_rate_rad_s = {
    (r[1] * v[2] - r[2] * v[1]) / range_squared,
    (r[2] * v[0] - r[0] * v[2]) / range_squared,
    (r[0] * v[1] - r[1] * v[0]) / range_squared};
  const double speed_squared = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  output.time_to_closest_approach_s = speed_squared > 1e-8 ?
    std::max(0.0, -radial_dot / speed_squared) : 0.0;
  std::array<double, 3> closest{};
  for (int axis = 0; axis < 3; ++axis) {
    closest[axis] = r[axis] + v[axis] * output.time_to_closest_approach_s;
  }
  output.predicted_miss_distance_m = vector_norm(closest);
  output.valid = true;
  return output;
}

ControlCore::ControlCore(const ControlConfig & config)
: config_(config)
{
  if (config_.target_timeout_s <= 0.0 || config_.maximum_position_variance_m2 <= 0.0 ||
    config_.minimum_confidence < 0.0 || config_.minimum_confidence > 1.0 ||
    config_.fire_distance_m <= config_.minimum_safe_range_m ||
    config_.center_threshold_px < 0.0 || config_.fire_hold_frames < 1 ||
    config_.track_confirm_frames < 1 || config_.land_hold_s < 0.0 ||
    config_.position_gain_xy < 0.0 || config_.position_gain_z < 0.0 ||
    config_.maximum_speed_xy_mps <= 0.0 || config_.maximum_speed_z_mps <= 0.0 ||
    config_.maximum_search_yaw_rate_rps <= 0.0 ||
    (config_.startup_search_enabled &&
    (!std::isfinite(config_.startup_search_yaw_rate_rps) ||
    config_.startup_search_yaw_rate_rps == 0.0 ||
    !std::isfinite(config_.startup_search_turns) || config_.startup_search_turns <= 0.0 ||
    !std::isfinite(config_.startup_search_settle_time_s) ||
    config_.startup_search_settle_time_s < 0.0)))
  {
    throw std::invalid_argument("invalid control configuration");
  }
}

void ControlCore::reset()
{
  state_ = MissionState::INIT;
  state_entered_s_ = 0.0;
  mission_started_s_ = 0.0;
  mission_time_initialized_ = false;
  track_frames_ = 0;
  fire_frames_ = 0;
  fire_sent_ = false;
  startup_search_started_ = false;
  startup_search_completed_ = !config_.startup_search_enabled;
  startup_search_started_s_ = 0.0;
  reason_ = "reset";
}

const char * ControlCore::state_name(MissionState state)
{
  switch (state) {
    case MissionState::INIT: return "INIT";
    case MissionState::SEARCH: return "SEARCH";
    case MissionState::TRACK: return "TRACK";
    case MissionState::APPROACH: return "APPROACH";
    case MissionState::FIRE: return "FIRE";
    case MissionState::LAND: return "LAND";
    case MissionState::DONE: return "DONE";
    case MissionState::ABORT: return "ABORT";
  }
  return "UNKNOWN";
}

void ControlCore::transition(MissionState next, const char * reason, double now_s)
{
  if (state_ == next) {return;}
  state_ = next;
  state_entered_s_ = now_s;
  reason_ = reason;
  if (next == MissionState::SEARCH) {
    track_frames_ = 0;
    fire_frames_ = 0;
  }
  if (next != MissionState::APPROACH) {fire_frames_ = 0;}
}

bool ControlCore::quality_passes(const ControlInput & input) const
{
  return input.target_valid && input.health_valid && finite_vector(input.target_position_body_frd_m) &&
    std::isfinite(input.target_age_s) && input.target_age_s >= 0.0 &&
    input.target_age_s <= config_.target_timeout_s && std::isfinite(input.confidence) &&
    input.confidence >= config_.minimum_confidence &&
    std::isfinite(input.maximum_position_variance_m2) &&
    input.maximum_position_variance_m2 >= 0.0 &&
    input.maximum_position_variance_m2 <= config_.maximum_position_variance_m2;
}

std::array<double, 3> ControlCore::approach_velocity(const ControlInput & input) const
{
  const double range = vector_norm(input.target_position_body_frd_m);
  if (!std::isfinite(range) || range <= config_.fire_distance_m) {return {};}
  const double remaining_scale = (range - config_.fire_distance_m) / range;
  std::array<double, 3> output{
    config_.position_gain_xy * input.target_position_body_frd_m[0] * remaining_scale,
    config_.position_gain_xy * input.target_position_body_frd_m[1] * remaining_scale,
    config_.position_gain_z * input.target_position_body_frd_m[2] * remaining_scale};
  const double horizontal = std::hypot(output[0], output[1]);
  if (horizontal > config_.maximum_speed_xy_mps) {
    const double scale = config_.maximum_speed_xy_mps / horizontal;
    output[0] *= scale;
    output[1] *= scale;
  }
  output[2] = std::clamp(
    output[2], -config_.maximum_speed_z_mps, config_.maximum_speed_z_mps);
  return output;
}

bool ControlCore::startup_search_pending(double now_s)
{
  if (!config_.startup_search_enabled || startup_search_completed_) {return false;}
  if (!startup_search_started_) {
    startup_search_started_ = true;
    startup_search_started_s_ = now_s;
  }
  const double sweep_duration =
    2.0 * 3.14159265358979323846 * config_.startup_search_turns /
    std::abs(config_.startup_search_yaw_rate_rps);
  const double elapsed = std::max(0.0, now_s - startup_search_started_s_);
  if (elapsed >= sweep_duration + config_.startup_search_settle_time_s) {
    startup_search_completed_ = true;
    return false;
  }
  return true;
}

double ControlCore::startup_search_yaw_rate(double now_s) const
{
  if (!startup_search_started_ || startup_search_completed_) {return 0.0;}
  const double sweep_duration =
    2.0 * 3.14159265358979323846 * config_.startup_search_turns /
    std::abs(config_.startup_search_yaw_rate_rps);
  return now_s - startup_search_started_s_ < sweep_duration ?
    config_.startup_search_yaw_rate_rps : 0.0;
}

ControlOutput ControlCore::update(const ControlInput & input)
{
  ControlOutput output;
  if (!std::isfinite(input.now_s)) {
    transition(MissionState::ABORT, "non-finite control timestamp", state_entered_s_);
  }
  if (input.abort_requested) {
    transition(MissionState::ABORT, "manual abort", input.now_s);
  }
  if (input.mission_enabled && !mission_time_initialized_) {
    mission_started_s_ = input.now_s;
    mission_time_initialized_ = true;
  }
  if (mission_time_initialized_ && config_.max_task_time_s > 0.0 &&
    input.now_s - mission_started_s_ > config_.max_task_time_s &&
    state_ != MissionState::LAND && state_ != MissionState::DONE && state_ != MissionState::ABORT)
  {
    transition(MissionState::ABORT, "maximum task time exceeded", input.now_s);
  }

  const bool target_quality = quality_passes(input);
  const auto kinematics = compute_relative_kinematics(
    input.target_position_body_frd_m, input.target_velocity_body_frd_mps);

  switch (state_) {
    case MissionState::INIT:
      if (input.mission_enabled) {transition(MissionState::SEARCH, "mission enabled", input.now_s);}
      break;
    case MissionState::SEARCH:
      if (startup_search_pending(input.now_s)) {
        // Keep translation at zero and do not hand over to tracking until the
        // configured one-shot sweep and settle interval have completed.
      } else if (target_quality) {
        track_frames_ = 1;
        transition(MissionState::TRACK, "valid target acquired", input.now_s);
      }
      break;
    case MissionState::TRACK:
      if (!target_quality) {
        transition(MissionState::SEARCH, "target quality lost", input.now_s);
      } else if (++track_frames_ >= config_.track_confirm_frames) {
        transition(MissionState::APPROACH, "target confirmed", input.now_s);
      }
      break;
    case MissionState::APPROACH:
      if (!target_quality) {
        transition(MissionState::SEARCH, "target quality lost", input.now_s);
      } else if (kinematics.valid && kinematics.range_m < config_.minimum_safe_range_m) {
        transition(MissionState::ABORT, "minimum safe range violated", input.now_s);
      } else if (kinematics.valid && kinematics.range_m <= config_.fire_distance_m &&
        input.left_centered && input.right_centered)
      {
        ++fire_frames_;
        if (fire_frames_ >= config_.fire_hold_frames) {
          transition(MissionState::FIRE, "fire window held", input.now_s);
        }
      } else {
        fire_frames_ = 0;
      }
      break;
    case MissionState::FIRE:
      // Keep FIRE observable for one timer period, then use PX4 native Land.
      if (fire_sent_) {transition(MissionState::LAND, "fire event published", input.now_s);}
      break;
    case MissionState::LAND:
      if (input.landed && input.now_s - state_entered_s_ >= config_.land_hold_s) {
        transition(MissionState::DONE, "vehicle landed", input.now_s);
      }
      break;
    case MissionState::DONE:
    case MissionState::ABORT:
      break;
  }

  output.state = state_;
  output.reason = reason_;
  output.kinematics = kinematics;
  output.land_request = state_ == MissionState::LAND;
  if (state_ == MissionState::SEARCH) {
    const bool startup_pending = startup_search_pending(input.now_s);
    const double desired = startup_pending ? startup_search_yaw_rate(input.now_s) :
      (input.search_cue_valid ? config_.search_yaw_gain * input.search_horizontal_error_px :
      config_.scan_yaw_rate_rps);
    output.yaw_rate_rps = std::clamp(
      desired, -config_.maximum_search_yaw_rate_rps, config_.maximum_search_yaw_rate_rps);
    output.valid = input.mission_enabled;
    if (startup_pending) {
      output.reason = std::abs(output.yaw_rate_rps) > 0.0 ?
        "startup search sweep" : "startup search settle";
    }
  } else if (state_ == MissionState::APPROACH && target_quality) {
    output.velocity_body_frd_mps = approach_velocity(input);
    output.valid = true;
  } else if (state_ == MissionState::TRACK || state_ == MissionState::FIRE ||
    state_ == MissionState::LAND)
  {
    output.valid = input.mission_enabled;
  }
  if (state_ == MissionState::FIRE && !fire_sent_) {
    output.fire_event = true;
    fire_sent_ = true;
  }
  if (state_ == MissionState::ABORT || state_ == MissionState::DONE ||
    state_ == MissionState::INIT)
  {
    output.valid = false;
    output.velocity_body_frd_mps = {};
    output.yaw_rate_rps = 0.0;
  }
  return output;
}

}  // namespace falcon_intercept_control
