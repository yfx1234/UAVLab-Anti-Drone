#include "netgun_intercept/netgun_fire_control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace netgun_intercept
{

RangeLateralSigma sigma_range_lateral(
  const std::array<double, 81> & full_covariance, const double u[3])
{
  double sigma_range2 = 0.0;
  double trace = 0.0;
  for (int row = 0; row < 3; ++row) {
    trace += full_covariance[9 * row + row];
    for (int col = 0; col < 3; ++col) {
      sigma_range2 += u[row] * full_covariance[9 * row + col] * u[col];
    }
  }
  RangeLateralSigma out;
  out.range_m = std::sqrt(std::max(0.0, sigma_range2));
  out.lateral_m = std::sqrt(std::max(0.0, (trace - sigma_range2) / 2.0));
  return out;
}

std::array<double, 3> apply_muzzle_offset(
  const std::array<double, 3> & aim_point_ned,
  const std::array<double, 3> & muzzle_offset_ned)
{
  return {
    aim_point_ned[0] - muzzle_offset_ned[0],
    aim_point_ned[1] - muzzle_offset_ned[1],
    aim_point_ned[2] - muzzle_offset_ned[2]};
}

std::array<double, 3> gun_axis_body(
  double boresight_pitch_rad, double boresight_yaw_rad)
{
  // Rz(yaw)·Ry(pitch)·(0,0,-1)：先抬头/低头（绕 body-y），再水平偏
  //（绕 body-z）。默认 (0,0) → (0,0,-1)，与"枪管同轴光轴"旧口径一致。
  const double sin_pitch = std::sin(boresight_pitch_rad);
  const double cos_pitch = std::cos(boresight_pitch_rad);
  const double sin_yaw = std::sin(boresight_yaw_rad);
  const double cos_yaw = std::cos(boresight_yaw_rad);
  return {-sin_pitch * cos_yaw, -sin_pitch * sin_yaw, -cos_pitch};
}

double tilt_from_vertical_rad(const std::array<double, 3> & axis_ned)
{
  const double norm = std::sqrt(
    axis_ned[0] * axis_ned[0] + axis_ned[1] * axis_ned[1] +
    axis_ned[2] * axis_ned[2]);
  if (!std::isfinite(norm) || norm < 1e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // 竖直向上 = NED (0,0,-1)：cos(tilt) = -z/|v|
  return std::acos(std::max(-1.0, std::min(1.0, -axis_ned[2] / norm)));
}

ManualFireBlock manual_fire_block(const ManualFireGateInput & input)
{
  if (!input.manual_enabled) {return ManualFireBlock::DISABLED;}
  if (input.latched) {return ManualFireBlock::LATCHED;}
  if (!input.fcu_fresh) {return ManualFireBlock::FCU_STALE;}
  if (!input.armed) {return ManualFireBlock::NOT_ARMED;}
  if (!input.height_ok) {return ManualFireBlock::LOW_HEIGHT;}
  if (!input.tilt_ok) {return ManualFireBlock::TILT;}
  if (input.require_authorization &&
    !(input.authorization_passed && input.estimate_valid))
  {
    return ManualFireBlock::NOT_AUTHORIZED;
  }
  return ManualFireBlock::NONE;
}

std::string manual_fire_block_name(ManualFireBlock block)
{
  switch (block) {
    case ManualFireBlock::DISABLED: return "DISABLED";
    case ManualFireBlock::LATCHED: return "LATCHED";
    case ManualFireBlock::FCU_STALE: return "FCU_STALE";
    case ManualFireBlock::NOT_ARMED: return "NOT_ARMED";
    case ManualFireBlock::LOW_HEIGHT: return "LOW_HEIGHT";
    case ManualFireBlock::TILT: return "TILT";
    case ManualFireBlock::NOT_AUTHORIZED: return "NOT_AUTHORIZED";
    case ManualFireBlock::NONE: break;
  }
  return "NONE";
}

NetgunFireControl::NetgunFireControl(const FireControlConfig & config)
: config_(config)
{
}

bool NetgunFireControl::distance_gate_passed(const FireControlInput & input) const
{
  return std::isfinite(input.range_m) &&
         input.range_m >= config_.effective_range_min_m &&
         input.range_m <= config_.effective_range_max_m;
}

FireDecision NetgunFireControl::request_manual()
{
  FireDecision decision;
  if (latched_) {
    decision.reason = FireDecisionReason::LATCHED;
    decision.shot_id = shot_count_;
    return decision;
  }
  latched_ = true;
  ++shot_count_;
  decision.fire = true;
  decision.reason = FireDecisionReason::MANUAL;
  decision.shot_id = shot_count_;
  return decision;
}

FireDecision NetgunFireControl::update(const FireControlInput & input)
{
  FireDecision decision;
  decision.shot_id = shot_count_;

  if (latched_) {
    decision.reason = FireDecisionReason::LATCHED;
    return decision;
  }

  const bool base_ok = input.authorization_passed &&
    input.estimate_valid &&
    input.measurement_quality_ok &&
    distance_gate_passed(input) &&
    std::isfinite(input.axis_error_rad) &&
    input.axis_error_rad >= 0.0 &&
    std::isfinite(input.los_rate_rad_s) &&
    std::abs(input.los_rate_rad_s) <= config_.maximum_los_rate_rad_s;
  if (!std::isfinite(input.now_s) || !base_ok) {
    window_hold_started_s_ = -1.0;
    return decision;
  }

  // 主支：误差 ≤ primary_axis_error_rad 立即开火。
  if (input.axis_error_rad <= config_.primary_axis_error_rad) {
    latched_ = true;
    ++shot_count_;
    decision.fire = true;
    decision.reason = FireDecisionReason::PRIMARY_WINDOW;
    decision.shot_id = shot_count_;
    return decision;
  }

  // 兜底支：primary < 误差 ≤ window 需持续 window_hold_s（中断即重置）。
  if (input.axis_error_rad <= config_.window_axis_error_rad) {
    if (window_hold_started_s_ < 0.0) {
      window_hold_started_s_ = input.now_s;
    }
    decision.window_hold_elapsed_s =
      std::max(0.0, input.now_s - window_hold_started_s_);
    // 浮点边界容差（与上游 capture_window 的 duration_epsilon 同法）。
    constexpr double hold_epsilon_s = 1e-9;
    if (decision.window_hold_elapsed_s + hold_epsilon_s >= config_.window_hold_s) {
      latched_ = true;
      ++shot_count_;
      decision.fire = true;
      decision.reason = FireDecisionReason::WINDOW_HOLD;
      decision.shot_id = shot_count_;
    }
    return decision;
  }

  window_hold_started_s_ = -1.0;

  // 蒙射支（默认关闭，仅预留接口）：预测有效且误差在覆盖余量内，
  // 窗口即将穿出或任务超时 → 一次性预测射击。
  if (config_.blind_shot_enabled &&
    input.axis_error_rad <= config_.blind_shot_max_axis_error_rad)
  {
    const bool exiting_window =
      (input.range_rate_mps > 0.0 &&
      input.range_m + input.range_rate_mps * config_.blind_shot_exit_margin_s >
      config_.effective_range_max_m) ||
      (input.range_rate_mps < 0.0 &&
      input.range_m + input.range_rate_mps * config_.blind_shot_exit_margin_s <
      config_.effective_range_min_m);
    if (exiting_window || input.engagement_timeout) {
      latched_ = true;
      ++shot_count_;
      decision.fire = true;
      decision.reason = input.engagement_timeout
        ? FireDecisionReason::BLIND_SHOT_TIMEOUT
        : FireDecisionReason::BLIND_SHOT_EXIT;
      decision.shot_id = shot_count_;
    }
  }
  return decision;
}

bool NetgunFireControl::latched() const
{
  return latched_;
}

int NetgunFireControl::shot_count() const
{
  return shot_count_;
}

void NetgunFireControl::reset()
{
  latched_ = false;
  shot_count_ = 0;
  window_hold_started_s_ = -1.0;
}

const FireControlConfig & NetgunFireControl::config() const
{
  return config_;
}

std::string NetgunFireControl::reason_name(FireDecisionReason reason)
{
  switch (reason) {
    case FireDecisionReason::PRIMARY_WINDOW: return "PRIMARY_WINDOW";
    case FireDecisionReason::WINDOW_HOLD: return "WINDOW_HOLD";
    case FireDecisionReason::BLIND_SHOT_EXIT: return "BLIND_SHOT_EXIT";
    case FireDecisionReason::BLIND_SHOT_TIMEOUT: return "BLIND_SHOT_TIMEOUT";
    case FireDecisionReason::MANUAL: return "MANUAL";
    case FireDecisionReason::LATCHED: return "LATCHED";
    case FireDecisionReason::NONE: break;
  }
  return "NONE";
}

}  // namespace netgun_intercept
