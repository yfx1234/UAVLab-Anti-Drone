#include "netgun_intercept/netgun_fire_control.hpp"

#include <cmath>

namespace netgun_intercept
{

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
    case FireDecisionReason::LATCHED: return "LATCHED";
    case FireDecisionReason::NONE: break;
  }
  return "NONE";
}

}  // namespace netgun_intercept
