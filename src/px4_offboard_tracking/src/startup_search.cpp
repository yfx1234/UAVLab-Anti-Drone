#include "px4_offboard_tracking/startup_search.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

StartupSearch::StartupSearch(const StartupSearchConfig & config)
: config_(config), completed_(!config.enabled)
{
  if (!std::isfinite(config_.yaw_rate_rad_s) || config_.yaw_rate_rad_s == 0.0 ||
    !std::isfinite(config_.turns) || config_.turns <= 0.0 ||
    !std::isfinite(config_.settle_time_s) || config_.settle_time_s < 0.0 ||
    !std::isfinite(config_.acquisition_hold_time_s) || config_.acquisition_hold_time_s < 0.0)
  {
    throw std::invalid_argument("invalid startup search configuration");
  }
}

void StartupSearch::reset()
{
  started_ = false;
  completed_ = !config_.enabled;
  completed_by_acquisition_ = false;
  start_time_s_ = 0.0;
  start_yaw_rad_ = 0.0;
  acquisition_started_s_ = 0.0;
  acquisition_pending_ = false;
}

StartupSearchResult StartupSearch::update(
  bool takeoff_ready, bool target_visible, double now_s, double current_yaw_rad)
{
  if (!std::isfinite(now_s) || !std::isfinite(current_yaw_rad)) {
    throw std::invalid_argument("invalid startup search input");
  }
  StartupSearchResult result;
  result.desired_yaw_rad = current_yaw_rad;
  if (!config_.enabled || completed_) {
    result.completed = true;
    result.target_acquired = completed_by_acquisition_;
    return result;
  }
  result.pending = true;
  result.completed = false;
  if (!takeoff_ready) {return result;}

  if (!started_) {
    started_ = true;
    start_time_s_ = now_s;
    start_yaw_rad_ = current_yaw_rad;
  }
  if (config_.stop_on_acquisition && target_visible) {
    if (!acquisition_pending_) {
      acquisition_pending_ = true;
      acquisition_started_s_ = now_s;
    }
    if (now_s - acquisition_started_s_ >= config_.acquisition_hold_time_s) {
      completed_ = true;
      completed_by_acquisition_ = true;
      result.pending = false;
      result.active = false;
      result.completed = true;
      result.target_acquired = true;
      result.progress = std::clamp(
        std::abs(config_.yaw_rate_rad_s) * std::max(0.0, now_s - start_time_s_) /
        (2.0 * kPi * config_.turns), 0.0, 1.0);
      return result;
    }
  } else {
    acquisition_pending_ = false;
  }
  const double sweep_angle = 2.0 * kPi * config_.turns;
  const double sweep_duration = sweep_angle / std::abs(config_.yaw_rate_rad_s);
  const double elapsed = std::max(0.0, now_s - start_time_s_);
  const double sweep_elapsed = config_.repeat_until_acquired ?
    elapsed : std::min(elapsed, sweep_duration);
  result.desired_yaw_rad = start_yaw_rad_ +
    std::copysign(std::abs(config_.yaw_rate_rad_s) * sweep_elapsed, config_.yaw_rate_rad_s);
  result.progress = config_.repeat_until_acquired ?
    std::fmod(sweep_elapsed / sweep_duration, 1.0) :
    std::clamp(sweep_elapsed / sweep_duration, 0.0, 1.0);
  result.active = config_.repeat_until_acquired ||
    elapsed < sweep_duration + config_.settle_time_s;
  if (!result.active) {
    completed_ = true;
    result.pending = false;
    result.completed = true;
    result.progress = 1.0;
  }
  return result;
}

}  // namespace px4_offboard_tracking
