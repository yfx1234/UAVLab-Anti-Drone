#include "px4_offboard_tracking/bearing_handover.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

using Vector = std::array<double, 3>;

double dot(const Vector & a, const Vector & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector normalized(const Vector & value)
{
  const double length = std::sqrt(dot(value, value));
  if (!std::isfinite(length) || length < 1e-9) {
    return {};
  }
  return {value[0] / length, value[1] / length, value[2] / length};
}

Vector slerp(const Vector & from_input, const Vector & to_input, double fraction)
{
  const auto from = normalized(from_input);
  const auto to = normalized(to_input);
  const double cosine = std::clamp(dot(from, to), -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (angle < 1e-9) {
    return from;
  }
  const double sine = std::sin(angle);
  if (std::abs(sine) < 1e-9) {
    return normalized({
      (1.0 - fraction) * from[0] + fraction * to[0],
      (1.0 - fraction) * from[1] + fraction * to[1],
      (1.0 - fraction) * from[2] + fraction * to[2]});
  }
  const double first = std::sin((1.0 - fraction) * angle) / sine;
  const double second = std::sin(fraction * angle) / sine;
  return normalized({
    first * from[0] + second * to[0],
    first * from[1] + second * to[1],
    first * from[2] + second * to[2]});
}

bool finite(const Vector & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

}  // namespace

BearingHandover::BearingHandover(const BearingHandoverConfig & config)
: config_(config)
{
  if (config_.maximum_axis_rate_rad_s <= 0.0 ||
    config_.los_rate_filter_time_constant_s <= 0.0 ||
    config_.los_rate_lead_s < 0.0 || config_.maximum_sample_interval_s <= 0.0)
  {
    throw std::invalid_argument("invalid bearing handover configuration");
  }
}

BearingHandoverResult BearingHandover::update(const BearingHandoverInput & input)
{
  state_.new_measurement = false;
  if (!input.visible || input.sequence == 0 || !finite(input.measured_los_common) ||
    !finite(input.current_attack_axis_common) || !std::isfinite(input.sample_time_s))
  {
    return state_;
  }
  const Vector los = normalized(input.measured_los_common);
  const Vector attack = normalized(input.current_attack_axis_common);
  if (dot(los, los) < 0.5 || dot(attack, attack) < 0.5) {
    return state_;
  }
  if (initialized_ &&
    (input.sequence == previous_sequence_ || input.sample_time_s <= previous_time_s_))
  {
    return state_;
  }

  if (!initialized_) {
    state_.commanded_axis_common = attack;
    previous_los_ = los;
    previous_time_s_ = input.sample_time_s;
    previous_sequence_ = input.sequence;
    state_.valid = true;
    state_.new_measurement = true;
    initialized_ = true;
    return state_;
  }

  const double dt = input.sample_time_s - previous_time_s_;
  if (dt > config_.maximum_sample_interval_s) {
    previous_los_ = los;
    previous_time_s_ = input.sample_time_s;
    previous_sequence_ = input.sequence;
    state_.commanded_axis_common = attack;
    state_.los_rate_common = {};
    state_.commanded_axis_rate_rad_s = 0.0;
    state_.new_measurement = true;
    return state_;
  }
  const Vector raw_rate{
    (los[0] - previous_los_[0]) / dt,
    (los[1] - previous_los_[1]) / dt,
    (los[2] - previous_los_[2]) / dt};
  const double alpha = dt / (config_.los_rate_filter_time_constant_s + dt);
  for (int axis = 0; axis < 3; ++axis) {
    state_.los_rate_common[axis] +=
      alpha * (raw_rate[axis] - state_.los_rate_common[axis]);
  }
  const Vector lead_los = normalized({
    los[0] + config_.los_rate_lead_s * state_.los_rate_common[0],
    los[1] + config_.los_rate_lead_s * state_.los_rate_common[1],
    los[2] + config_.los_rate_lead_s * state_.los_rate_common[2]});
  const double angle = std::acos(std::clamp(
    dot(normalized(state_.commanded_axis_common), lead_los), -1.0, 1.0));
  const double maximum_step = config_.maximum_axis_rate_rad_s * dt;
  const double fraction = angle > maximum_step && angle > 1e-9 ? maximum_step / angle : 1.0;
  state_.commanded_axis_common =
    slerp(state_.commanded_axis_common, lead_los, fraction);
  state_.commanded_axis_rate_rad_s = angle > 1e-9 ?
    std::min(angle / dt, config_.maximum_axis_rate_rad_s) : 0.0;
  state_.new_measurement = true;
  state_.valid = true;
  previous_los_ = los;
  previous_time_s_ = input.sample_time_s;
  previous_sequence_ = input.sequence;
  return state_;
}

BearingHandoverResult BearingHandover::state() const
{
  return state_;
}

void BearingHandover::reset()
{
  *this = BearingHandover(config_);
}

}  // namespace px4_offboard_tracking
