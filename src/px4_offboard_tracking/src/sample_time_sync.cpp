#include "px4_offboard_tracking/sample_time_sync.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace px4_offboard_tracking
{

SampleTimeAttitudeBuffer::SampleTimeAttitudeBuffer(
  double history_duration_s, double synchronization_tolerance_s)
: history_duration_s_(history_duration_s),
  synchronization_tolerance_s_(synchronization_tolerance_s)
{
  if (!std::isfinite(history_duration_s_) || !std::isfinite(synchronization_tolerance_s_) ||
    history_duration_s_ <= 0.0 || synchronization_tolerance_s_ <= 0.0 ||
    history_duration_s_ <= synchronization_tolerance_s_)
  {
    throw std::invalid_argument("invalid sample-time attitude buffer configuration");
  }
}

Quaterniond SampleTimeAttitudeBuffer::normalized(const Quaterniond & quaternion)
{
  const double norm = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("invalid attitude quaternion");
  }
  return {
    quaternion.w / norm, quaternion.x / norm, quaternion.y / norm, quaternion.z / norm};
}

Quaterniond SampleTimeAttitudeBuffer::slerp(
  const Quaterniond & first_input, const Quaterniond & second_input, double fraction)
{
  const auto first = normalized(first_input);
  auto second = normalized(second_input);
  double dot = first.w * second.w + first.x * second.x +
    first.y * second.y + first.z * second.z;
  if (dot < 0.0) {
    second = {-second.w, -second.x, -second.y, -second.z};
    dot = -dot;
  }
  dot = std::clamp(dot, -1.0, 1.0);
  fraction = std::clamp(fraction, 0.0, 1.0);
  if (dot > 0.9995) {
    return normalized({
      first.w + fraction * (second.w - first.w),
      first.x + fraction * (second.x - first.x),
      first.y + fraction * (second.y - first.y),
      first.z + fraction * (second.z - first.z)});
  }
  const double angle = std::acos(dot);
  const double denominator = std::sin(angle);
  const double first_weight = std::sin((1.0 - fraction) * angle) / denominator;
  const double second_weight = std::sin(fraction * angle) / denominator;
  return normalized({
    first_weight * first.w + second_weight * second.w,
    first_weight * first.x + second_weight * second.x,
    first_weight * first.y + second_weight * second.y,
    first_weight * first.z + second_weight * second.z});
}

bool SampleTimeAttitudeBuffer::add(
  double timestamp_s, const Quaterniond & q_body_to_ned)
{
  if (!std::isfinite(timestamp_s)) {
    return false;
  }
  Quaterniond normalized_attitude;
  try {
    normalized_attitude = normalized(q_body_to_ned);
  } catch (const std::invalid_argument &) {
    return false;
  }
  if (!samples_.empty() && timestamp_s < samples_.back().timestamp_s - 1e-9) {
    return false;
  }
  if (!samples_.empty() && std::abs(timestamp_s - samples_.back().timestamp_s) <= 1e-9) {
    samples_.back().q_body_to_ned = normalized_attitude;
    return true;
  }
  samples_.push_back({timestamp_s, normalized_attitude});
  while (!samples_.empty() &&
    timestamp_s - samples_.front().timestamp_s > history_duration_s_)
  {
    samples_.pop_front();
  }
  return true;
}

bool SampleTimeAttitudeBuffer::sample(
  double timestamp_s, Quaterniond & q_body_to_ned) const
{
  if (!std::isfinite(timestamp_s) || samples_.empty()) {
    return false;
  }
  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), timestamp_s,
    [](const Sample & sample, double timestamp) {
      return sample.timestamp_s < timestamp;
    });
  if (upper == samples_.begin()) {
    if (std::abs(upper->timestamp_s - timestamp_s) > synchronization_tolerance_s_) {
      return false;
    }
    q_body_to_ned = upper->q_body_to_ned;
    return true;
  }
  if (upper == samples_.end()) {
    const auto & last = samples_.back();
    if (std::abs(last.timestamp_s - timestamp_s) > synchronization_tolerance_s_) {
      return false;
    }
    q_body_to_ned = last.q_body_to_ned;
    return true;
  }
  const auto lower = std::prev(upper);
  const double lower_age = timestamp_s - lower->timestamp_s;
  const double upper_age = upper->timestamp_s - timestamp_s;
  if (lower_age > synchronization_tolerance_s_ ||
    upper_age > synchronization_tolerance_s_)
  {
    return false;
  }
  const double interval = upper->timestamp_s - lower->timestamp_s;
  q_body_to_ned = interval <= 1e-9 ?
    lower->q_body_to_ned :
    slerp(lower->q_body_to_ned, upper->q_body_to_ned, lower_age / interval);
  return true;
}

void SampleTimeAttitudeBuffer::clear()
{
  samples_.clear();
}

std::size_t SampleTimeAttitudeBuffer::size() const
{
  return samples_.size();
}

double SampleTimeAttitudeBuffer::px4_sample_or_receipt_time(
  std::uint64_t timestamp_sample_us, double receipt_time_s,
  double maximum_clock_domain_difference_s)
{
  if (!std::isfinite(receipt_time_s) ||
    !std::isfinite(maximum_clock_domain_difference_s) ||
    maximum_clock_domain_difference_s <= 0.0)
  {
    throw std::invalid_argument("invalid timestamp clock-domain configuration");
  }
  const double candidate = static_cast<double>(timestamp_sample_us) * 1e-6;
  if (timestamp_sample_us > 0 && std::isfinite(candidate) &&
    std::abs(candidate - receipt_time_s) <= maximum_clock_domain_difference_s)
  {
    return candidate;
  }
  return receipt_time_s;
}

double SampleTimeAttitudeBuffer::px4_sample_or_receipt_time(
  std::uint64_t timestamp_us, std::uint64_t timestamp_sample_us,
  double receipt_time_s, double maximum_clock_domain_difference_s)
{
  const double direct = px4_sample_or_receipt_time(
    timestamp_sample_us, receipt_time_s, maximum_clock_domain_difference_s);
  if (direct != receipt_time_s || timestamp_sample_us == 0 || timestamp_us == 0) {
    return direct;
  }
  const double sample_offset_s =
    (static_cast<double>(timestamp_sample_us) - static_cast<double>(timestamp_us)) * 1e-6;
  if (std::isfinite(sample_offset_s) &&
    std::abs(sample_offset_s) <= maximum_clock_domain_difference_s)
  {
    return receipt_time_s + sample_offset_s;
  }
  return receipt_time_s;
}

}  // namespace px4_offboard_tracking
