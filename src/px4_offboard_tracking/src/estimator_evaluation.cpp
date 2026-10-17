#include "px4_offboard_tracking/estimator_evaluation.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace px4_offboard_tracking
{

bool EstimatorEvaluation::add(const EstimatorEvaluationSample & sample)
{
  if (!std::isfinite(sample.normalized_innovation_squared) ||
    !std::isfinite(sample.coast_time_s) || sample.coast_time_s < 0.0)
  {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(sample.estimated_position[axis]) ||
      !std::isfinite(sample.estimated_velocity[axis]) ||
      !std::isfinite(sample.truth_position[axis]) ||
      !std::isfinite(sample.truth_velocity[axis]) ||
      !std::isfinite(sample.innovation[axis]))
    {
      return false;
    }
    const double position_error =
      sample.estimated_position[axis] - sample.truth_position[axis];
    const double velocity_error =
      sample.estimated_velocity[axis] - sample.truth_velocity[axis];
    position_squared_error_sum_[axis] += position_error * position_error;
    velocity_squared_error_sum_[axis] += velocity_error * velocity_error;
    innovation_sum_[axis] += sample.innovation[axis];
  }
  nis_sum_ += sample.normalized_innovation_squared;
  maximum_coast_time_s_ = std::max(maximum_coast_time_s_, sample.coast_time_s);
  ++count_;
  return true;
}

EstimatorEvaluationSummary EstimatorEvaluation::summary() const
{
  EstimatorEvaluationSummary output;
  output.sample_count = count_;
  output.maximum_coast_time_s = maximum_coast_time_s_;
  if (count_ == 0) {
    return output;
  }
  double position_norm_squared = 0.0;
  double velocity_norm_squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    output.position_rmse[axis] =
      std::sqrt(position_squared_error_sum_[axis] / static_cast<double>(count_));
    output.velocity_rmse[axis] =
      std::sqrt(velocity_squared_error_sum_[axis] / static_cast<double>(count_));
    output.innovation_bias[axis] =
      innovation_sum_[axis] / static_cast<double>(count_);
    position_norm_squared += output.position_rmse[axis] * output.position_rmse[axis];
    velocity_norm_squared += output.velocity_rmse[axis] * output.velocity_rmse[axis];
  }
  output.position_rmse_norm = std::sqrt(position_norm_squared);
  output.velocity_rmse_norm = std::sqrt(velocity_norm_squared);
  output.mean_normalized_innovation_squared = nis_sum_ / static_cast<double>(count_);
  return output;
}

void EstimatorEvaluation::reset()
{
  *this = EstimatorEvaluation{};
}

RelativeTruthBuffer::RelativeTruthBuffer(
  double history_duration_s, double extrapolation_tolerance_s)
: history_duration_s_(history_duration_s),
  extrapolation_tolerance_s_(extrapolation_tolerance_s)
{
  if (!std::isfinite(history_duration_s_) || !std::isfinite(extrapolation_tolerance_s_) ||
    history_duration_s_ <= 0.0 || extrapolation_tolerance_s_ < 0.0)
  {
    throw std::invalid_argument("invalid relative truth buffer configuration");
  }
}

bool RelativeTruthBuffer::add(const RelativeTruthSample & sample)
{
  if (!std::isfinite(sample.timestamp_s)) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(sample.position[axis]) || !std::isfinite(sample.velocity[axis])) {
      return false;
    }
  }
  if (!samples_.empty() && sample.timestamp_s <= samples_.back().timestamp_s) {
    return false;
  }
  samples_.push_back(sample);
  while (!samples_.empty() &&
    sample.timestamp_s - samples_.front().timestamp_s > history_duration_s_)
  {
    samples_.pop_front();
  }
  return true;
}

bool RelativeTruthBuffer::sample(double timestamp_s, RelativeTruthSample & output) const
{
  if (!std::isfinite(timestamp_s) || samples_.empty()) {
    return false;
  }
  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), timestamp_s,
    [](const RelativeTruthSample & sample, double timestamp) {
      return sample.timestamp_s < timestamp;
    });
  if (upper == samples_.begin()) {
    if (upper->timestamp_s - timestamp_s > extrapolation_tolerance_s_) {
      return false;
    }
    output = *upper;
    output.timestamp_s = timestamp_s;
    return true;
  }
  if (upper == samples_.end()) {
    const auto & last = samples_.back();
    const double dt = timestamp_s - last.timestamp_s;
    if (dt > extrapolation_tolerance_s_) {
      return false;
    }
    output = last;
    output.timestamp_s = timestamp_s;
    for (int axis = 0; axis < 3; ++axis) {
      output.position[axis] += output.velocity[axis] * dt;
    }
    return true;
  }
  const auto lower = std::prev(upper);
  const double interval = upper->timestamp_s - lower->timestamp_s;
  const double fraction = interval > 1e-12 ?
    (timestamp_s - lower->timestamp_s) / interval : 0.0;
  output.timestamp_s = timestamp_s;
  for (int axis = 0; axis < 3; ++axis) {
    output.position[axis] =
      lower->position[axis] + fraction * (upper->position[axis] - lower->position[axis]);
    output.velocity[axis] =
      lower->velocity[axis] + fraction * (upper->velocity[axis] - lower->velocity[axis]);
  }
  return true;
}

std::size_t RelativeTruthBuffer::size() const
{
  return samples_.size();
}

}  // namespace px4_offboard_tracking
