#pragma once

#include <array>
#include <cstddef>
#include <deque>

namespace px4_offboard_tracking
{

struct EstimatorEvaluationSample
{
  std::array<double, 3> estimated_position{};
  std::array<double, 3> estimated_velocity{};
  std::array<double, 3> truth_position{};
  std::array<double, 3> truth_velocity{};
  std::array<double, 3> innovation{};
  double normalized_innovation_squared{};
  double coast_time_s{};
};

struct EstimatorEvaluationSummary
{
  std::array<double, 3> position_rmse{};
  std::array<double, 3> velocity_rmse{};
  std::array<double, 3> innovation_bias{};
  double position_rmse_norm{};
  double velocity_rmse_norm{};
  double mean_normalized_innovation_squared{};
  double maximum_coast_time_s{};
  std::size_t sample_count{};
};

class EstimatorEvaluation
{
public:
  bool add(const EstimatorEvaluationSample & sample);
  EstimatorEvaluationSummary summary() const;
  void reset();

private:
  std::array<double, 3> position_squared_error_sum_{};
  std::array<double, 3> velocity_squared_error_sum_{};
  std::array<double, 3> innovation_sum_{};
  double nis_sum_{};
  double maximum_coast_time_s_{};
  std::size_t count_{};
};

struct RelativeTruthSample
{
  double timestamp_s{};
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
};

class RelativeTruthBuffer
{
public:
  RelativeTruthBuffer(double history_duration_s, double extrapolation_tolerance_s);

  bool add(const RelativeTruthSample & sample);
  bool sample(double timestamp_s, RelativeTruthSample & output) const;
  std::size_t size() const;

private:
  double history_duration_s_;
  double extrapolation_tolerance_s_;
  std::deque<RelativeTruthSample> samples_;
};

}  // namespace px4_offboard_tracking
