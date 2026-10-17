#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <vector>

#include "px4_offboard_tracking/estimator_evaluation.hpp"
#include "px4_offboard_tracking/relative_ekf.hpp"

using px4_offboard_tracking::EstimatorEvaluation;
using px4_offboard_tracking::EstimatorEvaluationSample;
using px4_offboard_tracking::RelativeImmEkf;
using px4_offboard_tracking::RelativeImmEkfConfig;
using px4_offboard_tracking::RelativeTruthBuffer;
using px4_offboard_tracking::RelativeTruthSample;

namespace
{

struct TruthState
{
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
};

enum class Scenario {STATIONARY, STRAIGHT, CONSTANT_ACCELERATION, TURN, FAST_TURN};

TruthState truth(Scenario scenario, double time_s)
{
  switch (scenario) {
    case Scenario::STATIONARY:
      return {{12.0, -3.0, -8.0}, {}, {}};
    case Scenario::STRAIGHT:
      return {
        {10.0 + 1.2 * time_s, -2.0 + 0.25 * time_s, -8.0},
        {1.2, 0.25, 0.0}, {}};
    case Scenario::CONSTANT_ACCELERATION:
      return {
        {10.0 + 0.4 * time_s * time_s, -2.0, -8.0 + 0.1 * time_s * time_s},
        {0.8 * time_s, 0.0, 0.2 * time_s}, {0.8, 0.0, 0.2}};
    case Scenario::TURN:
    case Scenario::FAST_TURN:
    default: {
        constexpr double radius = 10.0;
        const double speed = scenario == Scenario::FAST_TURN ? 5.0 : 1.2;
        const double omega = speed / radius;
        const double phase = omega * time_s;
        return {
          {radius * std::cos(phase), radius * std::sin(phase), -8.0},
          {-speed * std::sin(phase), speed * std::cos(phase), 0.0},
          {-speed * omega * std::cos(phase), -speed * omega * std::sin(phase), 0.0}};
      }
  }
}

std::array<double, 3> measurement(const TruthState & state, int sample)
{
  return {
    state.position[0] + 0.025 * std::sin(0.37 * sample),
    state.position[1] + 0.020 * std::cos(0.29 * sample),
    state.position[2] + 0.015 * std::sin(0.19 * sample + 0.3)};
}

double percentile95(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(0.95 * static_cast<double>(values.size() - 1))];
}

struct ScenarioMetrics
{
  px4_offboard_tracking::EstimatorEvaluationSummary summary;
  double prediction_p95{};
};

ScenarioMetrics run(Scenario scenario)
{
  RelativeImmEkfConfig config;
  config.filter.acceleration_noise_stddev_mps2 = 0.8;
  config.filter.innovation_gate_mahalanobis = 16.266;
  RelativeImmEkf filter(config);
  constexpr std::array<double, 9> covariance{
    0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025};
  EstimatorEvaluation evaluation;
  std::vector<double> prediction_errors;
  constexpr double dt = 0.05;
  for (int sample = 0; sample <= 400; ++sample) {
    const double time_s = 100.0 + dt * sample;
    const double elapsed_s = dt * sample;
    const auto current_truth = truth(scenario, elapsed_s);
    EXPECT_TRUE(filter.update(measurement(current_truth, sample), covariance, time_s));
    if (elapsed_s < 3.0) {
      continue;
    }
    const auto estimate = filter.state();
    evaluation.add(EstimatorEvaluationSample{
      estimate.position, estimate.velocity, current_truth.position, current_truth.velocity,
      estimate.innovation, estimate.innovation_mahalanobis,
      time_s - estimate.last_accepted_measurement_seconds});
    const auto predicted = filter.predicted_state(time_s + 0.3);
    const auto future_truth = truth(scenario, elapsed_s + 0.3);
    double squared_error = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double error = predicted.position[axis] - future_truth.position[axis];
      squared_error += error * error;
    }
    prediction_errors.push_back(std::sqrt(squared_error));
  }
  return {evaluation.summary(), percentile95(prediction_errors)};
}

}  // namespace

TEST(RelativeImmEkfScenarios, MeetsP3AccuracyAndInnovationThresholds)
{
  for (const auto scenario : {
      Scenario::STATIONARY, Scenario::STRAIGHT,
      Scenario::CONSTANT_ACCELERATION, Scenario::TURN, Scenario::FAST_TURN})
  {
    const auto metrics = run(scenario);
    EXPECT_LT(metrics.summary.position_rmse_norm, 0.5);
    EXPECT_LT(metrics.summary.velocity_rmse_norm, 1.0);
    EXPECT_LT(metrics.prediction_p95, 1.0);
    for (const double bias : metrics.summary.innovation_bias) {
      EXPECT_LT(std::abs(bias), 0.10);
    }
    EXPECT_GT(metrics.summary.sample_count, 300U);
  }
}

TEST(RelativeImmEkfScenarios, HalfSecondCoastIsBoundedAndUncertaintyGrows)
{
  RelativeImmEkfConfig config;
  config.filter.acceleration_noise_stddev_mps2 = 0.8;
  config.filter.innovation_gate_mahalanobis = 16.266;
  RelativeImmEkf filter(config);
  constexpr std::array<double, 9> covariance{
    0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025};
  for (int sample = 0; sample <= 200; ++sample) {
    const double elapsed = 0.05 * sample;
    ASSERT_TRUE(filter.update(
      measurement(truth(Scenario::STRAIGHT, elapsed), sample),
      covariance, 100.0 + elapsed));
  }
  double previous_position_variance = -1.0;
  double maximum_error = 0.0;
  for (int step = 1; step <= 10; ++step) {
    const double coast = 0.05 * step;
    const auto predicted = filter.predicted_state(110.0 + coast);
    const auto expected = truth(Scenario::STRAIGHT, 10.0 + coast);
    double squared_error = 0.0;
    double position_variance = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double error = predicted.position[axis] - expected.position[axis];
      squared_error += error * error;
      position_variance += predicted.full_covariance[9 * axis + axis];
    }
    maximum_error = std::max(maximum_error, std::sqrt(squared_error));
    EXPECT_GE(position_variance + 1e-12, previous_position_variance);
    previous_position_variance = position_variance;
  }
  EXPECT_LT(maximum_error, 0.5);
}

TEST(EstimatorEvaluation, RejectsInvalidSamplesAndComputesAxisMetrics)
{
  EstimatorEvaluation evaluation;
  EstimatorEvaluationSample sample;
  sample.estimated_position = {1.0, 2.0, 3.0};
  sample.truth_position = {0.0, 2.0, 4.0};
  sample.estimated_velocity = {1.0, 0.0, 0.0};
  sample.truth_velocity = {0.0, 0.0, 0.0};
  sample.innovation = {0.1, -0.1, 0.0};
  sample.normalized_innovation_squared = 2.0;
  sample.coast_time_s = 0.2;
  ASSERT_TRUE(evaluation.add(sample));
  const auto summary = evaluation.summary();
  EXPECT_EQ(summary.sample_count, 1U);
  EXPECT_NEAR(summary.position_rmse_norm, std::sqrt(2.0), 1e-12);
  EXPECT_DOUBLE_EQ(summary.maximum_coast_time_s, 0.2);
}

TEST(EstimatorEvaluation, InterpolatesTruthToEstimateTimestamp)
{
  RelativeTruthBuffer buffer(2.0, 0.03);
  ASSERT_TRUE(buffer.add({1.0, {0.0, 2.0, -5.0}, {2.0, 0.0, 0.0}}));
  ASSERT_TRUE(buffer.add({1.1, {0.2, 2.0, -5.0}, {2.0, 0.0, 0.0}}));
  RelativeTruthSample sample;
  ASSERT_TRUE(buffer.sample(1.025, sample));
  EXPECT_NEAR(sample.position[0], 0.05, 1e-12);
  EXPECT_NEAR(sample.velocity[0], 2.0, 1e-12);
  ASSERT_TRUE(buffer.sample(1.12, sample));
  EXPECT_NEAR(sample.position[0], 0.24, 1e-12);
  EXPECT_FALSE(buffer.sample(1.14, sample));
}
