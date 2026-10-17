#include <gtest/gtest.h>

#include "px4_offboard_tracking/relative_ekf.hpp"

using px4_offboard_tracking::RelativeEkf;
using px4_offboard_tracking::RelativeEkfConfig;
using px4_offboard_tracking::RelativeImmEkf;
using px4_offboard_tracking::RelativeImmEkfConfig;

namespace
{
const std::array<double, 9> kMeasurementCovariance{
  0.01, 0.0, 0.0,
  0.0, 0.01, 0.0,
  0.0, 0.0, 0.01};
}  // namespace

TEST(RelativeEkf, InitializesPositionWithZeroVelocity)
{
  RelativeEkf filter(RelativeEkfConfig{});
  EXPECT_TRUE(filter.update({1.0, 2.0, -7.0}, kMeasurementCovariance, 1.0));
  const auto state = filter.state();
  EXPECT_TRUE(state.initialized);
  EXPECT_DOUBLE_EQ(state.position[0], 1.0);
  EXPECT_DOUBLE_EQ(state.position[2], -7.0);
  EXPECT_DOUBLE_EQ(state.velocity[0], 0.0);
  EXPECT_TRUE(filter.valid(1.1, 0.5, 10.0));
}

TEST(RelativeEkf, EstimatesRelativeVelocityFromPositionSequence)
{
  RelativeEkfConfig config;
  config.acceleration_noise_stddev_mps2 = 0.1;
  RelativeEkf filter(config);
  ASSERT_TRUE(filter.update({0.0, 0.0, -7.0}, kMeasurementCovariance, 0.1));
  ASSERT_TRUE(filter.update({1.0, 0.0, -7.0}, kMeasurementCovariance, 1.1));
  ASSERT_TRUE(filter.update({2.0, 0.0, -7.0}, kMeasurementCovariance, 2.1));
  EXPECT_NEAR(filter.state().velocity[0], 1.0, 0.1);
}

TEST(RelativeEkf, EstimatesConstantAccelerationFromPositionSequence)
{
  RelativeEkfConfig config;
  config.acceleration_noise_stddev_mps2 = 0.2;
  config.innovation_gate_mahalanobis = 100.0;
  RelativeEkf filter(config);
  for (int sample = 0; sample <= 40; ++sample) {
    const double time = 0.1 * sample + 0.1;
    const double elapsed = 0.1 * sample;
    ASSERT_TRUE(filter.update(
      {0.5 * elapsed * elapsed, 0.0, -7.0}, kMeasurementCovariance, time));
  }
  EXPECT_NEAR(filter.state().velocity[0], 4.0, 0.15);
  EXPECT_NEAR(filter.state().acceleration[0], 1.0, 0.15);
}

TEST(RelativeEkf, RejectsLargeInnovation)
{
  RelativeEkfConfig config;
  config.innovation_gate_mahalanobis = 9.0;
  RelativeEkf filter(config);
  ASSERT_TRUE(filter.update({0.0, 0.0, -7.0}, kMeasurementCovariance, 1.0));
  EXPECT_FALSE(filter.update({100.0, 0.0, -7.0}, kMeasurementCovariance, 1.1));
  EXPECT_GT(filter.state().innovation_mahalanobis, 9.0);
  EXPECT_FALSE(filter.valid(1.7, 0.5, 10.0));
}

TEST(RelativeEkf, RejectsNonMonotonicTimestamp)
{
  RelativeEkf filter(RelativeEkfConfig{});
  ASSERT_TRUE(filter.update({0.0, 0.0, -7.0}, kMeasurementCovariance, 2.0));
  EXPECT_FALSE(filter.update({0.1, 0.0, -7.0}, kMeasurementCovariance, 2.0));
  EXPECT_FALSE(filter.update({0.1, 0.0, -7.0}, kMeasurementCovariance, 1.9));
}

TEST(RelativeImmEkf, ShiftsProbabilityTowardAccelerationModel)
{
  RelativeImmEkfConfig config;
  config.filter.acceleration_noise_stddev_mps2 = 0.2;
  config.filter.innovation_gate_mahalanobis = 100.0;
  RelativeImmEkf filter(config);
  for (int sample = 0; sample <= 50; ++sample) {
    const double elapsed = 0.1 * sample;
    ASSERT_TRUE(filter.update(
      {0.5 * elapsed * elapsed, 0.0, -7.0}, kMeasurementCovariance, elapsed + 0.1));
  }
  const auto probabilities = filter.model_probabilities();
  EXPECT_GT(probabilities[1], probabilities[0]);
  EXPECT_GT(filter.state().acceleration[0], 0.6);
  EXPECT_LT(filter.state().acceleration[0], 1.1);
}

TEST(RelativeImmEkf, KeepsModelProbabilitiesNormalized)
{
  RelativeImmEkf filter(RelativeImmEkfConfig{});
  ASSERT_TRUE(filter.update({0.0, 0.0, -7.0}, kMeasurementCovariance, 0.1));
  ASSERT_TRUE(filter.update({0.1, 0.0, -7.0}, kMeasurementCovariance, 0.2));
  const auto probabilities = filter.model_probabilities();
  EXPECT_NEAR(probabilities[0] + probabilities[1], 1.0, 1e-12);
  EXPECT_GT(probabilities[0], 0.0);
  EXPECT_GT(probabilities[1], 0.0);
}
