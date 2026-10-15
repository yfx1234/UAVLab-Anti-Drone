#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "px4_offboard_tracking/tracking_quality_gate.hpp"

using px4_offboard_tracking::TrackingQualityGate;
using px4_offboard_tracking::TrackingQualityGateConfig;
using px4_offboard_tracking::TrackingQualityGateInput;

namespace
{
TrackingQualityGateInput nominal_input()
{
  return {true, 0.5, 2.0, true, 0.02, 0.03, false, true};
}
}  // namespace

TEST(TrackingQualityGate, PassesOnlyCompleteFreshQualitySet)
{
  TrackingQualityGate gate(TrackingQualityGateConfig{});
  const auto result = gate.evaluate(nominal_input());
  EXPECT_TRUE(result.passed);
  EXPECT_TRUE(result.timestamp_ok);
}

TEST(TrackingQualityGate, ReportsIndependentFailures)
{
  TrackingQualityGate gate(TrackingQualityGateConfig{});
  auto input = nominal_input();
  input.maximum_position_variance = 5.0;
  input.innovation_mahalanobis = 12.0;
  input.observation_visible = false;
  input.observation_age_s = 0.3;
  input.attitude_saturated = true;
  input.alignment_feasible = false;
  const auto result = gate.evaluate(input);
  EXPECT_FALSE(result.passed);
  EXPECT_FALSE(result.covariance_ok);
  EXPECT_FALSE(result.innovation_ok);
  EXPECT_FALSE(result.visibility_ok);
  EXPECT_FALSE(result.timestamp_ok);
  EXPECT_FALSE(result.attitude_ok);
  EXPECT_FALSE(result.alignment_ok);
}

TEST(TrackingQualityGate, RejectsNonFiniteTimestamp)
{
  TrackingQualityGate gate(TrackingQualityGateConfig{});
  auto input = nominal_input();
  input.estimate_age_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gate.evaluate(input).passed);
}

TEST(TrackingQualityGate, RejectsInvalidConfiguration)
{
  auto config = TrackingQualityGateConfig{};
  config.maximum_estimate_age_s = 0.0;
  EXPECT_THROW({TrackingQualityGate gate(config);}, std::invalid_argument);
}
