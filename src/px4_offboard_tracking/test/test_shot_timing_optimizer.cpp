#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/shot_timing_optimizer.hpp"

using px4_offboard_tracking::ShotTimingConfig;
using px4_offboard_tracking::ShotTimingInput;
using px4_offboard_tracking::ShotTimingOptimizer;

namespace
{
constexpr double kDegree = 3.14159265358979323846 / 180.0;
}

TEST(ShotTimingOptimizer, RejectsInvalidConfiguration)
{
  ShotTimingConfig config;
  config.preferred_axis_error_rad = 2.0 * kDegree;
  config.hard_axis_error_rad = 1.0 * kDegree;
  EXPECT_THROW(ShotTimingOptimizer optimizer(config), std::invalid_argument);
}

TEST(ShotTimingOptimizer, DoesNotFireAtTwoDegreeHardBoundary)
{
  ShotTimingOptimizer optimizer({});
  const auto result = optimizer.update({true, 1.8 * kDegree, 1.0});
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.ready);
}

TEST(ShotTimingOptimizer, RequiresStableOneDegreeWindow)
{
  ShotTimingConfig config;
  config.minimum_ready_hold_s = 0.04;
  ShotTimingOptimizer optimizer(config);
  EXPECT_FALSE(optimizer.update({true, 0.8 * kDegree, 1.00}).ready);
  EXPECT_FALSE(optimizer.update({true, 0.8 * kDegree, 1.02}).ready);
  EXPECT_TRUE(optimizer.update({true, 0.8 * kDegree, 1.05}).ready);
}

TEST(ShotTimingOptimizer, PredictsLeavingPreferredWindow)
{
  ShotTimingConfig config;
  config.minimum_ready_hold_s = 0.0;
  config.error_rate_filter_time_constant_s = 0.01;
  config.prediction_horizon_s = 0.05;
  ShotTimingOptimizer optimizer(config);
  optimizer.update({true, 0.6 * kDegree, 1.00});
  const auto result = optimizer.update({true, 0.95 * kDegree, 1.02});
  EXPECT_GT(result.predicted_axis_error_rad, 1.0 * kDegree);
  EXPECT_FALSE(result.ready);
}

TEST(ShotTimingOptimizer, ResetsHoldWhenBaseGateFails)
{
  ShotTimingConfig config;
  config.minimum_ready_hold_s = 0.04;
  ShotTimingOptimizer optimizer(config);
  optimizer.update({true, 0.5 * kDegree, 1.00});
  optimizer.update({true, 0.5 * kDegree, 1.03});
  EXPECT_FALSE(optimizer.update({false, 0.5 * kDegree, 1.05}).ready);
  EXPECT_FALSE(optimizer.update({true, 0.5 * kDegree, 1.07}).ready);
}
