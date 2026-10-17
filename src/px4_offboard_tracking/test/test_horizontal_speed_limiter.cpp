#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/horizontal_speed_limiter.hpp"

using px4_offboard_tracking::AccelerationCommand;
using px4_offboard_tracking::HorizontalSpeedLimiter;
using px4_offboard_tracking::HorizontalSpeedLimiterConfig;

TEST(HorizontalSpeedLimiter, LeavesAccelerationBelowSoftZoneUnchanged)
{
  HorizontalSpeedLimiter limiter({30.0, 2.0, 2.0, 6.0});
  const auto result = limiter.apply(AccelerationCommand{3.0, 1.0, 0.0, 0.0}, 20.0, 0.0);
  EXPECT_DOUBLE_EQ(result.acceleration.ax, 3.0);
  EXPECT_DOUBLE_EQ(result.acceleration.ay, 1.0);
  EXPECT_FALSE(result.outward_acceleration_limited);
}

TEST(HorizontalSpeedLimiter, ProgressivelyRemovesOnlyOutwardAcceleration)
{
  HorizontalSpeedLimiter limiter({30.0, 2.0, 2.0, 6.0});
  const auto result = limiter.apply(AccelerationCommand{4.0, 2.0, 0.0, 0.0}, 29.0, 0.0);
  EXPECT_NEAR(result.acceleration.ax, 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.acceleration.ay, 2.0);
  EXPECT_TRUE(result.outward_acceleration_limited);
}

TEST(HorizontalSpeedLimiter, CommandsBoundedBrakingAboveMaximumSpeed)
{
  HorizontalSpeedLimiter limiter({30.0, 2.0, 2.0, 6.0});
  const auto result = limiter.apply(AccelerationCommand{1.0, 2.0, 0.0, 0.0}, 34.0, 0.0);
  EXPECT_NEAR(result.acceleration.ax, -6.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.acceleration.ay, 2.0);
  EXPECT_TRUE(result.overspeed_braking_active);
}

TEST(HorizontalSpeedLimiter, PreservesStrongerExistingBraking)
{
  HorizontalSpeedLimiter limiter({30.0, 2.0, 2.0, 6.0});
  const auto result = limiter.apply(AccelerationCommand{-8.0, 1.0, 0.0, 0.0}, 31.0, 0.0);
  EXPECT_DOUBLE_EQ(result.acceleration.ax, -8.0);
  EXPECT_DOUBLE_EQ(result.acceleration.ay, 1.0);
}

TEST(HorizontalSpeedLimiter, RejectsInvalidConfiguration)
{
  EXPECT_THROW(
    HorizontalSpeedLimiter(HorizontalSpeedLimiterConfig{0.0, 0.5, 2.0, 2.0}),
    std::invalid_argument);
  EXPECT_THROW(
    HorizontalSpeedLimiter(HorizontalSpeedLimiterConfig{4.0, 4.0, 2.0, 2.0}),
    std::invalid_argument);
}
