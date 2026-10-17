#include <gtest/gtest.h>

#include "px4_offboard_tracking/vertical_speed_limiter.hpp"

using px4_offboard_tracking::VerticalSpeedLimiter;
using px4_offboard_tracking::VerticalSpeedLimiterConfig;

TEST(VerticalSpeedLimiter, PreservesAccelerationBelowSoftZone)
{
  VerticalSpeedLimiter limiter(VerticalSpeedLimiterConfig{});
  EXPECT_DOUBLE_EQ(limiter.apply(0.5, 0.5).acceleration_down_mps2, 0.5);
  EXPECT_DOUBLE_EQ(limiter.apply(-0.5, -0.5).acceleration_down_mps2, -0.5);
}

TEST(VerticalSpeedLimiter, RemovesOutwardAccelerationNearEitherLimit)
{
  VerticalSpeedLimiter limiter(VerticalSpeedLimiterConfig{});
  const auto down = limiter.apply(1.0, 0.9);
  const auto up = limiter.apply(-1.0, -0.9);
  EXPECT_TRUE(down.outward_acceleration_limited);
  EXPECT_TRUE(up.outward_acceleration_limited);
  EXPECT_NEAR(down.acceleration_down_mps2, 0.5, 1.0e-9);
  EXPECT_NEAR(up.acceleration_down_mps2, -0.5, 1.0e-9);
}

TEST(VerticalSpeedLimiter, CommandsOppositeBrakingWhenOverspeed)
{
  VerticalSpeedLimiter limiter(VerticalSpeedLimiterConfig{});
  const auto down = limiter.apply(1.0, 1.5);
  const auto up = limiter.apply(-1.0, -1.5);
  EXPECT_TRUE(down.overspeed_braking_active);
  EXPECT_TRUE(up.overspeed_braking_active);
  EXPECT_NEAR(down.acceleration_down_mps2, -1.0, 1.0e-9);
  EXPECT_NEAR(up.acceleration_down_mps2, 1.0, 1.0e-9);
}
