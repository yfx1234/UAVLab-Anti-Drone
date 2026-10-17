#include <gtest/gtest.h>

#include "px4_offboard_tracking/relative_kinematics.hpp"

using px4_offboard_tracking::compute_relative_kinematics;

TEST(RelativeKinematics, HeadOnClosingPredictsZeroMiss)
{
  const auto result = compute_relative_kinematics({10.0, 0.0, 0.0}, {-2.0, 0.0, 0.0});
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.range_m, 10.0);
  EXPECT_DOUBLE_EQ(result.closing_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(result.time_to_closest_approach_s, 5.0);
  EXPECT_DOUBLE_EQ(result.predicted_miss_distance_m, 0.0);
  EXPECT_DOUBLE_EQ(result.los_rate_rad_s[2], 0.0);
}

TEST(RelativeKinematics, TangentialMotionProducesLosRate)
{
  const auto result = compute_relative_kinematics({10.0, 0.0, 0.0}, {0.0, 2.0, 0.0});
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.closing_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.los_rate_rad_s[2], 0.2);
  EXPECT_DOUBLE_EQ(result.time_to_closest_approach_s, 0.0);
  EXPECT_DOUBLE_EQ(result.predicted_miss_distance_m, 10.0);
}

TEST(RelativeKinematics, RecedingTargetHasNegativeClosingAndFutureTcpaClampedToZero)
{
  const auto result = compute_relative_kinematics({10.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.closing_speed_mps, -2.0);
  EXPECT_DOUBLE_EQ(result.time_to_closest_approach_s, 0.0);
  EXPECT_DOUBLE_EQ(result.predicted_miss_distance_m, 10.0);
}

TEST(RelativeKinematics, ZeroRangeIsInvalid)
{
  EXPECT_FALSE(compute_relative_kinematics({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}).valid);
}
