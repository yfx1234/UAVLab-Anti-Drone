#include <gtest/gtest.h>

#include "px4_offboard_tracking/minimum_height_constraint.hpp"

using px4_offboard_tracking::MinimumHeightConstraint;
using px4_offboard_tracking::MinimumHeightConstraintConfig;

TEST(MinimumHeightConstraint, DoesNotCreateAFixedHeightAboveTheBoundary)
{
  MinimumHeightConstraint constraint;
  const auto result = constraint.apply({3.4, 0.0, 0.5});
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.acceleration_down_mps2, 0.5);
}

TEST(MinimumHeightConstraint, IntervenesOnlyWhenPredictionCrossesThreeMetres)
{
  MinimumHeightConstraint constraint;
  const auto result = constraint.apply({3.5, 1.0, 0.0});
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.maximum_allowed_acceleration_down_mps2, -1.0, 1e-12);
  EXPECT_NEAR(result.acceleration_down_mps2, -1.0, 1e-12);
}

TEST(MinimumHeightConstraint, PreventsDownwardAccelerationAtTheBoundary)
{
  MinimumHeightConstraint constraint;
  const auto result = constraint.apply({3.0, 0.0, 1.0});
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.acceleration_down_mps2, 0.0, 1e-12);
}

TEST(MinimumHeightConstraint, DoesNotClimbMerelyBecauseHeightIsBelowFiveMetres)
{
  MinimumHeightConstraint constraint;
  const auto result = constraint.apply({4.5, 0.0, 0.5});
  EXPECT_FALSE(result.active);
  EXPECT_NEAR(result.acceleration_down_mps2, 0.5, 1e-12);
}

TEST(MinimumHeightConstraint, CommandsRecoveryBelowTheBoundary)
{
  MinimumHeightConstraint constraint;
  const auto result = constraint.apply({2.5, 0.0, 0.0});
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.acceleration_down_mps2, -0.8, 1e-12);
}

TEST(MinimumHeightConstraint, LimitsMaximumRecoveryAcceleration)
{
  MinimumHeightConstraint constraint(
    MinimumHeightConstraintConfig{3.0, 1.0, 2.5, 3.3, 1.0, 1.0});
  const auto result = constraint.apply({1.0, 3.0, 0.0});
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.acceleration_down_mps2, -2.5, 1e-12);
}
