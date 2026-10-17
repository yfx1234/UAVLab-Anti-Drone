#include <gtest/gtest.h>

#include "px4_offboard_tracking/staged_range_policy.hpp"

using px4_offboard_tracking::RangeControlStage;
using px4_offboard_tracking::StagedRangePolicy;

TEST(StagedRangePolicy, InitializesAtCurrentInBandRange)
{
  StagedRangePolicy policy;
  const auto result = policy.update(
    RangeControlStage::DEPTH_ACQUIRE, 23.0, 0.02, 1.0);
  EXPECT_DOUBLE_EQ(result.active_reference_m, 23.0);
  EXPECT_DOUBLE_EQ(result.desired_closing_speed_mps, 0.0);
}

TEST(StagedRangePolicy, UsesBandBoundaries)
{
  StagedRangePolicy policy;
  EXPECT_GT(policy.update(
      RangeControlStage::DEPTH_ACQUIRE, 35.0, 0.02, 1.0).
    desired_closing_speed_mps, 0.0);
  EXPECT_LT(policy.update(
      RangeControlStage::DEPTH_ACQUIRE, 18.0, 0.02, 1.0).
    desired_closing_speed_mps, 0.0);
}

TEST(StagedRangePolicy, FovPriorityPausesOnlyApproach)
{
  StagedRangePolicy policy;
  EXPECT_DOUBLE_EQ(policy.update(
      RangeControlStage::INTERCEPT, 32.0, 0.02, 0.0).
    desired_closing_speed_mps, 0.0);
  EXPECT_LT(policy.update(
      RangeControlStage::INTERCEPT, 14.0, 0.02, 0.0).
    desired_closing_speed_mps, 0.0);
}

TEST(StagedRangePolicy, SoftReferenceSlewsWithoutEntryJump)
{
  StagedRangePolicy policy;
  EXPECT_DOUBLE_EQ(policy.update(
      RangeControlStage::INTERCEPT, 23.0, 0.0, 1.0).active_reference_m, 23.0);
  EXPECT_NEAR(policy.update(
      RangeControlStage::INTERCEPT, 23.0, 1.0, 1.0).active_reference_m,
    22.75, 1.0e-12);
}

TEST(StagedRangePolicy, ContinuouslyScalesApproachNearFovBoundary)
{
  StagedRangePolicy full;
  StagedRangePolicy half;
  StagedRangePolicy stopped;
  const double full_speed = full.update(
    RangeControlStage::INTERCEPT, 32.0, 0.02, 1.0).desired_closing_speed_mps;
  const double half_speed = half.update(
    RangeControlStage::INTERCEPT, 32.0, 0.02, 0.5).desired_closing_speed_mps;
  const auto stopped_result = stopped.update(
    RangeControlStage::INTERCEPT, 32.0, 0.02, 0.0);
  EXPECT_GT(full_speed, 0.0);
  EXPECT_NEAR(half_speed, 0.5 * full_speed, 1.0e-12);
  EXPECT_DOUBLE_EQ(stopped_result.desired_closing_speed_mps, 0.0);
  EXPECT_TRUE(stopped_result.approach_paused_for_fov);
}
