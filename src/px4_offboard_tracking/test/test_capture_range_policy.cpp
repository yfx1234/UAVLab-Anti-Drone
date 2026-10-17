#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "px4_offboard_tracking/capture_range_policy.hpp"

using px4_offboard_tracking::CaptureRangePolicy;
using px4_offboard_tracking::CaptureRangePolicyConfig;

TEST(CaptureRangePolicy, PreAttackSettlesAtPreferredRange)
{
  CaptureRangePolicy policy(CaptureRangePolicyConfig{});
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(40.0, false), 1.5);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(25.0, false), 0.0);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(10.0, false), -1.5);
}

TEST(CaptureRangePolicy, AttackConvergesToPreferredStandoffWithoutCollisionCourse)
{
  CaptureRangePolicy policy(CaptureRangePolicyConfig{});
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(40.0, true), 4.0);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(35.0, true), 4.0);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(30.0, true), 2.0);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(25.0, true), 0.0);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(20.0, true), -2.0);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(15.0, true), -4.0);
  EXPECT_DOUBLE_EQ(policy.desired_closing_speed(10.0, true), -4.0);
}

TEST(CaptureRangePolicy, ScheduleIsContinuousAcrossIdealWindow)
{
  CaptureRangePolicy policy(CaptureRangePolicyConfig{});
  constexpr double epsilon = 1e-6;
  EXPECT_NEAR(
    policy.desired_closing_speed(15.0 - epsilon, true),
    policy.desired_closing_speed(15.0 + epsilon, true), epsilon);
  EXPECT_NEAR(
    policy.desired_closing_speed(25.0 - epsilon, true),
    policy.desired_closing_speed(25.0 + epsilon, true), epsilon);
  EXPECT_NEAR(
    policy.desired_closing_speed(35.0 - epsilon, true),
    policy.desired_closing_speed(35.0 + epsilon, true), epsilon);
}

TEST(CaptureRangePolicy, InvalidRangeCannotCreateAnUnsafeCommand)
{
  CaptureRangePolicy policy(CaptureRangePolicyConfig{});
  EXPECT_DOUBLE_EQ(
    policy.desired_closing_speed(std::numeric_limits<double>::quiet_NaN(), true), 0.0);
}

TEST(CaptureRangePolicy, RejectsAnInvalidIdealWindow)
{
  CaptureRangePolicyConfig config;
  config.minimum_range_m = 35.0;
  config.maximum_range_m = 15.0;
  EXPECT_THROW((void)CaptureRangePolicy{config}, std::invalid_argument);
}
