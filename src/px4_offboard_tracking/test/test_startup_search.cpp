#include <gtest/gtest.h>

#include <cmath>

#include "px4_offboard_tracking/startup_search.hpp"

using px4_offboard_tracking::StartupSearch;
using px4_offboard_tracking::StartupSearchConfig;

constexpr double kPi = 3.14159265358979323846;

TEST(StartupSearch, DisabledStageIsTransparent)
{
  StartupSearch search({false, 0.5, 1.0, 0.5, true, true, 0.2});
  const auto result = search.update(false, false, 0.0, 1.2);
  EXPECT_TRUE(result.completed);
  EXPECT_FALSE(result.pending);
  EXPECT_DOUBLE_EQ(result.desired_yaw_rad, 1.2);
}

TEST(StartupSearch, WaitsForTakeoffThenCompletesSweepAndSettle)
{
  StartupSearch search({true, 1.0, 1.0, 0.5, false, false, 0.2});
  EXPECT_TRUE(search.update(false, false, 5.0, 0.3).pending);
  const auto start = search.update(true, false, 10.0, 0.3);
  EXPECT_TRUE(start.active);
  EXPECT_NEAR(start.desired_yaw_rad, 0.3, 1e-9);
  const auto half = search.update(true, false, 10.0 + kPi, 0.0);
  EXPECT_NEAR(half.desired_yaw_rad, 0.3 + kPi, 1e-9);
  EXPECT_NEAR(half.progress, 0.5, 1e-9);
  EXPECT_TRUE(search.update(true, false, 10.0 + 2.0 * kPi + 0.25, 0.0).active);
  const auto done = search.update(true, false, 10.0 + 2.0 * kPi + 0.51, 0.0);
  EXPECT_TRUE(done.completed);
  EXPECT_FALSE(done.active);
}

TEST(StartupSearch, SupportsClockwiseSearch)
{
  StartupSearch search({true, -0.5, 0.5, 0.0, false, false, 0.2});
  search.update(true, false, 1.0, 1.0);
  const auto result = search.update(true, false, 2.0, 1.0);
  EXPECT_NEAR(result.desired_yaw_rad, 0.5, 1e-9);
}

TEST(StartupSearch, StopsAfterStableTargetAcquisition)
{
  StartupSearch search({true, 0.5, 1.0, 0.5, true, true, 0.2});
  EXPECT_TRUE(search.update(true, false, 1.0, 0.0).active);
  EXPECT_FALSE(search.update(true, true, 2.0, 0.5).completed);
  const auto acquired = search.update(true, true, 2.21, 0.6);
  EXPECT_TRUE(acquired.completed);
  EXPECT_TRUE(acquired.target_acquired);
  EXPECT_FALSE(acquired.pending);
}

TEST(StartupSearch, RepeatsSweepUntilTargetIsAcquired)
{
  StartupSearch search({true, 1.0, 1.0, 0.0, true, true, 0.2});
  search.update(true, false, 0.0, 0.0);
  const auto after_one_turn = search.update(true, false, 2.0 * kPi + 1.0, 0.0);
  EXPECT_TRUE(after_one_turn.active);
  EXPECT_FALSE(after_one_turn.completed);
  EXPECT_GT(after_one_turn.desired_yaw_rad, 2.0 * kPi);
}
