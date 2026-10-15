#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/launch_position_planner.hpp"

using px4_offboard_tracking::LaunchPositionPlanner;
using px4_offboard_tracking::LaunchPositionPlannerConfig;
using px4_offboard_tracking::LaunchPositionPlannerInput;

namespace
{

LaunchPositionPlannerConfig test_config()
{
  LaunchPositionPlannerConfig config;
  config.minimum_horizon_s = 0.5;
  config.maximum_horizon_s = 10.0;
  config.horizon_step_s = 0.05;
  config.minimum_launch_range_m = 20.0;
  config.maximum_launch_range_m = 30.0;
  config.preferred_launch_range_m = 25.0;
  config.launch_range_step_m = 1.0;
  config.maximum_horizontal_acceleration_mps2 = 3.0;
  config.maximum_vertical_acceleration_mps2 = 2.0;
  config.maximum_speed_mps = 14.0;
  config.maximum_tilt_rad = 0.6;
  config.time_weight = 0.05;
  config.acceleration_weight = 1.0;
  config.acceleration_change_weight = 0.1;
  return config;
}

void expect_reaches_launch_geometry(
  const LaunchPositionPlannerInput & input,
  const px4_offboard_tracking::LaunchPositionPlannerResult & result)
{
  ASSERT_TRUE(result.valid);
  const double t = result.horizon_s;
  for (int axis = 0; axis < 3; ++axis) {
    const double target_velocity =
      input.pursuer_velocity_ned[axis] + input.relative_velocity_ned[axis];
    const double target = input.relative_position_ned[axis] +
      target_velocity * t +
      0.5 * input.target_acceleration_ned[axis] *
      input.target_acceleration_confidence * t * t;
    const double pursuer = input.pursuer_velocity_ned[axis] * t +
      0.5 * result.acceleration_ned[axis] * t * t;
    const double expected = axis == 2 ? -result.launch_range_m : 0.0;
    EXPECT_NEAR(target - pursuer, expected, 1.0e-8);
  }
}

}  // namespace

TEST(LaunchPositionPlanner, RejectsInvalidConfiguration)
{
  auto config = test_config();
  config.maximum_horizon_s = 0.1;
  EXPECT_THROW((void)LaunchPositionPlanner{config}, std::invalid_argument);
}

TEST(LaunchPositionPlanner, PlansInboundFirstPassWithoutSpeedMatching)
{
  LaunchPositionPlanner planner(test_config());
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {40.0, 0.0, -25.0};
  input.relative_velocity_ned = {-5.0, 0.0, 0.0};
  const auto result = planner.plan(input);
  expect_reaches_launch_geometry(input, result);
  EXPECT_GT(result.horizon_s, 4.0);
  EXPECT_LT(std::abs(result.acceleration_ned[0]), 1.5);
  // The target is still moving rapidly at launch; no common-speed terminal
  // constraint was silently introduced.
  EXPECT_GT(std::abs(result.predicted_relative_velocity_ned[0]), 2.0);
}

TEST(LaunchPositionPlanner, HoldsAnImmediateLaunchCorridorInsteadOfSkippingIt)
{
  LaunchPositionPlanner planner(test_config());
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {0.25, 0.0, -25.0};
  input.relative_velocity_ned = {5.0, 0.0, 0.0};
  input.pursuer_velocity_ned = {-1.0, 0.0, 0.0};
  const auto result = planner.plan(input);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.horizon_s, 0.0);
  EXPECT_DOUBLE_EQ(result.cost, 0.0);
  EXPECT_NEAR(result.launch_range_m, std::hypot(0.25, 25.0), 1.0e-12);
  EXPECT_DOUBLE_EQ(result.acceleration_ned[0], 0.0);
  EXPECT_GT(std::abs(result.predicted_relative_velocity_ned[0]), 2.0);
}

TEST(LaunchPositionPlanner, DoesNotCallAnObliquePassAnImmediateLaunch)
{
  LaunchPositionPlanner planner(test_config());
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {2.0, 0.0, -25.0};
  input.relative_velocity_ned = {5.0, 0.0, 0.0};
  const auto result = planner.plan(input);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.horizon_s, 0.0);
}

TEST(LaunchPositionPlanner, PlansCrossingTarget)
{
  LaunchPositionPlanner planner(test_config());
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {0.0, 30.0, -25.0};
  input.relative_velocity_ned = {4.0, -1.0, 0.0};
  input.pursuer_velocity_ned = {0.5, 0.0, 0.0};
  const auto result = planner.plan(input);
  expect_reaches_launch_geometry(input, result);
  EXPECT_GT(std::hypot(result.acceleration_ned[0], result.acceleration_ned[1]), 0.1);
}

TEST(LaunchPositionPlanner, PlansTailChaseWithLongerReachableHorizon)
{
  LaunchPositionPlanner planner(test_config());
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {8.0, 0.0, -25.0};
  input.relative_velocity_ned = {5.0, 0.0, 0.0};
  const auto result = planner.plan(input);
  expect_reaches_launch_geometry(input, result);
  EXPECT_GT(result.acceleration_ned[0], 0.0);
  EXPECT_GT(result.horizon_s, 4.0);
}

TEST(LaunchPositionPlanner, UsesBoundedTargetAccelerationPrediction)
{
  auto config = test_config();
  config.maximum_target_acceleration_mps2 = 2.0;
  LaunchPositionPlanner planner(config);
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {0.0, 20.0, -25.0};
  input.relative_velocity_ned = {0.0, 0.0, 0.0};
  input.target_acceleration_ned = {10.0, 0.0, 0.0};
  input.target_acceleration_confidence = 0.5;
  const auto result = planner.plan(input);
  ASSERT_TRUE(result.valid);
  // 10 m/s^2 is norm-limited to 2, then confidence-scaled to 1 m/s^2.
  EXPECT_NEAR(
    result.predicted_target_displacement_ned[0],
    0.5 * result.horizon_s * result.horizon_s, 1.0e-8);
}

TEST(LaunchPositionPlanner, ReturnsInvalidWhenNoCandidateIsReachable)
{
  auto config = test_config();
  config.maximum_horizon_s = 1.0;
  LaunchPositionPlanner planner(config);
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {100.0, 0.0, -25.0};
  const auto result = planner.plan(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.feasible_candidate_count, 0U);
}

TEST(LaunchPositionPlanner, RespectsDownstreamExecutableSpeedLimits)
{
  auto config = test_config();
  config.maximum_speed_mps = 14.0;
  LaunchPositionPlanner planner(config);
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {12.0, 0.0, -25.0};
  input.maximum_executable_horizontal_speed_mps = 3.0;
  input.maximum_executable_vertical_speed_mps = 1.0;
  const auto result = planner.plan(input);
  ASSERT_TRUE(result.valid);
  EXPECT_LE(std::hypot(
    result.predicted_pursuer_displacement_ned[0] * 0.0 +
    result.acceleration_ned[0] * result.horizon_s,
    result.predicted_pursuer_displacement_ned[1] * 0.0 +
    result.acceleration_ned[1] * result.horizon_s), 3.0 + 1.0e-9);
  EXPECT_LE(std::abs(result.acceleration_ned[2] * result.horizon_s), 1.0 + 1.0e-9);
}

TEST(LaunchPositionPlanner, SoftSpeedPreferenceDoesNotChangeHardCapability)
{
  auto unweighted_config = test_config();
  unweighted_config.excess_speed_weight = 0.0;
  LaunchPositionPlanner unweighted(unweighted_config);
  auto weighted_config = unweighted_config;
  weighted_config.excess_speed_weight = 1.0;
  weighted_config.closing_speed_reserve_mps = 2.0;
  LaunchPositionPlanner weighted(weighted_config);
  LaunchPositionPlannerInput input;
  input.relative_position_ned = {24.0, 0.0, -25.0};
  input.maximum_executable_horizontal_speed_mps = 12.0;
  input.maximum_executable_vertical_speed_mps = 1.0;
  const auto fast = unweighted.plan(input);
  const auto economical = weighted.plan(input);
  ASSERT_TRUE(fast.valid);
  ASSERT_TRUE(economical.valid);
  EXPECT_GE(economical.horizon_s, fast.horizon_s);
  EXPECT_LE(economical.predicted_speed_mps, fast.predicted_speed_mps + 1.0e-9);
}
