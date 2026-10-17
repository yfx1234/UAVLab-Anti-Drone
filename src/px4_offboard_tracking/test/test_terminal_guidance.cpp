#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "px4_offboard_tracking/terminal_guidance.hpp"

using px4_offboard_tracking::TerminalGuidance;
using px4_offboard_tracking::TerminalGuidanceConfig;

TEST(TerminalGuidance, CommandsAccelerationAlongLineOfSight)
{
  TerminalGuidance guidance(TerminalGuidanceConfig{});
  const auto result = guidance.update(
    {10.0, 0.0, -30.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.acceleration.ax, 0.0);
  EXPECT_LT(result.acceleration.az, 0.0);
  EXPECT_NEAR(result.line_of_sight_rate_rad_s, 0.0, 1e-12);
}

TEST(TerminalGuidance, UsesThreeDimensionalLineOfSightWithoutAltitudeHold)
{
  TerminalGuidance guidance(TerminalGuidanceConfig{});
  const auto result = guidance.update(
    {5.0, 0.0, -25.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.acceleration.ax, 0.0);
  EXPECT_LT(result.acceleration.az, 0.0);
  EXPECT_NEAR(
    result.acceleration.ax / -result.acceleration.az, 5.0 / 25.0, 1e-12);
}

TEST(TerminalGuidance, ProportionalNavigationOpposesLineOfSightRotation)
{
  TerminalGuidanceConfig config;
  config.line_of_sight_acceleration_mps2 = 0.1;
  TerminalGuidance guidance(config);
  const auto result = guidance.update(
    {20.0, 0.0, -20.0}, {-2.0, 3.0, 0.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.line_of_sight_rate_rad_s, 0.0);
  EXPECT_GT(result.acceleration.ay, 0.0);
}

TEST(TerminalGuidance, EnforcesAccelerationLimits)
{
  TerminalGuidanceConfig config;
  config.maximum_horizontal_acceleration_mps2 = 0.5;
  config.maximum_vertical_acceleration_mps2 = 0.3;
  TerminalGuidance guidance(config);
  const auto result = guidance.update(
    {100.0, 100.0, -100.0}, {-20.0, 20.0, 0.0}, {10.0, -10.0, -10.0});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.acceleration_limited);
  EXPECT_LE(std::hypot(result.acceleration.ax, result.acceleration.ay), 0.5 + 1e-12);
  EXPECT_LE(std::abs(result.acceleration.az), 0.3 + 1e-12);
}

TEST(TerminalGuidance, RejectsInvalidInputAndConfiguration)
{
  TerminalGuidance guidance(TerminalGuidanceConfig{});
  EXPECT_FALSE(guidance.update(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}).valid);
  EXPECT_FALSE(guidance.update(
    {std::numeric_limits<double>::quiet_NaN(), 0.0, -25.0},
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}).valid);
  auto config = TerminalGuidanceConfig{};
  config.navigation_constant = 0.0;
  EXPECT_THROW({TerminalGuidance invalid(config);}, std::invalid_argument);
}
