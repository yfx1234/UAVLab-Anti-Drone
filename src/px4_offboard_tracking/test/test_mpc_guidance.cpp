#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "px4_offboard_tracking/mpc_guidance.hpp"

using px4_offboard_tracking::MpcGuidance;
using px4_offboard_tracking::MpcGuidanceConfig;

TEST(MpcGuidance, CommandsPursuerTowardRelativePositionAndVelocity)
{
  MpcGuidance controller(MpcGuidanceConfig{});
  const auto result = controller.update({4.0, -2.0, -30.0}, {1.0, -0.5, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.acceleration.ax, 0.0);
  EXPECT_LT(result.acceleration.ay, 0.0);
  EXPECT_LE(std::hypot(result.acceleration.ax, result.acceleration.ay), 1.5 + 1e-12);
  EXPECT_NEAR(result.acceleration.az, 0.0, 1e-12);
}

TEST(MpcGuidance, BrakesWhenPursuerIsClosingTooFast)
{
  MpcGuidance controller(MpcGuidanceConfig{});
  const auto result = controller.update({0.0, 0.0, -30.0}, {-2.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.acceleration.ax, 0.0);
}

TEST(MpcGuidance, AnticipatesEstimatedTargetAcceleration)
{
  MpcGuidance controller(MpcGuidanceConfig{});
  const auto without_acceleration = controller.update(
    {1.0, 0.0, -30.0}, {0.0, 0.0, 0.0});
  const auto with_acceleration = controller.update(
    {1.0, 0.0, -30.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  ASSERT_TRUE(without_acceleration.valid);
  ASSERT_TRUE(with_acceleration.valid);
  EXPECT_GT(with_acceleration.acceleration.ax, without_acceleration.acceleration.ax);
}

TEST(MpcGuidance, EnforcesHorizontalAccelerationNorm)
{
  MpcGuidanceConfig config;
  config.maximum_horizontal_acceleration_mps2 = 0.7;
  MpcGuidance controller(config);
  const auto result = controller.update({100.0, 100.0, -30.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.acceleration_limited);
  EXPECT_NEAR(std::hypot(result.acceleration.ax, result.acceleration.ay), 0.7, 1e-12);
}

TEST(MpcGuidance, ControlsVerticalSeparationAndLimitsAcceleration)
{
  MpcGuidanceConfig config;
  config.desired_vertical_separation_m = 30.0;
  config.maximum_vertical_acceleration_mps2 = 0.6;
  MpcGuidance controller(config);
  const auto climb = controller.update({0.0, 0.0, -40.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(climb.valid);
  EXPECT_LT(climb.acceleration.az, 0.0);
  EXPECT_NEAR(climb.acceleration.az, -0.6, 1e-12);
  EXPECT_TRUE(climb.vertical_acceleration_limited);
  const auto descend = controller.update({0.0, 0.0, -20.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(descend.valid);
  EXPECT_GT(descend.acceleration.az, 0.0);
}

TEST(MpcGuidance, RejectsInvalidOrNonUpwardGeometry)
{
  MpcGuidance controller(MpcGuidanceConfig{});
  EXPECT_FALSE(controller.update({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}).valid);
  EXPECT_FALSE(controller.update(
    {std::numeric_limits<double>::quiet_NaN(), 0.0, -30.0},
    {0.0, 0.0, 0.0}).valid);
}

TEST(MpcGuidance, RejectsInvalidConfiguration)
{
  MpcGuidanceConfig config;
  config.horizon_steps = 0;
  EXPECT_THROW({MpcGuidance controller(config);}, std::invalid_argument);
}
