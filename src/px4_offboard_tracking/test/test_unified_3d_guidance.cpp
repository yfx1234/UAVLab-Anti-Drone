#include <gtest/gtest.h>

#include <cmath>

#include "px4_offboard_tracking/unified_3d_guidance.hpp"

using px4_offboard_tracking::Unified3dGuidance;
using px4_offboard_tracking::Unified3dGuidanceConfig;
using px4_offboard_tracking::Unified3dGuidanceInput;

TEST(Unified3dGuidance, ProducesOneValidThreeDimensionalCommand)
{
  Unified3dGuidance guidance(Unified3dGuidanceConfig{});
  const auto result = guidance.update(Unified3dGuidanceInput{
    {4.0, 0.0, -20.0}, {-2.0, 1.0, 0.0}, {0.0, 0.0, 0.0}});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.range_m, 20.0);
  EXPECT_TRUE(std::isfinite(result.acceleration.ax));
  EXPECT_TRUE(std::isfinite(result.acceleration.ay));
  EXPECT_TRUE(std::isfinite(result.acceleration.az));
  EXPECT_LE(result.attack_axis_error_rad, result.allowed_axis_error_rad + 1e-9);
}

TEST(Unified3dGuidance, RejectsGeometryOutsideTiltAndCameraCones)
{
  Unified3dGuidance guidance(Unified3dGuidanceConfig{});
  const auto result = guidance.update(Unified3dGuidanceInput{
    {30.0, 0.0, -1.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  EXPECT_FALSE(result.valid);
}

TEST(Unified3dGuidance, TightensAlignmentNearCaptureRange)
{
  Unified3dGuidance guidance(Unified3dGuidanceConfig{});
  const auto far_result = guidance.update(Unified3dGuidanceInput{
    {3.0, 0.0, -46.0}, {-1.0, 2.0, 0.0}, {0.0, 0.0, 0.0}});
  const auto near_result = guidance.update(Unified3dGuidanceInput{
    {1.0, 0.0, -34.5}, {-1.0, 2.0, 0.0}, {0.0, 0.0, 0.0}});
  ASSERT_TRUE(far_result.valid);
  ASSERT_TRUE(near_result.valid);
  EXPECT_LT(near_result.allowed_axis_error_rad, far_result.allowed_axis_error_rad);
}

TEST(Unified3dGuidance, UsesWiderCameraConeBeforeAttackIsEnabled)
{
  Unified3dGuidanceConfig config;
  Unified3dGuidance guidance(config);
  const auto tracking = guidance.update(Unified3dGuidanceInput{
    {1.0, 0.0, -25.0}, {-1.0, 2.0, 0.0}, {},
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), false});
  const auto attacking = guidance.update(Unified3dGuidanceInput{
    {1.0, 0.0, -25.0}, {-1.0, 2.0, 0.0}, {},
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), true});
  ASSERT_TRUE(tracking.valid);
  ASSERT_TRUE(attacking.valid);
  EXPECT_NEAR(
    tracking.allowed_axis_error_rad, config.depth_fov_constraint_rad, 1e-12);
  EXPECT_NEAR(
    attacking.allowed_axis_error_rad, config.capture_alignment_rad, 1e-12);
}

TEST(Unified3dGuidance, RampsTerminalAlignmentWithoutAnAxisConstraintStep)
{
  Unified3dGuidanceConfig config;
  Unified3dGuidance guidance(config);
  Unified3dGuidanceInput input;
  input.relative_position_ned = {1.0, 0.0, -25.0};
  input.relative_velocity_ned = {-1.0, 2.0, 0.0};
  input.terminal_alignment_required = true;

  input.terminal_alignment_strength = 0.0;
  const auto wide = guidance.update(input);
  input.terminal_alignment_strength = 0.5;
  const auto halfway = guidance.update(input);
  input.terminal_alignment_strength = 1.0;
  const auto tight = guidance.update(input);

  ASSERT_TRUE(wide.valid);
  ASSERT_TRUE(halfway.valid);
  ASSERT_TRUE(tight.valid);
  EXPECT_GT(wide.allowed_axis_error_rad, halfway.allowed_axis_error_rad);
  EXPECT_GT(halfway.allowed_axis_error_rad, tight.allowed_axis_error_rad);
  EXPECT_NEAR(
    halfway.allowed_axis_error_rad,
    0.5 * (wide.allowed_axis_error_rad + tight.allowed_axis_error_rad), 1.0e-12);
}

TEST(Unified3dGuidance, AppliesPredictiveFovBarrierOverride)
{
  Unified3dGuidanceConfig config;
  Unified3dGuidance guidance(config);
  Unified3dGuidanceInput input;
  input.relative_position_ned = {2.0, 0.0, -25.0};
  input.relative_velocity_ned = {-1.0, 2.0, 0.0};
  input.maximum_axis_error_rad = 0.08726646259971647;
  input.terminal_alignment_required = false;
  const auto result = guidance.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.allowed_axis_error_rad, input.maximum_axis_error_rad, 1e-12);
  EXPECT_LE(result.attack_axis_error_rad, input.maximum_axis_error_rad + 1e-9);
}

TEST(Unified3dGuidance, RadialLoopAcceleratesAndBrakesWithoutLateralBias)
{
  Unified3dGuidanceConfig config;
  config.maximum_tilt_rad = 0.8;
  config.depth_fov_constraint_rad = 0.2;
  Unified3dGuidance guidance(config);
  const auto slow = guidance.update(Unified3dGuidanceInput{
    {0.0, 0.0, -20.0}, {0.0, 0.0, 0.0}, {}});
  const auto fast = guidance.update(Unified3dGuidanceInput{
    {0.0, 0.0, -20.0}, {0.0, 0.0, 8.0}, {}});
  ASSERT_TRUE(slow.valid);
  ASSERT_TRUE(fast.valid);
  EXPECT_LT(slow.acceleration.az, 0.0);
  EXPECT_GT(fast.acceleration.az, 0.0);
  EXPECT_NEAR(slow.acceleration.ay, 0.0, 1e-12);
  EXPECT_NEAR(fast.acceleration.ay, 0.0, 1e-12);
}

TEST(Unified3dGuidance, MatchesCrossingTargetVelocityWithoutRadialClosure)
{
  Unified3dGuidanceConfig config;
  config.depth_fov_constraint_rad = 0.2;
  Unified3dGuidance guidance(config);
  Unified3dGuidanceInput input;
  input.relative_position_ned = {0.0, 0.0, -20.0};
  input.relative_velocity_ned = {0.0, 2.0, 0.0};
  input.desired_closing_speed_mps = 0.0;
  input.terminal_alignment_required = false;
  const auto result = guidance.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.closing_speed_mps, 0.0, 1e-12);
  EXPECT_GT(result.transverse_velocity_acceleration_ned[1], 1.9);
  EXPECT_GT(result.acceleration.ay, 0.0);
}

TEST(Unified3dGuidance, AccelerationMapsBackToTheSingleConstrainedAttackAxis)
{
  Unified3dGuidance guidance(Unified3dGuidanceConfig{});
  const auto result = guidance.update(Unified3dGuidanceInput{
    {3.0, 1.0, -20.0}, {-1.0, 0.5, 0.0}, {}});
  ASSERT_TRUE(result.valid);
  const std::array<double, 3> specific_force{
    result.acceleration.ax,
    result.acceleration.ay,
    result.acceleration.az - 9.80665};
  const double length = std::sqrt(
    specific_force[0] * specific_force[0] +
    specific_force[1] * specific_force[1] +
    specific_force[2] * specific_force[2]);
  ASSERT_GT(length, 1e-9);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(
      specific_force[axis] / length,
      result.constrained_attack_axis_ned[axis], 1e-12);
  }
}

TEST(Unified3dGuidance, ScalesTargetAccelerationByManeuverConfidence)
{
  Unified3dGuidanceConfig config;
  config.target_acceleration_feedforward_gain = 0.6;
  config.maximum_tilt_rad = 0.8;
  config.depth_fov_constraint_rad = 0.3;
  Unified3dGuidance guidance(config);

  Unified3dGuidanceInput full;
  full.relative_position_ned = {0.0, 0.0, -25.0};
  full.target_acceleration_ned = {0.0, 2.0, 0.0};
  full.desired_closing_speed_mps = 0.0;
  full.terminal_alignment_required = false;
  full.target_acceleration_confidence = 1.0;
  const auto full_result = guidance.update(full);

  auto half = full;
  half.target_acceleration_confidence = 0.5;
  const auto half_result = guidance.update(half);
  ASSERT_TRUE(full_result.valid);
  ASSERT_TRUE(half_result.valid);
  EXPECT_NEAR(full_result.target_acceleration_feedforward_ned[1], 1.2, 1e-12);
  EXPECT_NEAR(half_result.target_acceleration_feedforward_ned[1], 0.6, 1e-12);
}

TEST(Unified3dGuidance, RejectsInvalidManeuverConfidence)
{
  Unified3dGuidance guidance(Unified3dGuidanceConfig{});
  Unified3dGuidanceInput input;
  input.relative_position_ned = {0.0, 0.0, -25.0};
  input.target_acceleration_confidence = 1.1;
  EXPECT_FALSE(guidance.update(input).valid);
}

TEST(Unified3dGuidance, PlannerReplacesOnlyTheRadialControlSource)
{
  Unified3dGuidanceConfig config;
  config.maximum_tilt_rad = 0.6;
  config.depth_fov_constraint_rad = 0.2;
  Unified3dGuidance guidance(config);
  Unified3dGuidanceInput input;
  input.relative_position_ned = {2.0, 0.0, -25.0};
  input.relative_velocity_ned = {-8.0, 0.0, 0.0};
  input.desired_closing_speed_mps = 0.0;
  input.terminal_alignment_required = false;
  input.planned_acceleration_ned = {0.25, 0.0, 0.0};
  input.launch_planner_blend = 1.0;
  const auto result = guidance.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.launch_planner_blend, 1.0);
  EXPECT_TRUE(result.planned_radial_control_active);
  // The legacy radial controller would brake the 8 m/s closure. Verify the
  // selected radial source directly: the final NED x component may have the
  // opposite sign after the common camera-cone projection adds a transverse
  // correction for this deliberately off-axis geometry.
  const double expected_planned_radial =
    input.planned_acceleration_ned[0] * result.line_of_sight_ned[0];
  EXPECT_NEAR(result.selected_radial_acceleration_mps2, expected_planned_radial, 1.0e-12);
  EXPECT_GT(result.selected_radial_acceleration_mps2, 0.0);
  EXPECT_LE(result.attack_axis_error_rad, result.allowed_axis_error_rad + 1.0e-9);
}

TEST(Unified3dGuidance, RejectsInvalidLaunchPlannerBlend)
{
  Unified3dGuidance guidance(Unified3dGuidanceConfig{});
  Unified3dGuidanceInput input;
  input.relative_position_ned = {0.0, 0.0, -25.0};
  input.launch_planner_blend = 1.1;
  EXPECT_FALSE(guidance.update(input).valid);
}
