#include <gtest/gtest.h>

#include <cmath>

#include "px4_offboard_tracking/z_axis_alignment.hpp"

using px4_offboard_tracking::Quaternion;
using px4_offboard_tracking::ZAxisAlignment;
using px4_offboard_tracking::ZAxisAlignmentConfig;

TEST(ZAxisAlignment, LevelVehicleIsAlignedWithTargetDirectlyAbove)
{
  ZAxisAlignment alignment(ZAxisAlignmentConfig{});
  const auto result = alignment.update({0.0, 0.0, -7.0}, {0.0, 0.0, 0.0}, Quaternion{});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.desired_camera_axis_ned[2], -1.0, 1e-12);
  EXPECT_NEAR(result.alignment_error_rad, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.alignment_weight, 1.0);
  EXPECT_FALSE(result.tilt_limited);
}

TEST(ZAxisAlignment, PredictsLosAndLimitsTilt)
{
  ZAxisAlignmentConfig config;
  config.prediction_time_s = 1.0;
  config.maximum_tilt_rad = 0.2;
  ZAxisAlignment alignment(config);
  const auto result = alignment.update({3.0, 0.0, -7.0}, {2.0, 0.0, 0.0}, Quaternion{});
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.predicted_los_ned[0], 5.0);
  EXPECT_TRUE(result.tilt_limited);
  EXPECT_DOUBLE_EQ(result.alignment_weight, 0.0);
  EXPECT_NEAR(
    std::atan2(result.desired_camera_axis_ned[0], -result.desired_camera_axis_ned[2]),
    0.2, 1e-12);
}

TEST(ZAxisAlignment, DelaysFullAlignmentUntilCorridorMotionIsSettled)
{
  ZAxisAlignment alignment(ZAxisAlignmentConfig{});
  const auto moving = alignment.update({0.5, 0.0, -7.0}, {1.0, 0.0, 0.0}, Quaternion{});
  const auto settled = alignment.update({0.5, 0.0, -7.0}, {0.0, 0.0, 0.0}, Quaternion{});
  ASSERT_TRUE(moving.valid);
  ASSERT_TRUE(settled.valid);
  EXPECT_DOUBLE_EQ(moving.alignment_weight, 0.0);
  EXPECT_NEAR(settled.alignment_weight, 0.75, 1e-12);
}

TEST(ZAxisAlignment, RejectsTargetBelowCamera)
{
  ZAxisAlignment alignment(ZAxisAlignmentConfig{});
  EXPECT_FALSE(alignment.update({0.0, 0.0, 3.0}, {0.0, 0.0, 0.0}, Quaternion{}).valid);
}
