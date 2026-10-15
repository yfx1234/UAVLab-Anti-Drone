#include <gtest/gtest.h>

#include <cmath>

#include "px4_offboard_tracking/approach_corridor.hpp"

using px4_offboard_tracking::ApproachCorridor;
using px4_offboard_tracking::ApproachCorridorConfig;

TEST(ApproachCorridor, PredictsMovingTargetAndCommandsUnderTargetApproach)
{
  ApproachCorridor corridor(ApproachCorridorConfig{});
  const auto result = corridor.update({4.0, 0.0, -7.0}, {1.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.predicted_relative_position_ned[0], 4.8);
  EXPECT_DOUBLE_EQ(result.corridor_error_ned[2], 0.0);
  EXPECT_GT(result.acceleration.ax, 0.0);
  EXPECT_DOUBLE_EQ(result.acceleration.ay, 0.0);
}

TEST(ApproachCorridor, LimitsPredictionAndHorizontalAcceleration)
{
  ApproachCorridorConfig config;
  config.maximum_lead_distance_m = 1.0;
  config.maximum_horizontal_acceleration_mps2 = 1.5;
  ApproachCorridor corridor(config);
  const auto result = corridor.update({20.0, 0.0, -7.0}, {10.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.predicted_relative_position_ned[0], 21.0);
  EXPECT_NEAR(std::hypot(result.acceleration.ax, result.acceleration.ay), 1.5, 1e-12);
}

TEST(ApproachCorridor, RejectsTargetThatIsNotAbovePursuer)
{
  ApproachCorridor corridor(ApproachCorridorConfig{});
  EXPECT_FALSE(corridor.update({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}).valid);
  EXPECT_FALSE(corridor.update({1.0, 0.0, 2.0}, {0.0, 0.0, 0.0}).valid);
}
