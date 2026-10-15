#include <gtest/gtest.h>

#include <cmath>

#include "virtual_interception_referee/referee_model.hpp"

using virtual_interception_referee::RefereeConfig;
using virtual_interception_referee::RefereeModel;
using virtual_interception_referee::RefereeSegment;
using virtual_interception_referee::RefereeShot;
using virtual_interception_referee::MissionHitCounter;
using virtual_interception_referee::MissionHitCounterConfig;
using virtual_interception_referee::body_minus_z_in_ned;
using virtual_interception_referee::position_in_task_frame;
using virtual_interception_referee::truth_interval_usable;

TEST(RefereeFrames, SubtractsPropellerStartTaskOrigin)
{
  const auto task = position_in_task_frame(
    {12.5, -3.0, -8.0}, {2.5, 1.0, -0.5});
  EXPECT_DOUBLE_EQ(task[0], 10.0);
  EXPECT_DOUBLE_EQ(task[1], -4.0);
  EXPECT_DOUBLE_EQ(task[2], -7.5);
}

TEST(RefereeFrames, RotatesBodyMinusZIntoNed)
{
  const auto level = body_minus_z_in_ned({1.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(level[0], 0.0, 1e-12);
  EXPECT_NEAR(level[1], 0.0, 1e-12);
  EXPECT_NEAR(level[2], -1.0, 1e-12);

  const double half_angle = 0.25 * std::acos(-1.0);
  const auto pitched = body_minus_z_in_ned(
    {std::cos(half_angle), 0.0, std::sin(half_angle), 0.0});
  EXPECT_NEAR(pitched[0], -1.0, 1e-12);
  EXPECT_NEAR(pitched[1], 0.0, 1e-12);
  EXPECT_NEAR(pitched[2], 0.0, 1e-12);
}

TEST(RefereeTruthTiming, RejectsMissingOrOutOfOrderTruth)
{
  EXPECT_TRUE(truth_interval_usable(0.0, 0.033, 0.1));
  EXPECT_TRUE(truth_interval_usable(0.033, 0.099, 0.1));
  EXPECT_FALSE(truth_interval_usable(0.0, 1.6, 0.1));
  EXPECT_FALSE(truth_interval_usable(0.1, 0.09, 0.1));
}

TEST(RefereeModel, DirectBallisticPathHitsStationaryTarget)
{
  RefereeModel model(RefereeConfig{});
  RefereeShot shot;
  shot.direction_ned = {1.0, 0.0, 0.0};
  RefereeSegment segment;
  segment.start_time_s = 0.0;
  segment.end_time_s = 0.1;
  segment.target_start_ned = {25.0, 0.0, 0.019156};
  segment.target_end_ned = segment.target_start_ned;
  const auto result = model.evaluate_segment(shot, segment);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.hit);
  EXPECT_LT(result.minimum_miss_distance_m, 0.01);
}

TEST(RefereeModel, LateralOffsetProducesMiss)
{
  RefereeModel model(RefereeConfig{});
  RefereeShot shot;
  shot.direction_ned = {1.0, 0.0, 0.0};
  RefereeSegment segment;
  segment.start_time_s = 0.0;
  segment.end_time_s = 0.1;
  segment.target_start_ned = {25.0, 2.0, 0.019156};
  segment.target_end_ned = segment.target_start_ned;
  const auto result = model.evaluate_segment(shot, segment);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.hit);
  EXPECT_GT(result.minimum_miss_distance_m, 1.0);
}

TEST(RefereeModel, MeasuredStrikeConeScalesWithProjectileTravel)
{
  RefereeConfig config;
  config.gravity_mps2 = 0.0;
  config.effect_cone_half_angle_rad = std::atan(0.85 / 35.0);
  RefereeModel model(config);
  RefereeShot shot;
  shot.direction_ned = {1.0, 0.0, 0.0};
  RefereeSegment segment;
  segment.start_time_s = 0.0;
  segment.end_time_s = 0.1;
  segment.target_start_ned = {35.0, 0.8, 0.0};
  segment.target_end_ned = segment.target_start_ned;
  const auto inside = model.evaluate_segment(shot, segment);
  ASSERT_TRUE(inside.valid);
  EXPECT_TRUE(inside.hit);
  EXPECT_NEAR(inside.effective_hit_radius_m, 0.85, 0.002);

  segment.target_start_ned = {35.0, 0.9, 0.0};
  segment.target_end_ned = segment.target_start_ned;
  const auto outside = model.evaluate_segment(shot, segment);
  ASSERT_TRUE(outside.valid);
  EXPECT_FALSE(outside.hit);
  EXPECT_NEAR(outside.effective_hit_radius_m, 0.85, 0.002);
}

TEST(RefereeModel, IncludesPursuerVelocityInProjectileInitialVelocity)
{
  RefereeModel model(RefereeConfig{});
  RefereeShot shot;
  shot.pursuer_velocity_ned = {5.0, 0.0, 0.0};
  shot.direction_ned = {1.0, 0.0, 0.0};
  const auto position = model.projectile_position(shot, 0.05);
  EXPECT_NEAR(position[0], 20.25, 1e-9);
}

TEST(MissionHitCounter, RequiresTwoHitsSeparatedByThreeSeconds)
{
  MissionHitCounter counter(MissionHitCounterConfig{3.0, 2});
  EXPECT_TRUE(counter.accept(true, 30.0));
  EXPECT_FALSE(counter.mission_success());
  EXPECT_FALSE(counter.accept(true, 32.9));
  EXPECT_EQ(counter.successful_hit_count(), 1U);
  EXPECT_TRUE(counter.accept(true, 33.0));
  EXPECT_TRUE(counter.mission_success());
}

TEST(MissionHitCounter, MissesNeverIncrementCount)
{
  MissionHitCounter counter(MissionHitCounterConfig{3.0, 2});
  EXPECT_FALSE(counter.accept(false, 30.0));
  EXPECT_EQ(counter.successful_hit_count(), 0U);
}
