#include <gtest/gtest.h>

#include "falcon_intercept_control/control_core.hpp"

#include <array>
#include <cmath>

using falcon_intercept_control::ControlConfig;
using falcon_intercept_control::ControlCore;
using falcon_intercept_control::ControlInput;
using falcon_intercept_control::MissionState;
using falcon_intercept_control::RigidTransform;

namespace
{

ControlInput valid_input(double now_s, double forward_m = 5.0)
{
  ControlInput input;
  input.now_s = now_s;
  input.mission_enabled = true;
  input.target_valid = true;
  input.target_position_body_frd_m = {forward_m, 0.0, 0.0};
  input.target_velocity_body_frd_mps = {0.0, 0.0, 0.0};
  input.maximum_position_variance_m2 = 0.25;
  input.confidence = 0.9;
  input.target_age_s = 0.01;
  input.health_valid = true;
  input.left_centered = true;
  input.right_centered = true;
  return input;
}

}  // namespace

TEST(CoordinateTransform, StandardForwardCameraMapsOpticalToBodyFrd)
{
  RigidTransform transform;
  // body FRD -> optical: (x,y,z) -> (y,z,x)
  transform.q_body_to_optical = {0.5, -0.5, -0.5, -0.5};
  transform.camera_translation_body_frd_m = {0.1, 0.0, 0.0};
  const auto body = falcon_intercept_control::optical_position_to_body_frd(
    {0.2, 0.1, 5.0}, transform);
  EXPECT_NEAR(body[0], 5.1, 1e-9);
  EXPECT_NEAR(body[1], 0.2, 1e-9);
  EXPECT_NEAR(body[2], 0.1, 1e-9);
}

TEST(CoordinateTransform, CovarianceAxesFollowOpticalToBodyRotation)
{
  RigidTransform transform;
  transform.q_body_to_optical = {0.5, -0.5, -0.5, -0.5};
  const auto body = falcon_intercept_control::optical_covariance_to_body_frd(
    {1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0}, transform);
  EXPECT_NEAR(body[0], 3.0, 1e-9);
  EXPECT_NEAR(body[4], 1.0, 1e-9);
  EXPECT_NEAR(body[8], 2.0, 1e-9);
}

TEST(RelativeKinematics, ComputesClosingSpeedAndClosestApproach)
{
  const auto result = falcon_intercept_control::compute_relative_kinematics(
    {10.0, 0.0, 0.0}, {-2.0, 0.0, 0.0});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.range_m, 10.0, 1e-9);
  EXPECT_NEAR(result.closing_speed_mps, 2.0, 1e-9);
  EXPECT_NEAR(result.time_to_closest_approach_s, 5.0, 1e-9);
  EXPECT_NEAR(result.predicted_miss_distance_m, 0.0, 1e-9);
}

TEST(MissionControl, SearchNeverProducesTranslation)
{
  ControlCore core(ControlConfig{});
  ControlInput input;
  input.now_s = 0.0;
  input.mission_enabled = true;
  input.search_cue_valid = true;
  input.search_horizontal_error_px = 200.0;
  const auto output = core.update(input);
  EXPECT_EQ(output.state, MissionState::SEARCH);
  EXPECT_EQ(output.velocity_body_frd_mps, (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_NEAR(output.yaw_rate_rps, 0.35, 1e-9);
  EXPECT_TRUE(output.valid);
}

TEST(MissionControl, OptionalStartupSearchCompletesBeforeTracking)
{
  ControlConfig config;
  config.startup_search_enabled = true;
  config.startup_search_yaw_rate_rps = 1.0;
  config.startup_search_turns = 0.25;
  config.startup_search_settle_time_s = 0.5;
  config.maximum_search_yaw_rate_rps = 1.0;
  ControlCore core(config);

  auto input = valid_input(10.0);
  const auto start = core.update(input);
  EXPECT_EQ(start.state, MissionState::SEARCH);
  EXPECT_NEAR(start.yaw_rate_rps, 1.0, 1e-9);
  EXPECT_EQ(start.reason, "startup search sweep");

  input.now_s = 10.5;
  EXPECT_EQ(core.update(input).state, MissionState::SEARCH);
  input.now_s = 10.0 + 0.5 * 3.14159265358979323846 + 0.25;
  const auto settle = core.update(input);
  EXPECT_EQ(settle.state, MissionState::SEARCH);
  EXPECT_NEAR(settle.yaw_rate_rps, 0.0, 1e-9);
  EXPECT_EQ(settle.reason, "startup search settle");

  input.now_s = 10.0 + 0.5 * 3.14159265358979323846 + 0.51;
  EXPECT_EQ(core.update(input).state, MissionState::TRACK);
}

TEST(MissionControl, DisabledStartupSearchPreservesExistingAcquisition)
{
  ControlConfig config;
  config.startup_search_enabled = false;
  ControlCore core(config);
  EXPECT_EQ(core.update(valid_input(0.0)).state, MissionState::SEARCH);
  EXPECT_EQ(core.update(valid_input(0.1)).state, MissionState::TRACK);
}

TEST(MissionControl, ValidTargetReachesFireOnceThenLandAndDone)
{
  ControlConfig config;
  config.track_confirm_frames = 1;
  config.fire_hold_frames = 2;
  config.land_hold_s = 0.1;
  ControlCore core(config);
  EXPECT_EQ(core.update(valid_input(0.0)).state, MissionState::SEARCH);
  EXPECT_EQ(core.update(valid_input(0.1)).state, MissionState::TRACK);
  EXPECT_EQ(core.update(valid_input(0.2)).state, MissionState::APPROACH);
  EXPECT_GT(core.update(valid_input(0.3)).velocity_body_frd_mps[0], 0.0);
  EXPECT_FALSE(core.update(valid_input(0.4, 1.8)).fire_event);
  const auto fire = core.update(valid_input(0.5, 1.8));
  EXPECT_EQ(fire.state, MissionState::FIRE);
  EXPECT_TRUE(fire.fire_event);
  const auto land = core.update(valid_input(0.6, 1.8));
  EXPECT_EQ(land.state, MissionState::LAND);
  EXPECT_TRUE(land.land_request);
  auto landed = valid_input(0.8, 1.8);
  landed.landed = true;
  EXPECT_EQ(core.update(landed).state, MissionState::DONE);
  EXPECT_FALSE(core.update(landed).fire_event);
}

TEST(MissionControl, StaleHealthReturnsApproachToSearchWithZeroVelocity)
{
  ControlConfig config;
  config.track_confirm_frames = 1;
  ControlCore core(config);
  core.update(valid_input(0.0));
  core.update(valid_input(0.1));
  ASSERT_EQ(core.update(valid_input(0.2)).state, MissionState::APPROACH);
  auto stale = valid_input(0.8);
  stale.health_valid = false;
  const auto output = core.update(stale);
  EXPECT_EQ(output.state, MissionState::SEARCH);
  EXPECT_EQ(output.velocity_body_frd_mps, (std::array<double, 3>{0.0, 0.0, 0.0}));
}

TEST(MissionControl, UnsafeMinimumRangeLatchesAbort)
{
  ControlConfig config;
  config.track_confirm_frames = 1;
  ControlCore core(config);
  core.update(valid_input(0.0));
  core.update(valid_input(0.1));
  core.update(valid_input(0.2));
  const auto output = core.update(valid_input(0.3, 1.4));
  EXPECT_EQ(output.state, MissionState::ABORT);
  EXPECT_FALSE(output.valid);
  EXPECT_EQ(output.velocity_body_frd_mps, (std::array<double, 3>{0.0, 0.0, 0.0}));
}

TEST(MissionControl, InvalidCovarianceCannotLeaveSearch)
{
  ControlCore core(ControlConfig{});
  auto input = valid_input(0.0);
  input.maximum_position_variance_m2 = 100.0;
  EXPECT_EQ(core.update(input).state, MissionState::SEARCH);
  input.now_s = 0.1;
  EXPECT_EQ(core.update(input).state, MissionState::SEARCH);
}
