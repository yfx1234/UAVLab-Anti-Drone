#include <gtest/gtest.h>

#include "px4_offboard_tracking/pure_mission.hpp"

using px4_offboard_tracking::PureMission;
using px4_offboard_tracking::PureMissionConfig;
using px4_offboard_tracking::PureMissionInput;
using px4_offboard_tracking::PureMissionState;

namespace
{

PureMissionInput enter_intercept(PureMission & mission)
{
  PureMissionInput input;
  input.system_ready = true;
  EXPECT_EQ(mission.update(input, 0.0).state, PureMissionState::TAKEOFF_SAFE);
  input.takeoff_ready = true;
  EXPECT_EQ(mission.update(input, 0.1).state, PureMissionState::SEARCH);
  input.search_visible = true;
  input.search_sequence = 1;
  EXPECT_EQ(mission.update(input, 0.2).state, PureMissionState::BEARING_TRACK);
  input.depth_visible = true;
  input.depth_admission_ready = true;
  input.depth_sequence = 1;
  EXPECT_EQ(mission.update(input, 0.3).state, PureMissionState::DEPTH_HANDOVER);
  input.depth_sequence = 2;
  EXPECT_EQ(mission.update(input, 0.4).state, PureMissionState::DEPTH_ACQUIRE);
  input.estimate_valid = true;
  input.guidance_valid = true;
  EXPECT_EQ(mission.update(input, 0.5).state, PureMissionState::INTERCEPT);
  return input;
}

PureMissionConfig immediate_config()
{
  PureMissionConfig config;
  config.search_acquisition_hold_s = 0.0;
  config.depth_acquisition_hold_s = 0.0;
  config.minimum_depth_acquisition_frames = 1;
  config.estimate_stability_hold_s = 0.0;
  config.capture_hold_s = 0.0;
  config.depth_loss_hysteresis_s = 0.0;
  config.fov_recovery_entry_hold_s = 0.0;
  config.tracking_invalid_hysteresis_s = 0.0;
  config.recoil_duration_s = 0.2;
  config.reattack_cooldown_s = 3.0;
  return config;
}

}  // namespace

TEST(PureMission, HoldsSafeTakeoffUntilMissionIsEnabled)
{
  PureMission mission(PureMissionConfig{});
  PureMissionInput input;
  input.system_ready = true;
  EXPECT_EQ(mission.update(input, 0.0).state, PureMissionState::TAKEOFF_SAFE);
  input.takeoff_ready = true;
  input.mission_enabled = false;
  EXPECT_EQ(mission.update(input, 1.0).state, PureMissionState::TAKEOFF_SAFE);
  input.mission_enabled = true;
  EXPECT_EQ(mission.update(input, 1.1).state, PureMissionState::SEARCH);
}

TEST(PureMission, EmitsAtOptimizedInstantWithoutAnExtraControlCycle)
{
  PureMission mission(immediate_config());
  auto input = enter_intercept(mission);
  input.attack_enabled = true;
  input.capture_instantaneous_pass = true;
  const auto shot = mission.update(input, 0.6);
  EXPECT_EQ(shot.state, PureMissionState::EGRESS_REATTACK);
  EXPECT_TRUE(shot.capture_event);
}

TEST(PureMission, DepthAdmissionRejectsHardEdgeFrames)
{
  PureMission mission(immediate_config());
  PureMissionInput input;
  input.system_ready = true;
  mission.update(input, 0.0);
  input.takeoff_ready = true;
  mission.update(input, 0.1);
  input.search_visible = true;
  mission.update(input, 0.2);
  ASSERT_EQ(mission.state(), PureMissionState::BEARING_TRACK);
  input.depth_visible = true;
  input.depth_admission_ready = false;
  input.depth_sequence = 1;
  EXPECT_EQ(mission.update(input, 0.3).state, PureMissionState::BEARING_TRACK);
  input.depth_admission_ready = true;
  EXPECT_EQ(mission.update(input, 0.4).state, PureMissionState::DEPTH_HANDOVER);
}

TEST(PureMission, FovRecoveryResumesOnlyAfterSixDegreeHysteresis)
{
  PureMission mission(immediate_config());
  auto input = enter_intercept(mission);
  input.fov_recovery_required = true;
  EXPECT_EQ(mission.update(input, 0.6).state, PureMissionState::FOV_RECOVERY);
  input.fov_recovery_required = false;
  input.fov_recovery_complete = false;
  EXPECT_EQ(mission.update(input, 0.7).state, PureMissionState::FOV_RECOVERY);
  input.fov_recovery_complete = true;
  EXPECT_EQ(mission.update(input, 0.8).state, PureMissionState::FOV_RECOVERY);
  EXPECT_EQ(mission.update(input, 1.29).state, PureMissionState::FOV_RECOVERY);
  EXPECT_EQ(mission.update(input, 1.31).state, PureMissionState::INTERCEPT);
}

TEST(PureMission, FovRecoveryMayFireOnlyWhenTheCompleteShotGatePasses)
{
  PureMission mission(immediate_config());
  auto input = enter_intercept(mission);
  input.fov_recovery_required = true;
  ASSERT_EQ(mission.update(input, 0.6).state, PureMissionState::FOV_RECOVERY);

  input.attack_enabled = true;
  input.capture_instantaneous_pass = false;
  EXPECT_FALSE(mission.update(input, 0.61).capture_event);
  EXPECT_EQ(mission.state(), PureMissionState::FOV_RECOVERY);

  input.capture_instantaneous_pass = true;
  const auto shot = mission.update(input, 0.62);
  EXPECT_TRUE(shot.capture_event);
  EXPECT_EQ(shot.state, PureMissionState::EGRESS_REATTACK);
}

TEST(PureMission, ImmCoastAvoidsSearchUntilPredictionExpires)
{
  PureMission mission(immediate_config());
  auto input = enter_intercept(mission);
  input.depth_visible = false;
  input.depth_admission_ready = false;
  input.imm_coast_active = true;
  EXPECT_EQ(mission.update(input, 0.6).state, PureMissionState::IMM_COAST);
  EXPECT_EQ(mission.update(input, 1.0).state, PureMissionState::IMM_COAST);
  input.imm_coast_active = false;
  input.estimate_valid = false;
  input.guidance_valid = false;
  input.search_visible = true;
  EXPECT_EQ(mission.update(input, 1.1).state, PureMissionState::BEARING_TRACK);
}

TEST(PureMission, RecoilEndsBeforeIndependentThreeSecondCooldown)
{
  PureMission mission(immediate_config());
  auto input = enter_intercept(mission);
  input.attack_enabled = true;
  input.capture_instantaneous_pass = true;
  const auto first = mission.update(input, 0.6);
  ASSERT_TRUE(first.capture_event);
  EXPECT_TRUE(first.recoil_active);
  EXPECT_TRUE(first.reattack_cooldown_active);
  input.capture_count = 1;
  EXPECT_EQ(mission.update(input, 0.79).state, PureMissionState::EGRESS_REATTACK);
  const auto resumed = mission.update(input, 0.81);
  EXPECT_EQ(resumed.state, PureMissionState::INTERCEPT);
  EXPECT_FALSE(resumed.recoil_active);
  EXPECT_TRUE(resumed.reattack_cooldown_active);
  EXPECT_NEAR(resumed.reattack_cooldown_remaining_s, 2.79, 1e-9);

  // A continuously passing firing gate must not hold the vehicle in recoil
  // and must not emit again before the independent cooldown expires.
  const auto blocked = mission.update(input, 3.59);
  EXPECT_EQ(blocked.state, PureMissionState::INTERCEPT);
  EXPECT_FALSE(blocked.capture_event);
  EXPECT_TRUE(blocked.reattack_cooldown_active);

  input.capture_instantaneous_pass = true;
  const auto second = mission.update(input, 3.61);
  ASSERT_TRUE(second.capture_event);
  EXPECT_EQ(second.state, PureMissionState::EGRESS_REATTACK);
  input.capture_count = 2;
  input.capture_instantaneous_pass = false;
  EXPECT_EQ(mission.update(input, 3.62).state, PureMissionState::COMPLETE);
}

TEST(PureMission, LostTargetStartsSearchDuringCooldown)
{
  auto config = immediate_config();
  config.camera_loss_hysteresis_s = 0.0;
  PureMission mission(config);
  auto input = enter_intercept(mission);
  input.attack_enabled = true;
  input.capture_instantaneous_pass = true;
  ASSERT_TRUE(mission.update(input, 0.6).capture_event);
  input.capture_count = 1;
  input.capture_instantaneous_pass = false;
  input.search_visible = false;
  input.depth_visible = false;
  input.depth_admission_ready = false;
  input.estimate_valid = false;
  input.guidance_valid = false;
  const auto lost = mission.update(input, 0.81);
  EXPECT_EQ(lost.state, PureMissionState::SEARCH);
  EXPECT_TRUE(lost.reattack_cooldown_active);
}

TEST(PureMission, FovRecoveryStartsDuringCooldown)
{
  PureMission mission(immediate_config());
  auto input = enter_intercept(mission);
  input.attack_enabled = true;
  input.capture_instantaneous_pass = true;
  ASSERT_TRUE(mission.update(input, 0.6).capture_event);
  input.capture_count = 1;
  input.capture_instantaneous_pass = false;
  input.fov_recovery_required = true;
  const auto recovery = mission.update(input, 0.81);
  EXPECT_EQ(recovery.state, PureMissionState::FOV_RECOVERY);
  EXPECT_TRUE(recovery.reattack_cooldown_active);
}

TEST(PureMission, SafetySupervisorOverridesTracking)
{
  PureMission mission(PureMissionConfig{});
  PureMissionInput input;
  input.system_ready = true;
  mission.update(input, 0.0);
  input.safety_abort = true;
  EXPECT_EQ(mission.update(input, 0.1).state, PureMissionState::SAFETY_ABORT);
}

TEST(PureMission, PredictedFovSpikeDoesNotLeaveIntercept)
{
  auto config = immediate_config();
  config.fov_recovery_entry_hold_s = 0.15;
  PureMission mission(config);
  auto input = enter_intercept(mission);
  input.fov_recovery_required = true;
  EXPECT_EQ(mission.update(input, 0.60).state, PureMissionState::INTERCEPT);
  input.fov_recovery_required = false;
  EXPECT_EQ(mission.update(input, 0.70).state, PureMissionState::INTERCEPT);
  input.fov_recovery_required = true;
  EXPECT_EQ(mission.update(input, 0.80).state, PureMissionState::INTERCEPT);
  EXPECT_EQ(mission.update(input, 0.96).state, PureMissionState::FOV_RECOVERY);
}

TEST(PureMission, MeasuredHardFovViolationRemainsImmediate)
{
  auto config = immediate_config();
  config.fov_recovery_entry_hold_s = 0.15;
  PureMission mission(config);
  auto input = enter_intercept(mission);
  input.fov_recovery_required = true;
  input.fov_hard_violation = true;
  EXPECT_EQ(mission.update(input, 0.60).state, PureMissionState::FOV_RECOVERY);
}

TEST(PureMission, OneInvalidTrackingFrameKeepsDepthPrimary)
{
  auto config = immediate_config();
  config.tracking_invalid_hysteresis_s = 0.2;
  PureMission mission(config);
  auto input = enter_intercept(mission);
  input.estimate_valid = false;
  input.guidance_valid = false;
  EXPECT_EQ(mission.update(input, 0.60).state, PureMissionState::INTERCEPT);
  input.estimate_valid = true;
  input.guidance_valid = true;
  EXPECT_EQ(mission.update(input, 0.70).state, PureMissionState::INTERCEPT);
}

TEST(PureMission, PersistentInvalidTrackingFallsBackToDepthAcquireNotSearch)
{
  auto config = immediate_config();
  config.tracking_invalid_hysteresis_s = 0.2;
  PureMission mission(config);
  auto input = enter_intercept(mission);
  input.estimate_valid = false;
  input.guidance_valid = false;
  ASSERT_EQ(mission.update(input, 0.60).state, PureMissionState::INTERCEPT);
  EXPECT_EQ(mission.update(input, 0.81).state, PureMissionState::DEPTH_ACQUIRE);
}

TEST(PureMission, DepthHandoverDoesNotReturnToSearchAtAdmissionEdge)
{
  PureMission mission(immediate_config());
  PureMissionInput input;
  input.system_ready = true;
  mission.update(input, 0.0);
  input.takeoff_ready = true;
  mission.update(input, 0.1);
  input.search_visible = true;
  mission.update(input, 0.2);
  input.depth_visible = true;
  input.depth_admission_ready = true;
  input.depth_sequence = 1;
  ASSERT_EQ(mission.update(input, 0.3).state, PureMissionState::DEPTH_HANDOVER);
  input.depth_admission_ready = false;
  EXPECT_EQ(mission.update(input, 0.4).state, PureMissionState::DEPTH_HANDOVER);
}
