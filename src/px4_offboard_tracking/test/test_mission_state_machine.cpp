#include <gtest/gtest.h>

#include "px4_offboard_tracking/mission_state_machine.hpp"

using px4_offboard_tracking::MissionInputs;
using px4_offboard_tracking::MissionState;
using px4_offboard_tracking::MissionStateMachine;

namespace
{
MissionInputs nominal()
{
  MissionInputs input;
  input.system_ready = true;
  input.estimate_valid = true;
  input.estimate_prediction_valid = true;
  input.takeoff_ready = true;
  input.target_above = true;
  input.approach_ready = true;
  input.alignment_feasible = true;
  return input;
}

TEST(MissionStateMachine, CoastsInApproachButNeverCapturesWithoutFreshEstimate)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  input.estimate_valid = false;
  EXPECT_EQ(machine.update(input, 0.3).state, MissionState::B3_APPROACH);
  input.estimate_prediction_valid = false;
  EXPECT_EQ(machine.update(input, 0.4).state, MissionState::B1_WAIT_ESTIMATE);
}
}  // namespace

TEST(MissionStateMachine, ReachesQualityGateButDoesNotBypassFutureMilestones)
{
  MissionStateMachine machine;
  auto input = nominal();
  EXPECT_EQ(machine.update(input, 0.0).state, MissionState::B1_WAIT_ESTIMATE);
  EXPECT_EQ(machine.update(input, 0.1).state, MissionState::B2_TAKEOFF_HOLD);
  EXPECT_EQ(machine.update(input, 0.2).state, MissionState::B3_APPROACH);
  EXPECT_EQ(machine.update(input, 0.3).state, MissionState::B3_APPROACH);
  EXPECT_EQ(machine.update(input, 1.2).state, MissionState::B4_ALIGN);
  input.alignment_ready = true;
  EXPECT_EQ(machine.update(input, 1.3).state, MissionState::B7_QUALITY_GATE);
  EXPECT_EQ(machine.update(input, 1.4).state, MissionState::B7_QUALITY_GATE);
  input.quality_gate_passed = true;
  EXPECT_EQ(machine.update(input, 1.5).state, MissionState::B8_CAPTURE_WINDOW);
}

TEST(MissionStateMachine, PublishedCaptureHoldsRecoilBeforeRecovery)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.alignment_ready = true;
  machine.update(input, 1.3);
  input.quality_gate_passed = true;
  machine.update(input, 1.4);
  input.capture_window_passed = true;
  EXPECT_EQ(machine.update(input, 2.0).state, MissionState::B9_VIRTUAL_CAPTURE);
  input.virtual_capture_published = true;
  EXPECT_EQ(machine.update(input, 2.1).state, MissionState::B10_POST_CAPTURE);
  EXPECT_EQ(machine.update(input, 2.2).state, MissionState::B10_POST_CAPTURE);
  input.post_capture_recoil_complete = true;
  EXPECT_EQ(machine.update(input, 2.2).state, MissionState::B16_REATTACK_TRACK);
}

TEST(MissionStateMachine, PostCaptureRecoveryReturnsThroughAcquisitionStates)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.alignment_ready = true;
  machine.update(input, 1.3);
  input.quality_gate_passed = true;
  machine.update(input, 1.4);
  input.capture_window_passed = true;
  machine.update(input, 2.0);
  input.virtual_capture_published = true;
  machine.update(input, 2.1);
  input.post_capture_recoil_complete = true;
  EXPECT_EQ(machine.update(input, 2.2).state, MissionState::B16_REATTACK_TRACK);
  input.reattack_ready = true;
  EXPECT_EQ(machine.update(input, 5.2).state, MissionState::B3_APPROACH);
}

TEST(MissionStateMachine, ReattackCooldownCannotBeBypassedByEstimateDropout)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.alignment_ready = true;
  machine.update(input, 1.3);
  input.quality_gate_passed = true;
  machine.update(input, 1.4);
  input.capture_window_passed = true;
  machine.update(input, 2.0);
  input.virtual_capture_published = true;
  machine.update(input, 2.1);
  input.post_capture_recoil_complete = true;
  EXPECT_EQ(machine.update(input, 2.2).state, MissionState::B16_REATTACK_TRACK);

  input.estimate_valid = false;
  input.estimate_prediction_valid = false;
  input.reattack_ready = false;
  EXPECT_EQ(machine.update(input, 3.0).state, MissionState::B16_REATTACK_TRACK);

  input.reattack_ready = true;
  EXPECT_EQ(machine.update(input, 5.2).state, MissionState::B1_WAIT_ESTIMATE);
}

TEST(MissionStateMachine, SearchAcquiresBearingThenConfirmsDepth)
{
  MissionStateMachine machine;
  auto input = nominal();
  input.search_enabled = true;
  input.estimate_valid = false;
  input.estimate_prediction_valid = false;
  EXPECT_EQ(machine.update(input, 0.0).state, MissionState::B1_WAIT_ESTIMATE);
  EXPECT_EQ(machine.update(input, 0.1).state, MissionState::B2_TAKEOFF_HOLD);
  EXPECT_EQ(machine.update(input, 0.2).state, MissionState::B13_SEARCH_SCAN);
  input.search_target_acquired = true;
  EXPECT_EQ(machine.update(input, 0.3).state, MissionState::B14_SEARCH_HANDOVER);
  input.depth_observation_visible = true;
  EXPECT_EQ(machine.update(input, 0.4).state, MissionState::B15_DEPTH_CONFIRM);
  input.depth_confirmed = true;
  input.estimate_valid = true;
  input.estimate_prediction_valid = true;
  EXPECT_EQ(machine.update(input, 0.7).state, MissionState::B3_APPROACH);
}

TEST(MissionStateMachine, SearchRestartsWhenBothCamerasLoseTarget)
{
  MissionStateMachine machine;
  auto input = nominal();
  input.search_enabled = true;
  input.search_target_acquired = true;
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  EXPECT_EQ(machine.update(input, 0.2).state, MissionState::B14_SEARCH_HANDOVER);
  input.search_target_acquired = false;
  EXPECT_EQ(machine.update(input, 0.3).state, MissionState::B13_SEARCH_SCAN);
}

TEST(MissionStateMachine, CompletedCaptureMissionTransitionsToCompleteState)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.alignment_ready = true;
  machine.update(input, 1.3);
  input.quality_gate_passed = true;
  machine.update(input, 1.4);
  input.capture_window_passed = true;
  machine.update(input, 2.0);
  input.virtual_capture_published = true;
  input.mission_complete = true;
  EXPECT_EQ(machine.update(input, 2.1).state, MissionState::B10_POST_CAPTURE);
  EXPECT_EQ(machine.update(input, 2.2).state, MissionState::B17_MISSION_COMPLETE);
}

TEST(MissionStateMachine, SafetyRecoveryStillReturnsThroughAcquisitionStates)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  input.safety_abort = true;
  machine.update(input, 0.1);
  input.safety_abort = false;
  EXPECT_EQ(machine.update(input, 0.2).state, MissionState::B12_RECOVERY);
  input.recovery_complete = true;
  EXPECT_EQ(machine.update(input, 0.3).state, MissionState::B1_WAIT_ESTIMATE);
}

TEST(MissionStateMachine, CompletedCaptureMissionHoldsPostCaptureState)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.alignment_ready = true;
  machine.update(input, 1.3);
  input.quality_gate_passed = true;
  machine.update(input, 1.4);
  input.capture_window_passed = true;
  EXPECT_EQ(machine.update(input, 2.0).state, MissionState::B9_VIRTUAL_CAPTURE);
  input.virtual_capture_published = true;
  input.mission_complete = true;
  EXPECT_EQ(machine.update(input, 2.1).state, MissionState::B10_POST_CAPTURE);
  EXPECT_EQ(machine.update(input, 2.2).state, MissionState::B17_MISSION_COMPLETE);
  input.safety_abort = true;
  EXPECT_EQ(machine.update(input, 3.0).state, MissionState::B17_MISSION_COMPLETE);
}

TEST(MissionStateMachine, CorrectionSubmodeReturnsToItsParent)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.lateral_correction_needed = true;
  EXPECT_EQ(machine.update(input, 1.3).state, MissionState::B6_LATERAL_CORRECTION);
  input.lateral_correction_needed = false;
  EXPECT_EQ(machine.update(input, 1.4).state, MissionState::B4_ALIGN);
}

TEST(MissionStateMachine, SafetyAbortHasPriorityAndRecovers)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  input.safety_abort = true;
  EXPECT_EQ(machine.update(input, 0.1).state, MissionState::B11_SAFETY_ABORT);
  input.safety_abort = false;
  EXPECT_EQ(machine.update(input, 0.2).state, MissionState::B12_RECOVERY);
  input.recovery_complete = true;
  EXPECT_EQ(machine.update(input, 0.3).state, MissionState::B1_WAIT_ESTIMATE);
}

TEST(MissionStateMachine, UnsafeRecoveryReturnsToAbort)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  input.safety_abort = true;
  machine.update(input, 0.1);
  input.safety_abort = false;
  EXPECT_EQ(machine.update(input, 0.2).state, MissionState::B12_RECOVERY);
  input.safety_abort = true;
  EXPECT_EQ(machine.update(input, 0.3).state, MissionState::B11_SAFETY_ABORT);
}

TEST(MissionStateMachine, TerminalGuidanceStaysLatchedAcrossFeasibilityDropout)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.alignment_feasible = false;
  EXPECT_EQ(machine.update(input, 1.3).state, MissionState::B4_ALIGN);
  EXPECT_EQ(machine.update(input, 1.7).state, MissionState::B4_ALIGN);
}

TEST(MissionStateMachine, CaptureWindowQualityDropoutOnlyResetsCaptureEvidence)
{
  MissionStateMachine machine;
  auto input = nominal();
  machine.update(input, 0.0);
  machine.update(input, 0.1);
  machine.update(input, 0.2);
  machine.update(input, 1.2);
  input.alignment_ready = true;
  machine.update(input, 1.3);
  input.quality_gate_passed = true;
  EXPECT_EQ(machine.update(input, 1.4).state, MissionState::B8_CAPTURE_WINDOW);

  input.estimate_valid = false;
  input.quality_gate_passed = false;
  EXPECT_EQ(machine.update(input, 1.5).state, MissionState::B8_CAPTURE_WINDOW);

  input.estimate_prediction_valid = false;
  EXPECT_EQ(machine.update(input, 1.6).state, MissionState::B1_WAIT_ESTIMATE);
}
