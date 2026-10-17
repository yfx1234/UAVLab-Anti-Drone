#pragma once

#include <cstdint>
#include <string>

namespace px4_offboard_tracking
{

enum class MissionState : std::uint8_t
{
  B0_INIT = 0,
  B1_WAIT_ESTIMATE = 1,
  B2_TAKEOFF_HOLD = 2,
  B3_APPROACH = 3,
  B4_ALIGN = 4,
  B5_ALTITUDE_CORRECTION = 5,
  B6_LATERAL_CORRECTION = 6,
  B7_QUALITY_GATE = 7,
  B8_CAPTURE_WINDOW = 8,
  B9_VIRTUAL_CAPTURE = 9,
  B10_POST_CAPTURE = 10,
  B11_SAFETY_ABORT = 11,
  B12_RECOVERY = 12,
  B13_SEARCH_SCAN = 13,
  B14_SEARCH_HANDOVER = 14,
  B15_DEPTH_CONFIRM = 15,
  B16_REATTACK_TRACK = 16,
  B17_MISSION_COMPLETE = 17,
};

struct MissionInputs
{
  bool system_ready{false};
  bool estimate_valid{false};
  bool estimate_prediction_valid{false};
  bool search_enabled{false};
  bool search_target_acquired{false};
  bool depth_observation_visible{false};
  bool depth_confirmed{false};
  bool takeoff_ready{false};
  bool target_above{false};
  bool approach_ready{false};
  bool alignment_ready{false};
  bool altitude_correction_needed{false};
  bool lateral_correction_needed{false};
  bool alignment_feasible{false};
  bool quality_gate_passed{false};
  bool capture_window_passed{false};
  bool virtual_capture_published{false};
  bool post_capture_recoil_complete{false};
  bool reattack_ready{false};
  bool mission_complete{false};
  bool safety_abort{false};
  bool recovery_complete{false};
};

struct MissionStatus
{
  MissionState state{MissionState::B0_INIT};
  std::string state_name{"B0_INIT"};
  std::string transition_reason{"initialized"};
  double state_elapsed_s{};
  bool transitioned{false};
};

class MissionStateMachine
{
public:
  MissionStatus update(const MissionInputs & inputs, double now_s);
  void reset(double now_s = 0.0);

  MissionState state() const {return state_;}
  static const char * state_name(MissionState state);

private:
  void transition(MissionState next, const char * reason, double now_s);

  MissionState state_{MissionState::B0_INIT};
  MissionState correction_parent_{MissionState::B3_APPROACH};
  double state_entered_s_{};
  std::string transition_reason_{"initialized"};
  bool initialized_{false};
  bool transitioned_{false};
};

}  // namespace px4_offboard_tracking
