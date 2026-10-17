#pragma once

#include <cstdint>
#include <string>

namespace px4_offboard_tracking
{

enum class PureMissionState : std::uint8_t
{
  INIT = 0,
  TAKEOFF_SAFE = 1,
  SEARCH = 2,
  BEARING_TRACK = 3,
  BEARING_HANDOVER = BEARING_TRACK,
  DEPTH_HANDOVER = 4,
  DEPTH_ACQUIRE = 5,
  INTERCEPT = 6,
  SHOT_READY = 7,
  CAPTURE_CONFIRM = SHOT_READY,
  EGRESS_REATTACK = 8,
  FOV_RECOVERY = 9,
  IMM_COAST = 10,
  COMPLETE = 11,
  SAFETY_ABORT = 12
};

struct PureMissionConfig
{
  double search_acquisition_hold_s{0.2};
  double depth_acquisition_hold_s{0.2};
  double camera_loss_hysteresis_s{0.6};
  double depth_loss_hysteresis_s{0.2};
  std::uint32_t minimum_depth_acquisition_frames{4};
  double estimate_stability_hold_s{1.0};
  double capture_hold_s{0.15};
  // Predicted FOV excursions and tracking-validity gaps must persist before
  // changing mission phase. A measured hard-FOV violation remains immediate.
  double fov_recovery_entry_hold_s{0.15};
  double tracking_invalid_hysteresis_s{0.2};
  // Keep the predicted error below the 6 degree exit boundary for one full
  // 0.5 s prediction horizon before leaving FOV_RECOVERY.
  double fov_recovery_exit_hold_s{0.5};
  double recoil_duration_s{0.2};
  double reattack_cooldown_s{3.0};
  std::uint32_t required_captures{2};
};

struct PureMissionInput
{
  bool system_ready{false};
  bool takeoff_ready{false};
  bool mission_enabled{true};
  bool search_visible{false};
  bool depth_visible{false};
  bool depth_admission_ready{false};
  bool estimate_valid{false};
  bool guidance_valid{false};
  bool fov_recovery_required{false};
  bool fov_hard_violation{false};
  bool fov_recovery_complete{false};
  bool imm_coast_active{false};
  bool attack_enabled{false};
  bool capture_instantaneous_pass{false};
  bool safety_abort{false};
  bool recovery_ready{false};
  std::uint32_t search_sequence{0};
  std::uint32_t depth_sequence{0};
  std::uint32_t capture_count{0};
};

struct PureMissionResult
{
  PureMissionState state{PureMissionState::INIT};
  std::string state_name{"INIT"};
  std::string transition_reason{"initial state"};
  double state_elapsed_s{};
  bool capture_event{false};
  bool transitioned{false};
  bool recoil_active{false};
  bool reattack_cooldown_active{false};
  double reattack_cooldown_remaining_s{};
};

class PureMission
{
public:
  explicit PureMission(const PureMissionConfig & config);

  PureMissionResult update(const PureMissionInput & input, double now_s);
  PureMissionState state() const;
  void reset(double now_s = 0.0);

  static const char * state_name(PureMissionState state);

private:
  void transition(PureMissionState next, const char * reason, double now_s);
  void reset_hold_timers();

  PureMissionConfig config_;
  PureMissionState state_{PureMissionState::INIT};
  std::string reason_{"initial state"};
  double state_entered_s_{};
  double search_visible_since_s_{-1.0};
  double depth_visible_since_s_{-1.0};
  double estimate_valid_since_s_{-1.0};
  double capture_pass_since_s_{-1.0};
  double depth_unavailable_since_s_{-1.0};
  double fov_recovery_required_since_s_{-1.0};
  double tracking_invalid_since_s_{-1.0};
  double fov_recovery_complete_since_s_{-1.0};
  double last_search_visible_s_{-1.0};
  double last_depth_visible_s_{-1.0};
  double last_tracking_valid_s_{-1.0};
  double last_capture_time_s_{-1.0};
  std::uint32_t last_depth_sequence_{0};
  std::uint32_t depth_acquisition_frames_{0};
  bool transitioned_{false};
  bool capture_event_{false};
  PureMissionState resume_state_{PureMissionState::DEPTH_HANDOVER};
};

}  // namespace px4_offboard_tracking
