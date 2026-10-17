#include "px4_offboard_tracking/pure_mission.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

PureMission::PureMission(const PureMissionConfig & config)
: config_(config)
{
  if (config_.search_acquisition_hold_s < 0.0 ||
    config_.depth_acquisition_hold_s < 0.0 ||
    config_.camera_loss_hysteresis_s < 0.0 ||
    config_.depth_loss_hysteresis_s < 0.0 ||
    config_.minimum_depth_acquisition_frames == 0 ||
    config_.estimate_stability_hold_s < 0.0 ||
    config_.capture_hold_s < 0.0 ||
    config_.fov_recovery_entry_hold_s < 0.0 ||
    config_.tracking_invalid_hysteresis_s < 0.0 ||
    config_.fov_recovery_exit_hold_s < 0.0 ||
    config_.recoil_duration_s < 0.0 ||
    config_.reattack_cooldown_s < 0.0 ||
    config_.required_captures == 0)
  {
    throw std::invalid_argument("invalid pure mission configuration");
  }
}

const char * PureMission::state_name(PureMissionState state)
{
  switch (state) {
    case PureMissionState::INIT: return "INIT";
    case PureMissionState::TAKEOFF_SAFE: return "TAKEOFF_SAFE";
    case PureMissionState::SEARCH: return "SEARCH";
    case PureMissionState::BEARING_TRACK: return "BEARING_TRACK";
    case PureMissionState::DEPTH_HANDOVER: return "DEPTH_HANDOVER";
    case PureMissionState::DEPTH_ACQUIRE: return "DEPTH_ACQUIRE";
    case PureMissionState::INTERCEPT: return "INTERCEPT";
    case PureMissionState::SHOT_READY: return "SHOT_READY";
    case PureMissionState::EGRESS_REATTACK: return "EGRESS_REATTACK";
    case PureMissionState::FOV_RECOVERY: return "FOV_RECOVERY";
    case PureMissionState::IMM_COAST: return "IMM_COAST";
    case PureMissionState::COMPLETE: return "COMPLETE";
    case PureMissionState::SAFETY_ABORT: return "SAFETY_ABORT";
  }
  return "UNKNOWN";
}

void PureMission::reset_hold_timers()
{
  search_visible_since_s_ = -1.0;
  depth_visible_since_s_ = -1.0;
  estimate_valid_since_s_ = -1.0;
  capture_pass_since_s_ = -1.0;
  depth_unavailable_since_s_ = -1.0;
  fov_recovery_required_since_s_ = -1.0;
  tracking_invalid_since_s_ = -1.0;
  fov_recovery_complete_since_s_ = -1.0;
  depth_acquisition_frames_ = 0;
  last_depth_sequence_ = 0;
}

void PureMission::transition(
  PureMissionState next, const char * reason, double now_s)
{
  if (state_ == next) {
    return;
  }
  state_ = next;
  reason_ = reason;
  state_entered_s_ = now_s;
  transitioned_ = true;
  reset_hold_timers();
}

PureMissionResult PureMission::update(
  const PureMissionInput & input, double now_s)
{
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("mission time must be finite");
  }
  transitioned_ = false;
  capture_event_ = false;
  if (input.search_visible) {
    last_search_visible_s_ = now_s;
  }
  if (input.depth_visible) {
    last_depth_visible_s_ = now_s;
    if (input.depth_sequence != 0 &&
      input.depth_sequence != last_depth_sequence_)
    {
      last_depth_sequence_ = input.depth_sequence;
      ++depth_acquisition_frames_;
    }
  }
  if (input.depth_visible && input.estimate_valid && input.guidance_valid) {
    last_tracking_valid_s_ = now_s;
  }

  if (input.safety_abort && state_ != PureMissionState::SAFETY_ABORT &&
    state_ != PureMissionState::COMPLETE)
  {
    transition(
      PureMissionState::SAFETY_ABORT,
      "safety supervisor requested abort", now_s);
  }

  auto hold_passed = [now_s](bool condition, double & since, double duration) {
      if (!condition) {
        since = -1.0;
        return false;
      }
      if (since < 0.0) {
        since = now_s;
      }
      return now_s - since >= duration;
    };
  auto perception_lost = [&]() {
      const double last_visible =
        std::max(last_search_visible_s_, last_depth_visible_s_);
      return !input.search_visible && !input.depth_visible &&
             (last_visible < 0.0 ||
             now_s - last_visible > config_.camera_loss_hysteresis_s);
    };
  auto reattack_ready = [&]() {
      return last_capture_time_s_ < 0.0 ||
             now_s - last_capture_time_s_ >= config_.reattack_cooldown_s;
    };
  auto fov_recovery_confirmed = [&]() {
      return input.fov_hard_violation || hold_passed(
        input.fov_recovery_required, fov_recovery_required_since_s_,
        config_.fov_recovery_entry_hold_s);
    };
  auto depth_loss_confirmed = [&]() {
      return hold_passed(
        !input.depth_visible, depth_unavailable_since_s_,
        config_.depth_loss_hysteresis_s);
    };
  auto tracking_invalid_confirmed = [&]() {
      return hold_passed(
        !input.guidance_valid || !input.estimate_valid,
        tracking_invalid_since_s_, config_.tracking_invalid_hysteresis_s);
    };

  switch (state_) {
    case PureMissionState::INIT:
      if (input.system_ready) {
        transition(
          PureMissionState::TAKEOFF_SAFE,
          "PX4 local state and attitude ready", now_s);
      }
      break;
    case PureMissionState::TAKEOFF_SAFE:
      if (input.takeoff_ready && input.mission_enabled) {
        transition(
          PureMissionState::SEARCH,
          "safe takeoff altitude reached and mission enabled", now_s);
      }
      break;
    case PureMissionState::SEARCH:
      if (hold_passed(
          input.search_visible, search_visible_since_s_,
          config_.search_acquisition_hold_s))
      {
        transition(
          PureMissionState::BEARING_TRACK,
          "search bearing acquired", now_s);
      }
      break;
    case PureMissionState::BEARING_TRACK:
      if (perception_lost()) {
        transition(PureMissionState::SEARCH, "both cameras lost target", now_s);
      } else if (input.depth_admission_ready) {
        transition(
          PureMissionState::DEPTH_HANDOVER,
          "depth entered preferred handover region", now_s);
      }
      break;
    case PureMissionState::DEPTH_HANDOVER:
      // Handover already uses the measured camera axis and the shared
      // acceleration/FOV constraints. Keep acquiring instead of turning a
      // predicted boundary crossing into a high-level phase reset.
      if (perception_lost()) {
        transition(PureMissionState::SEARCH, "depth handover lost target", now_s);
      } else if (depth_loss_confirmed() && input.search_visible) {
        transition(
          PureMissionState::BEARING_TRACK,
          "depth loss confirmed; search bearing remains available", now_s);
      } else if (hold_passed(
          input.depth_admission_ready, depth_visible_since_s_,
          config_.depth_acquisition_hold_s) &&
        depth_acquisition_frames_ >= config_.minimum_depth_acquisition_frames)
      {
        transition(
          PureMissionState::DEPTH_ACQUIRE,
          "depth handover stable", now_s);
      }
      break;
    case PureMissionState::DEPTH_ACQUIRE:
      // FOV centering is a continuous guidance constraint while the estimate
      // is stabilizing. Do not restart acquisition solely because the
      // prediction touches the recovery boundary.
      if (!input.depth_visible && input.imm_coast_active) {
        resume_state_ = PureMissionState::DEPTH_ACQUIRE;
        transition(
          PureMissionState::IMM_COAST,
          "bounded IMM coast started", now_s);
      } else if (perception_lost()) {
        transition(PureMissionState::SEARCH, "depth acquisition lost target", now_s);
      } else if (depth_loss_confirmed() && input.search_visible) {
        transition(
          PureMissionState::BEARING_TRACK,
          "depth loss confirmed during acquisition", now_s);
      } else if (hold_passed(
          input.estimate_valid, estimate_valid_since_s_,
          config_.estimate_stability_hold_s))
      {
        transition(
          PureMissionState::INTERCEPT,
          "relative estimate stable", now_s);
      }
      break;
    case PureMissionState::INTERCEPT:
      if (fov_recovery_confirmed()) {
        resume_state_ = PureMissionState::INTERCEPT;
        transition(
          PureMissionState::FOV_RECOVERY,
          "predicted FOV barrier active", now_s);
      } else if (!input.depth_visible && input.imm_coast_active) {
        resume_state_ = PureMissionState::INTERCEPT;
        transition(
          PureMissionState::IMM_COAST,
          "bounded IMM coast started", now_s);
      } else if (tracking_invalid_confirmed()) {
        if (input.depth_visible) {
          transition(
            PureMissionState::DEPTH_ACQUIRE,
            "tracking invalidity persisted; depth remains primary", now_s);
        } else {
          transition(
            input.search_visible ? PureMissionState::BEARING_TRACK :
            PureMissionState::SEARCH,
            "guidance or estimate unavailable beyond hysteresis", now_s);
        }
      } else if (input.attack_enabled && input.capture_instantaneous_pass &&
        reattack_ready())
      {
        // The shot-timing optimizer upstream has already enforced the stable
        // one-degree hold and forward prediction. Emit in this same control
        // cycle; an extra SHOT_READY cycle can miss a narrow optimum at speed.
        capture_event_ = true;
        last_capture_time_s_ = now_s;
        transition(
          PureMissionState::EGRESS_REATTACK,
          "optimized predictive shot emitted", now_s);
      }
      break;
    case PureMissionState::SHOT_READY:
      if (!input.attack_enabled || !input.capture_instantaneous_pass) {
        transition(
          PureMissionState::INTERCEPT,
          "predictive shot gates left", now_s);
      } else if (reattack_ready()) {
        capture_event_ = true;
        last_capture_time_s_ = now_s;
        transition(
          PureMissionState::EGRESS_REATTACK,
          "predictive shot emitted", now_s);
      }
      break;
    case PureMissionState::FOV_RECOVERY:
      if (input.attack_enabled && input.capture_instantaneous_pass &&
        reattack_ready())
      {
        // FOV_RECOVERY is a continuous centering mode, not a declaration that
        // the target is unusable.  The upstream hard gate still requires a
        // fresh depth frame, accepted estimate, valid ballistic solution and
        // stable one-degree shot timing.  Fire immediately when those stricter
        // conditions pass instead of waiting an extra recovery-state cycle and
        // missing a short head-on crossing window.
        capture_event_ = true;
        last_capture_time_s_ = now_s;
        transition(
          PureMissionState::EGRESS_REATTACK,
          "optimized predictive shot emitted during FOV centering", now_s);
      } else if (depth_loss_confirmed() && !input.search_visible &&
        input.imm_coast_active)
      {
        transition(
          PureMissionState::IMM_COAST,
          "depth loss persisted during FOV recovery", now_s);
      } else if (hold_passed(
          input.depth_visible && input.fov_recovery_complete,
          fov_recovery_complete_since_s_,
          config_.fov_recovery_exit_hold_s))
      {
        transition(
          input.estimate_valid ? resume_state_ :
          PureMissionState::DEPTH_HANDOVER,
          "FOV recovered below six degrees", now_s);
      } else if (perception_lost() && !input.estimate_valid) {
        transition(PureMissionState::SEARCH, "all perception and IMM lost", now_s);
      }
      break;
    case PureMissionState::IMM_COAST:
      if (input.depth_visible) {
        if (input.fov_hard_violation || input.fov_recovery_required) {
          resume_state_ = input.estimate_valid ?
            PureMissionState::INTERCEPT : PureMissionState::DEPTH_ACQUIRE;
          transition(
            PureMissionState::FOV_RECOVERY,
            "depth reacquired outside recovered FOV", now_s);
        } else {
          transition(
            input.estimate_valid && input.guidance_valid ?
            PureMissionState::INTERCEPT : PureMissionState::DEPTH_ACQUIRE,
            "depth reacquired inside usable FOV", now_s);
        }
      } else if (!input.estimate_valid) {
        transition(
          input.search_visible ? PureMissionState::BEARING_TRACK :
          PureMissionState::SEARCH,
          "IMM timeout or covariance exceeded", now_s);
      }
      break;
    case PureMissionState::EGRESS_REATTACK:
      if (input.capture_count >= config_.required_captures) {
        transition(
          PureMissionState::COMPLETE,
          "required captures completed", now_s);
      } else if (now_s - state_entered_s_ >= config_.recoil_duration_s) {
        if (fov_recovery_confirmed()) {
          resume_state_ = input.estimate_valid && input.guidance_valid ?
            PureMissionState::INTERCEPT : PureMissionState::DEPTH_ACQUIRE;
          transition(
            PureMissionState::FOV_RECOVERY,
            "recoil complete; predicted FOV recovery required", now_s);
        } else if (perception_lost()) {
          transition(
            PureMissionState::SEARCH,
            "recoil complete; target lost during reattack cooldown", now_s);
        } else if (input.depth_visible && input.estimate_valid &&
          input.guidance_valid)
        {
          transition(
            PureMissionState::INTERCEPT,
            "recoil complete; tracking resumes during reattack cooldown", now_s);
        } else if (input.depth_visible) {
          transition(
            input.depth_admission_ready ? PureMissionState::DEPTH_ACQUIRE :
            (input.search_visible ? PureMissionState::BEARING_TRACK :
            PureMissionState::DEPTH_HANDOVER),
            "recoil complete; depth reacquisition resumes", now_s);
        } else {
          transition(
            input.search_visible ? PureMissionState::BEARING_TRACK :
            PureMissionState::SEARCH,
            "recoil complete; visual reacquisition resumes", now_s);
        }
      }
      break;
    case PureMissionState::COMPLETE:
      break;
    case PureMissionState::SAFETY_ABORT:
      if (!input.safety_abort && input.recovery_ready) {
        transition(PureMissionState::SEARCH, "safety state recovered", now_s);
      }
      break;
  }

  const bool cooldown_active =
    last_capture_time_s_ >= 0.0 && !reattack_ready();
  const double cooldown_remaining_s = cooldown_active ?
    std::max(
    0.0, config_.reattack_cooldown_s - (now_s - last_capture_time_s_)) : 0.0;
  return {
    state_, state_name(state_), reason_,
    std::max(0.0, now_s - state_entered_s_),
    capture_event_, transitioned_,
    state_ == PureMissionState::EGRESS_REATTACK,
    cooldown_active, cooldown_remaining_s};
}

PureMissionState PureMission::state() const
{
  return state_;
}

void PureMission::reset(double now_s)
{
  state_ = PureMissionState::INIT;
  reason_ = "reset";
  state_entered_s_ = now_s;
  transitioned_ = false;
  capture_event_ = false;
  last_search_visible_s_ = -1.0;
  last_depth_visible_s_ = -1.0;
  last_tracking_valid_s_ = -1.0;
  last_capture_time_s_ = -1.0;
  resume_state_ = PureMissionState::DEPTH_HANDOVER;
  reset_hold_timers();
}

}  // namespace px4_offboard_tracking
