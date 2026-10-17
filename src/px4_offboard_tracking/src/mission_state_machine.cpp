#include "px4_offboard_tracking/mission_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

const char * MissionStateMachine::state_name(MissionState state)
{
  switch (state) {
    case MissionState::B0_INIT: return "B0_INIT";
    case MissionState::B1_WAIT_ESTIMATE: return "B1_WAIT_ESTIMATE";
    case MissionState::B2_TAKEOFF_HOLD: return "B2_TAKEOFF_HOLD";
    case MissionState::B3_APPROACH: return "B3_APPROACH";
    case MissionState::B4_ALIGN: return "B4_ALIGN";
    case MissionState::B5_ALTITUDE_CORRECTION: return "B5_ALTITUDE_CORRECTION";
    case MissionState::B6_LATERAL_CORRECTION: return "B6_LATERAL_CORRECTION";
    case MissionState::B7_QUALITY_GATE: return "B7_QUALITY_GATE";
    case MissionState::B8_CAPTURE_WINDOW: return "B8_CAPTURE_WINDOW";
    case MissionState::B9_VIRTUAL_CAPTURE: return "B9_VIRTUAL_CAPTURE";
    case MissionState::B10_POST_CAPTURE: return "B10_POST_CAPTURE";
    case MissionState::B11_SAFETY_ABORT: return "B11_SAFETY_ABORT";
    case MissionState::B12_RECOVERY: return "B12_RECOVERY";
    case MissionState::B13_SEARCH_SCAN: return "B13_SEARCH_SCAN";
    case MissionState::B14_SEARCH_HANDOVER: return "B14_SEARCH_HANDOVER";
    case MissionState::B15_DEPTH_CONFIRM: return "B15_DEPTH_CONFIRM";
    case MissionState::B16_REATTACK_TRACK: return "B16_REATTACK_TRACK";
    case MissionState::B17_MISSION_COMPLETE: return "B17_MISSION_COMPLETE";
  }
  return "UNKNOWN";
}

void MissionStateMachine::reset(double now_s)
{
  if (!std::isfinite(now_s)) {throw std::invalid_argument("mission time must be finite");}
  state_ = MissionState::B0_INIT;
  correction_parent_ = MissionState::B3_APPROACH;
  state_entered_s_ = now_s;
  transition_reason_ = "reset";
  initialized_ = true;
  transitioned_ = true;
}

void MissionStateMachine::transition(
  MissionState next, const char * reason, double now_s)
{
  if (next == state_) {return;}
  state_ = next;
  state_entered_s_ = now_s;
  transition_reason_ = reason;
  transitioned_ = true;
}

MissionStatus MissionStateMachine::update(const MissionInputs & inputs, double now_s)
{
  if (!std::isfinite(now_s)) {throw std::invalid_argument("mission time must be finite");}
  if (!initialized_) {
    state_entered_s_ = now_s;
    initialized_ = true;
  }
  transitioned_ = false;

  if (inputs.safety_abort && !inputs.mission_complete &&
    state_ != MissionState::B11_SAFETY_ABORT &&
    state_ != MissionState::B12_RECOVERY)
  {
    transition(MissionState::B11_SAFETY_ABORT, "safety abort requested", now_s);
  } else {
    switch (state_) {
      case MissionState::B0_INIT:
        if (inputs.system_ready) {
          transition(MissionState::B1_WAIT_ESTIMATE, "PX4 state is ready", now_s);
        }
        break;
      case MissionState::B1_WAIT_ESTIMATE:
        if (inputs.search_enabled) {
          transition(MissionState::B2_TAKEOFF_HOLD, "search-enabled takeoff requested", now_s);
        } else if (inputs.estimate_valid) {
          transition(MissionState::B2_TAKEOFF_HOLD, "valid target estimate acquired", now_s);
        }
        break;
      case MissionState::B2_TAKEOFF_HOLD:
        if (inputs.search_enabled && inputs.takeoff_ready) {
          transition(
            inputs.search_target_acquired ? MissionState::B14_SEARCH_HANDOVER :
            MissionState::B13_SEARCH_SCAN,
            inputs.search_target_acquired ? "search target already acquired" :
            "takeoff height reached; scanning for target", now_s);
        } else if (!inputs.search_enabled && !inputs.estimate_valid) {
          transition(MissionState::B1_WAIT_ESTIMATE, "target estimate lost", now_s);
        } else if (!inputs.search_enabled && inputs.takeoff_ready && inputs.target_above) {
          transition(MissionState::B3_APPROACH, "takeoff height reached", now_s);
        }
        break;
      case MissionState::B13_SEARCH_SCAN:
        if (inputs.search_target_acquired) {
          transition(MissionState::B14_SEARCH_HANDOVER, "search bearing acquired", now_s);
        }
        break;
      case MissionState::B14_SEARCH_HANDOVER:
        if (!inputs.search_target_acquired && !inputs.depth_observation_visible) {
          transition(MissionState::B13_SEARCH_SCAN, "search bearing lost", now_s);
        } else if (inputs.depth_observation_visible) {
          transition(MissionState::B15_DEPTH_CONFIRM, "target entered depth camera", now_s);
        }
        break;
      case MissionState::B15_DEPTH_CONFIRM:
        if (inputs.depth_confirmed && inputs.estimate_valid && inputs.target_above) {
          transition(MissionState::B3_APPROACH, "depth observation and estimate confirmed", now_s);
        } else if (!inputs.depth_observation_visible && !inputs.search_target_acquired) {
          transition(MissionState::B13_SEARCH_SCAN, "both logical cameras lost target", now_s);
        } else if (!inputs.depth_observation_visible && inputs.search_target_acquired) {
          transition(MissionState::B14_SEARCH_HANDOVER, "depth target lost during confirmation", now_s);
        }
        break;
      case MissionState::B3_APPROACH:
      case MissionState::B4_ALIGN: {
          const MissionState parent = state_;
          if (!inputs.estimate_prediction_valid || !inputs.target_above) {
            transition(
              inputs.search_enabled ?
              (inputs.search_target_acquired ? MissionState::B14_SEARCH_HANDOVER :
              MissionState::B13_SEARCH_SCAN) : MissionState::B1_WAIT_ESTIMATE,
              "target estimate or upward geometry lost", now_s);
          } else if (inputs.altitude_correction_needed) {
            correction_parent_ = parent;
            transition(MissionState::B5_ALTITUDE_CORRECTION, "altitude correction required", now_s);
          } else if (inputs.lateral_correction_needed) {
            correction_parent_ = parent;
            transition(MissionState::B6_LATERAL_CORRECTION, "lateral correction required", now_s);
          } else if (parent == MissionState::B3_APPROACH && inputs.approach_ready &&
            inputs.alignment_feasible && now_s - state_entered_s_ >= 1.0)
          {
            transition(MissionState::B4_ALIGN, "inside configurable aiming corridor", now_s);
          } else if (parent == MissionState::B4_ALIGN && inputs.alignment_ready) {
            transition(MissionState::B7_QUALITY_GATE, "body -Z aligned with predicted LOS", now_s);
          }
        }
        break;
      case MissionState::B5_ALTITUDE_CORRECTION:
        if (!inputs.estimate_prediction_valid) {
          transition(MissionState::B1_WAIT_ESTIMATE, "target estimate lost during correction", now_s);
        } else if (!inputs.altitude_correction_needed) {
          transition(correction_parent_, "altitude correction complete", now_s);
        }
        break;
      case MissionState::B6_LATERAL_CORRECTION:
        if (!inputs.estimate_prediction_valid) {
          transition(MissionState::B1_WAIT_ESTIMATE, "target estimate lost during correction", now_s);
        } else if (!inputs.lateral_correction_needed) {
          transition(correction_parent_, "lateral correction complete", now_s);
        }
        break;
      case MissionState::B7_QUALITY_GATE:
        if (!inputs.estimate_prediction_valid || !inputs.target_above) {
          transition(
            inputs.search_enabled ?
            (inputs.search_target_acquired ? MissionState::B14_SEARCH_HANDOVER :
            MissionState::B13_SEARCH_SCAN) : MissionState::B1_WAIT_ESTIMATE,
            "terminal prediction or upward geometry lost", now_s);
        } else if (inputs.quality_gate_passed) {
          transition(MissionState::B8_CAPTURE_WINDOW, "quality gate passed", now_s);
        }
        break;
      case MissionState::B8_CAPTURE_WINDOW:
        if (!inputs.estimate_prediction_valid || !inputs.target_above) {
          transition(
            inputs.search_enabled ?
            (inputs.search_target_acquired ? MissionState::B14_SEARCH_HANDOVER :
            MissionState::B13_SEARCH_SCAN) : MissionState::B1_WAIT_ESTIMATE,
            "terminal prediction or upward geometry lost", now_s);
        } else if (inputs.capture_window_passed) {
          transition(MissionState::B9_VIRTUAL_CAPTURE, "virtual capture window held", now_s);
        }
        break;
      case MissionState::B9_VIRTUAL_CAPTURE:
        if (inputs.virtual_capture_published) {
          transition(MissionState::B10_POST_CAPTURE, "virtual capture event published", now_s);
        }
        break;
      case MissionState::B10_POST_CAPTURE:
        if (inputs.mission_complete) {
          transition(MissionState::B17_MISSION_COMPLETE, "required captures completed", now_s);
        } else if (inputs.post_capture_recoil_complete) {
          transition(MissionState::B16_REATTACK_TRACK, "post-capture tracking resumed", now_s);
        }
        break;
      case MissionState::B16_REATTACK_TRACK:
        if (!inputs.reattack_ready) {
          break;
        } else if (!inputs.estimate_prediction_valid) {
          transition(
            inputs.search_enabled ?
            (inputs.search_target_acquired ? MissionState::B14_SEARCH_HANDOVER :
            MissionState::B13_SEARCH_SCAN) : MissionState::B1_WAIT_ESTIMATE,
            "reattack estimate lost", now_s);
        } else {
          transition(MissionState::B3_APPROACH, "reattack cooldown complete", now_s);
        }
        break;
      case MissionState::B17_MISSION_COMPLETE:
        break;
      case MissionState::B11_SAFETY_ABORT:
        if (!inputs.safety_abort) {
          transition(MissionState::B12_RECOVERY, "abort condition cleared", now_s);
        }
        break;
      case MissionState::B12_RECOVERY:
        if (inputs.safety_abort) {
          transition(MissionState::B11_SAFETY_ABORT, "unsafe condition returned during recovery", now_s);
        } else if (inputs.recovery_complete) {
          transition(MissionState::B1_WAIT_ESTIMATE, "recovery complete", now_s);
        }
        break;
    }
  }

  return {state_, state_name(state_), transition_reason_,
    std::max(0.0, now_s - state_entered_s_), transitioned_};
}

}  // namespace px4_offboard_tracking
