#include <falcon_intercept_msgs/msg/capture_window_status.hpp>
#include <falcon_intercept_msgs/msg/command_proposal.hpp>
#include <falcon_intercept_msgs/msg/mission_status.hpp>
#include <falcon_intercept_msgs/msg/safety_status.hpp>
#include <falcon_intercept_msgs/msg/search_command.hpp>
#include <falcon_intercept_msgs/msg/target_track.hpp>
#include <falcon_intercept_msgs/msg/vehicle_interface_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace falcon_intercept_control
{
class InterceptFsmNode final : public rclcpp::Node
{
public:
  InterceptFsmNode() : Node("intercept_fsm_node")
  {
    minimum_track_hits_ = declare_parameter<int>("minimum_track_hits", 5);
    maximum_reprojection_error_px_ = declare_parameter<double>("maximum_reprojection_error_px", 3.0);
    minimum_geometry_score_ = declare_parameter<double>("minimum_geometry_score", 0.10);
    maximum_baseline_motion_ratio_ = declare_parameter<double>("maximum_baseline_motion_ratio", 0.10);
    alignment_gate_rad_ = declare_parameter<double>("alignment_gate_rad", 0.034906585);
    alignment_hold_s_ = declare_parameter<double>("alignment_hold_s", 0.15);
    auto_start_ = declare_parameter<bool>("auto_start_after_offboard", true);
    vehicle_sub_ = create_subscription<falcon_intercept_msgs::msg::VehicleInterfaceStatus>(
      "/control/px4_status", 10, [this](falcon_intercept_msgs::msg::VehicleInterfaceStatus::ConstSharedPtr m) {vehicle_ = *m; has_vehicle_ = true; vehicle_received_s_ = get_clock()->now().seconds();});
    track_sub_ = create_subscription<falcon_intercept_msgs::msg::TargetTrack>(
      "/target/track", rclcpp::SensorDataQoS().keep_last(1),
      [this](falcon_intercept_msgs::msg::TargetTrack::ConstSharedPtr m) {track_ = *m; has_track_ = true; track_received_s_ = get_clock()->now().seconds();});
    search_sub_ = create_subscription<falcon_intercept_msgs::msg::SearchCommand>(
      "/decision/search_command", 10, [this](falcon_intercept_msgs::msg::SearchCommand::ConstSharedPtr m) {search_ = *m; has_search_ = true;});
    guidance_sub_ = create_subscription<falcon_intercept_msgs::msg::CommandProposal>(
      "/decision/guidance_proposal", 10, [this](falcon_intercept_msgs::msg::CommandProposal::ConstSharedPtr m) {guidance_ = *m; has_guidance_ = true;});
    safety_sub_ = create_subscription<falcon_intercept_msgs::msg::SafetyStatus>(
      "/decision/safety_status", 10, [this](falcon_intercept_msgs::msg::SafetyStatus::ConstSharedPtr m) {safety_ = *m; has_safety_ = true;});
    capture_sub_ = create_subscription<falcon_intercept_msgs::msg::CaptureWindowStatus>(
      "/decision/capture_window", 10, [this](falcon_intercept_msgs::msg::CaptureWindowStatus::ConstSharedPtr m) {capture_ = *m; has_capture_ = true;});
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/intercept/reset", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {if (m->data) {reset_requested_ = true;}});
    status_pub_ = create_publisher<falcon_intercept_msgs::msg::MissionStatus>(
      "/decision/mission_status", 10);
    proposal_pub_ = create_publisher<falcon_intercept_msgs::msg::CommandProposal>(
      "/decision/command_proposal", 10);
    fire_pub_ = create_publisher<std_msgs::msg::String>("/intercept/fire_event", 10);
    state_started_s_ = get_clock()->now().seconds();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  using Mission = falcon_intercept_msgs::msg::MissionStatus;
  void transition(std::uint8_t next, const std::string & reason, double now)
  {
    if (state_ == next) {return;}
    state_ = next; state_started_s_ = now; transitioned_ = true; reason_ = reason;
    if (next != Mission::STATE_TRACK) {aligned_since_s_ = -1.0;}
  }

  double target_age(double now) const
  {
    if (!has_track_) {return INFINITY;}
    const double source_age = now - rclcpp::Time(track_.header.stamp).seconds();
    return std::max({static_cast<double>(track_.age_s), now - track_received_s_, source_age});
  }

  bool target_confirmed(double now) const
  {
    return has_track_ && track_.valid && target_age(now) <= 0.20 &&
      static_cast<int>(track_.consecutive_hits) >= minimum_track_hits_ &&
      track_.timing_valid && track_.range_observable &&
      std::isfinite(track_.reprojection_error_px) &&
      track_.reprojection_error_px <= maximum_reprojection_error_px_ &&
      std::isfinite(track_.measurement_geometry_score) &&
      track_.measurement_geometry_score >= minimum_geometry_score_ &&
      std::isfinite(track_.position_camera.x) &&
      std::isfinite(track_.position_camera.y) &&
      std::isfinite(track_.position_camera.z) &&
      std::isfinite(track_.velocity_camera.z) &&
      std::abs(track_.velocity_camera.x) * track_.timing_delta_s / 0.1 <=
      maximum_baseline_motion_ratio_;
  }

  double alignment_error() const
  {
    if (!has_track_) {return INFINITY;}
    return std::atan2(std::hypot(track_.position_camera.x, track_.position_camera.y),
      track_.position_camera.z);
  }

  void tick()
  {
    const double now = get_clock()->now().seconds();
    transitioned_ = false;
    if (reset_requested_) {
      reset_requested_ = false;
      if (has_vehicle_ && now - vehicle_received_s_ <= 0.5 && vehicle_.landed && !vehicle_.armed) {
        abort_latched_ = false; mission_started_ = false; fire_emitted_ = false;
        transition(Mission::STATE_INIT, "explicit reset after landed and disarmed", now);
      }
    }
    const bool vehicle_fresh = has_vehicle_ && now - vehicle_received_s_ <= 0.5;
    if (mission_started_ && state_ != Mission::STATE_LAND && state_ != Mission::STATE_DONE &&
      vehicle_fresh &&
      (vehicle_.takeover_latched || (!vehicle_.offboard && !vehicle_.landed))) {
      abort_latched_ = true;
      transition(Mission::STATE_ABORT, "human takeover or Offboard exit", now);
    }
    if (!abort_latched_) {
      switch (state_) {
        case Mission::STATE_INIT:
          if (auto_start_ && vehicle_fresh && vehicle_.ready_for_mission) {
            mission_started_ = true; transition(Mission::STATE_SEARCH, "Offboard confirmed", now);
          }
          break;
        case Mission::STATE_SEARCH:
          if (has_search_ && search_.timed_out) {transition(Mission::STATE_LAND, "search timeout", now);}
          else if (target_confirmed(now)) {transition(Mission::STATE_TRACK, "target confirmed", now);}
          break;
        case Mission::STATE_TRACK:
          if (!has_track_ || target_age(now) > 0.50) {transition(Mission::STATE_SEARCH, "target lost", now);}
          else if (target_confirmed(now) && alignment_error() <= alignment_gate_rad_) {
            if (aligned_since_s_ < 0.0) {aligned_since_s_ = now;}
            if (now - aligned_since_s_ >= alignment_hold_s_) {
              transition(Mission::STATE_APPROACH, "quality and alignment gates passed", now);
            }
          } else {aligned_since_s_ = -1.0;}
          break;
        case Mission::STATE_APPROACH:
          if (!has_track_ || target_age(now) > 0.50) {transition(Mission::STATE_SEARCH, "target lost", now);}
          else if (has_capture_ && capture_.held) {transition(Mission::STATE_FIRE, "capture window held", now);}
          else if (has_safety_ && safety_.severe_fault && has_vehicle_ && vehicle_.offboard) {
            transition(Mission::STATE_LAND, "internal safety fault", now);
          }
          break;
        case Mission::STATE_FIRE:
          if (!fire_emitted_) {
            std_msgs::msg::String event; event.data = "FIRE_SOFTWARE_EVENT"; fire_pub_->publish(event);
            fire_emitted_ = true;
          }
          transition(Mission::STATE_LAND, "FIRE emitted once", now);
          break;
        case Mission::STATE_LAND:
          if (vehicle_fresh && vehicle_.landed && vehicle_.land_ack_accepted) {
            transition(Mission::STATE_DONE, "PX4 accepted Land and reports landed", now);
          }
          break;
        default: break;
      }
    }

    falcon_intercept_msgs::msg::CommandProposal proposal;
    proposal.header.stamp = get_clock()->now(); proposal.header.frame_id = "pursuer_base_flu";
    proposal.sequence = sequence_; proposal.mission_state = state_; proposal.reason = reason_;
    if (state_ == Mission::STATE_SEARCH && has_search_ && search_.valid) {
      proposal.yaw_rate_flu = search_.yaw_rate_flu; proposal.valid = true;
    } else if ((state_ == Mission::STATE_TRACK || state_ == Mission::STATE_APPROACH) && has_guidance_) {
      proposal = guidance_; proposal.header.stamp = get_clock()->now();
      proposal.sequence = sequence_; proposal.mission_state = state_;
      if (state_ == Mission::STATE_TRACK) {
        proposal.velocity_body_flu.x = proposal.velocity_body_flu.y = proposal.velocity_body_flu.z = 0.0;
      }
    } else if (state_ == Mission::STATE_LAND) {
      proposal.valid = true; proposal.land_request = true; proposal.reason = "PX4 native LAND requested";
    }
    proposal_pub_->publish(proposal);

    Mission status; status.header.stamp = get_clock()->now(); status.header.frame_id = "mission";
    status.sequence = sequence_++; status.state = state_; status.state_elapsed_s = now - state_started_s_;
    status.transitioned = transitioned_; status.abort_latched = abort_latched_;
    status.mission_started = mission_started_; status.reason = reason_;
    status_pub_->publish(status);
  }

  int minimum_track_hits_{};
  double alignment_gate_rad_{}, alignment_hold_s_{};
  double maximum_reprojection_error_px_{3.0}, minimum_geometry_score_{0.10};
  double maximum_baseline_motion_ratio_{0.10};
  bool auto_start_{true}, has_vehicle_{false}, has_track_{false}, has_search_{false};
  bool has_guidance_{false}, has_safety_{false}, has_capture_{false};
  bool reset_requested_{false}, abort_latched_{false}, mission_started_{false};
  bool fire_emitted_{false}, transitioned_{false};
  std::uint8_t state_{Mission::STATE_INIT}; std::uint32_t sequence_{};
  double state_started_s_{}, aligned_since_s_{-1.0}, vehicle_received_s_{}, track_received_s_{};
  std::string reason_{"waiting for Offboard"};
  falcon_intercept_msgs::msg::VehicleInterfaceStatus vehicle_; falcon_intercept_msgs::msg::TargetTrack track_;
  falcon_intercept_msgs::msg::SearchCommand search_; falcon_intercept_msgs::msg::CommandProposal guidance_;
  falcon_intercept_msgs::msg::SafetyStatus safety_; falcon_intercept_msgs::msg::CaptureWindowStatus capture_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::VehicleInterfaceStatus>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::TargetTrack>::SharedPtr track_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::SearchCommand>::SharedPtr search_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::CommandProposal>::SharedPtr guidance_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::SafetyStatus>::SharedPtr safety_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::CaptureWindowStatus>::SharedPtr capture_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Publisher<Mission>::SharedPtr status_pub_;
  rclcpp::Publisher<falcon_intercept_msgs::msg::CommandProposal>::SharedPtr proposal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fire_pub_; rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace falcon_intercept_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<falcon_intercept_control::InterceptFsmNode>());
  rclcpp::shutdown(); return 0;
}
