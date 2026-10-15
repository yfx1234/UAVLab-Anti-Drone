#include "falcon_intercept_control/real_flight_core.hpp"

#include <falcon_intercept_msgs/msg/capture_window_status.hpp>
#include <falcon_intercept_msgs/msg/command_proposal.hpp>
#include <falcon_intercept_msgs/msg/guidance_command.hpp>
#include <falcon_intercept_msgs/msg/mission_status.hpp>
#include <falcon_intercept_msgs/msg/safety_status.hpp>
#include <falcon_intercept_msgs/msg/target_track.hpp>
#include <falcon_intercept_msgs/msg/vehicle_interface_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace falcon_intercept_control
{
class SafetyNode final : public rclcpp::Node
{
public:
  SafetyNode() : Node("safety_node"), capture_(CaptureConfig{})
  {
    calibration_ready_ = declare_parameter<bool>("calibration_ready", false);
    valid_for_s_ = declare_parameter<double>("command_valid_for_s", 0.25);
    CaptureConfig capture_config;
    capture_config.minimum_range_m = declare_parameter<double>("capture_minimum_range_m", 1.5);
    capture_config.maximum_range_m = declare_parameter<double>("capture_maximum_range_m", 2.0);
    capture_config.maximum_alignment_error_rad =
      declare_parameter<double>("shot_preferred_alignment_rad", 0.034906585);
    capture_config.maximum_los_rate_rad_s =
      declare_parameter<double>("capture_maximum_los_rate_rad_s", 0.20);
    capture_config.maximum_relative_speed_mps =
      declare_parameter<double>("capture_maximum_relative_speed_mps", 1.0);
    capture_config.maximum_predicted_miss_distance_m =
      declare_parameter<double>("capture_maximum_predicted_miss_distance_m", 0.5);
    capture_config.maximum_time_to_closest_approach_s =
      declare_parameter<double>("capture_maximum_time_to_closest_approach_s", 2.0);
    capture_config.required_hold_time_s =
      declare_parameter<double>("shot_minimum_ready_hold_s", 0.30);
    capture_config.minimum_distinct_frames =
      static_cast<std::uint32_t>(declare_parameter<int>("capture_minimum_distinct_frames", 6));
    capture_ = CaptureWindow(capture_config);
    shot_hard_alignment_rad_ =
      declare_parameter<double>("shot_hard_alignment_rad", 0.034906585);
    shot_prediction_horizon_s_ =
      declare_parameter<double>("shot_timing_prediction_horizon_s", 0.04);
    shot_error_filter_time_constant_s_ =
      declare_parameter<double>("shot_error_filter_time_constant_s", 0.08);
    shot_maximum_error_rate_rad_s_ =
      declare_parameter<double>("shot_maximum_error_rate_rad_s", 0.5);
    projectile_speed_mps_ = declare_parameter<double>("projectile_speed_mps", 400.0);
    shot_maximum_time_of_flight_s_ =
      declare_parameter<double>("shot_maximum_time_of_flight_s", 0.25);
    shot_maximum_lead_angle_rad_ =
      declare_parameter<double>("shot_maximum_lead_angle_rad", 0.069813170);
    shot_maximum_lateral_sigma_m_ =
      declare_parameter<double>("shot_maximum_lateral_sigma_m", 0.75);
    maximum_forward_speed_mps_ =
      declare_parameter<double>("maximum_forward_speed_mps", 0.30);
    maximum_lateral_speed_mps_ =
      declare_parameter<double>("maximum_lateral_speed_mps", 0.15);
    maximum_vertical_speed_mps_ =
      declare_parameter<double>("maximum_vertical_speed_mps", 0.10);
    maximum_yaw_rate_rps_ =
      declare_parameter<double>("maximum_yaw_rate_rps", 0.20);
    track_sub_ = create_subscription<falcon_intercept_msgs::msg::TargetTrack>(
      "/target/track", rclcpp::SensorDataQoS().keep_last(1), [this](falcon_intercept_msgs::msg::TargetTrack::ConstSharedPtr m) {track_ = *m; has_track_ = true; track_received_s_ = get_clock()->now().seconds();});
    mission_sub_ = create_subscription<falcon_intercept_msgs::msg::MissionStatus>(
      "/decision/mission_status", 10, [this](falcon_intercept_msgs::msg::MissionStatus::ConstSharedPtr m) {mission_ = *m; has_mission_ = true; mission_received_s_ = get_clock()->now().seconds();});
    proposal_sub_ = create_subscription<falcon_intercept_msgs::msg::CommandProposal>(
      "/decision/command_proposal", 10, [this](falcon_intercept_msgs::msg::CommandProposal::ConstSharedPtr m) {proposal_ = *m; has_proposal_ = true; proposal_received_s_ = get_clock()->now().seconds();});
    vehicle_sub_ = create_subscription<falcon_intercept_msgs::msg::VehicleInterfaceStatus>(
      "/control/px4_status", 10, [this](falcon_intercept_msgs::msg::VehicleInterfaceStatus::ConstSharedPtr m) {vehicle_ = *m; has_vehicle_ = true; vehicle_received_s_ = get_clock()->now().seconds();});
    safety_pub_ = create_publisher<falcon_intercept_msgs::msg::SafetyStatus>("/decision/safety_status", 10);
    capture_pub_ = create_publisher<falcon_intercept_msgs::msg::CaptureWindowStatus>("/decision/capture_window", 10);
    command_pub_ = create_publisher<falcon_intercept_msgs::msg::GuidanceCommand>("/intercept/guidance_command", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  void tick()
  {
    const auto stamp = get_clock()->now(); const double now = stamp.seconds();
    const bool proposal_fresh = has_proposal_ && now - proposal_received_s_ <= 0.25;
    const bool mission_fresh = has_mission_ && now - mission_received_s_ <= 0.50;
    const bool vehicle_fresh = has_vehicle_ && now - vehicle_received_s_ <= 0.50;
    const double target_age = has_track_ ? std::max({static_cast<double>(track_.age_s),
      now - track_received_s_, now - rclcpp::Time(track_.header.stamp).seconds()}) : INFINITY;
    const bool finite = proposal_fresh && std::isfinite(proposal_.velocity_body_flu.x) &&
      std::isfinite(proposal_.velocity_body_flu.y) && std::isfinite(proposal_.velocity_body_flu.z) &&
      std::isfinite(proposal_.yaw_rate_flu);
    double max_var = INFINITY;
    if (has_track_) {max_var = std::max({track_.covariance[0], track_.covariance[7], track_.covariance[14]});}
    const bool px4_ok = vehicle_fresh && vehicle_.ready_for_mission && !vehicle_.failsafe && !vehicle_.takeover_latched;
    const bool async_quality_ok = has_track_ && track_.valid && track_.timing_valid &&
      track_.range_observable && track_.consecutive_hits >= 5 &&
      std::isfinite(track_.reprojection_error_px) && track_.reprojection_error_px <= 3.0 &&
      std::isfinite(track_.measurement_geometry_score) && track_.measurement_geometry_score >= 0.10 &&
      std::isfinite(track_.range_m) && std::isfinite(track_.range_sigma_m) &&
      std::isfinite(track_.range_m + 2.0 * std::max(0.0, track_.range_sigma_m)) &&
      track_.range_m + 2.0 * std::max(0.0, track_.range_sigma_m) > 0.0 &&
      std::abs(track_.velocity_camera.x) * std::max(0.0, track_.timing_delta_s) / 0.1 <= 0.10;
    const auto quality = evaluate_quality(QualityConfig{}, QualityInput{
      calibration_ready_, has_track_ && track_.valid, target_age,
      max_var, has_track_ ? track_.innovation_nis : INFINITY, px4_ok});

    RelativeKinematicsResult kin;
    double alignment = INFINITY;
    double time_of_flight = INFINITY;
    double lead_angle = INFINITY;
    double lateral_sigma = INFINITY;
    if (has_track_) {
      Vector3 p{track_.position_camera.x, track_.position_camera.y, track_.position_camera.z};
      Vector3 v{track_.velocity_camera.x, track_.velocity_camera.y, track_.velocity_camera.z};
      kin = relative_kinematics(p, v);
      Vector3 intercept = p;
      if (kin.valid && projectile_speed_mps_ > 0.0) {
        time_of_flight = kin.range_m / projectile_speed_mps_;
        for (int iteration = 0; iteration < 12; ++iteration) {
          for (int axis = 0; axis < 3; ++axis) {
            intercept[axis] = p[axis] + v[axis] * time_of_flight;
          }
          const double next = std::sqrt(
            intercept[0] * intercept[0] + intercept[1] * intercept[1] +
            intercept[2] * intercept[2]) / projectile_speed_mps_;
          if (!std::isfinite(next)) {break;}
          if (std::abs(next - time_of_flight) < 1e-7) {
            time_of_flight = next;
            break;
          }
          time_of_flight = next;
        }
      }
      alignment = std::atan2(std::hypot(intercept[0], intercept[1]), intercept[2]);
      lead_angle = std::acos(std::clamp(
        (p[0] * intercept[0] + p[1] * intercept[1] + p[2] * intercept[2]) /
        std::max(1e-9, kin.range_m * std::sqrt(
          intercept[0] * intercept[0] + intercept[1] * intercept[1] +
          intercept[2] * intercept[2])), -1.0, 1.0));
      if (std::isfinite(time_of_flight)) {
        double trace = 0.0;
        double axial = 0.0;
        const double intercept_norm = std::sqrt(
          intercept[0] * intercept[0] + intercept[1] * intercept[1] +
          intercept[2] * intercept[2]);
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            const double covariance =
              track_.covariance[row * 6 + column] +
              time_of_flight * (
              track_.covariance[row * 6 + column + 3] +
              track_.covariance[(row + 3) * 6 + column]) +
              time_of_flight * time_of_flight *
              track_.covariance[(row + 3) * 6 + column + 3];
            if (row == column) {trace += covariance;}
            if (intercept_norm > 1e-9) {
              axial += intercept[row] * intercept[column] * covariance /
                (intercept_norm * intercept_norm);
            }
          }
        }
        lateral_sigma = std::sqrt(std::max(0.0, trace - axial));
      }
    }
    double predicted_alignment = alignment;
    if (std::isfinite(alignment)) {
      if (alignment_time_initialized_) {
        const double dt = now - previous_alignment_time_s_;
        if (dt > 1e-4 && dt < 0.5) {
          const double raw_rate = (alignment - previous_alignment_rad_) / dt;
          const double alpha = dt / (shot_error_filter_time_constant_s_ + dt);
          filtered_alignment_rate_rad_s_ +=
            alpha * (raw_rate - filtered_alignment_rate_rad_s_);
        } else if (dt < 0.0 || dt >= 0.5) {
          filtered_alignment_rate_rad_s_ = 0.0;
        }
      }
      alignment_time_initialized_ = true;
      previous_alignment_time_s_ = now;
      previous_alignment_rad_ = alignment;
      predicted_alignment = std::max(
        0.0, alignment + filtered_alignment_rate_rad_s_ * shot_prediction_horizon_s_);
    }
    const bool predictive_shot_ok =
      std::isfinite(alignment) && alignment <= shot_hard_alignment_rad_ &&
      std::isfinite(predicted_alignment) &&
      std::abs(filtered_alignment_rate_rad_s_) <= shot_maximum_error_rate_rad_s_ &&
      std::isfinite(time_of_flight) && time_of_flight > 0.0 &&
      time_of_flight <= shot_maximum_time_of_flight_s_ &&
      std::isfinite(lead_angle) && lead_angle <= shot_maximum_lead_angle_rad_ &&
      std::isfinite(lateral_sigma) && lateral_sigma <= shot_maximum_lateral_sigma_m_;
    const bool approach = mission_fresh && mission_.state == mission_.STATE_APPROACH;
    const auto cap = capture_.update(CaptureInput{now,
      has_track_ ? rclcpp::Time(track_.header.stamp).seconds() : 0.0,
      has_track_ ? track_.sequence : 0U, approach && has_track_ && track_.valid,
      quality.passed && async_quality_ok && predictive_shot_ok, predicted_alignment, kin,
      (has_track_ && std::isfinite(track_.range_m) && std::isfinite(track_.range_sigma_m)) ?
      track_.range_m + 2.0 * std::max(0.0, track_.range_sigma_m) : 0.0});

    falcon_intercept_msgs::msg::CaptureWindowStatus capture_msg;
    capture_msg.header.stamp = stamp; capture_msg.sequence = sequence_; capture_msg.quality_ok = cap.quality_ok;
    capture_msg.alignment_ok = cap.alignment_ok; capture_msg.range_ok = cap.range_ok;
    capture_msg.los_rate_ok = cap.los_rate_ok; capture_msg.relative_speed_ok = cap.relative_speed_ok;
    capture_msg.prediction_ok = cap.prediction_ok; capture_msg.frame_count_ok = cap.frame_count_ok;
    capture_msg.instantaneous_pass = cap.instantaneous_pass; capture_msg.held = cap.held;
    capture_msg.distinct_frames = cap.distinct_frames; capture_msg.hold_elapsed_s = cap.hold_elapsed_s;
    capture_msg.alignment_error_rad = predicted_alignment; capture_msg.range_m = kin.range_m;
    capture_msg.los_rate_rad_s = kin.los_rate_norm_rad_s; capture_msg.relative_speed_mps = kin.relative_speed_mps;
    capture_msg.predicted_miss_distance_m = kin.predicted_miss_distance_m;
    capture_msg.time_to_closest_approach_s = kin.time_to_closest_approach_s; capture_pub_->publish(capture_msg);

    const bool target_state = mission_fresh && (mission_.state == mission_.STATE_TRACK || mission_.state == mission_.STATE_APPROACH);
    const bool land = mission_fresh && mission_.state == mission_.STATE_LAND && proposal_fresh && proposal_.land_request;
    bool allowed = proposal_fresh && mission_fresh && proposal_.valid && finite && calibration_ready_ && px4_ok;
    allowed = allowed && (!target_state || async_quality_ok);
    if (target_state) {allowed = allowed && has_track_ && target_age <= 0.50 && quality.covariance_ok && quality.innovation_ok;}
    if (land) {allowed = has_vehicle_ && vehicle_.offboard && vehicle_.vehicle_status_fresh && !vehicle_.takeover_latched;}

    falcon_intercept_msgs::msg::GuidanceCommand command;
    command.header.stamp = stamp; command.header.frame_id = "pursuer_base_flu"; command.sequence = sequence_;
    command.mission_state = has_mission_ ? mission_.state : command.STATE_INIT; command.valid_for_s = valid_for_s_;
    command.valid = allowed; command.land_request = allowed && land; command.reason = allowed ? proposal_.reason : "safety gate blocked";
    if (allowed && !land) {
      double scale = (has_track_ && target_state) ? prediction_decay(target_age) : 1.0;
      command.velocity_body_flu.x = std::clamp(
        proposal_.velocity_body_flu.x * scale,
        -maximum_forward_speed_mps_, maximum_forward_speed_mps_);
      command.velocity_body_flu.y = std::clamp(
        proposal_.velocity_body_flu.y * scale,
        -maximum_lateral_speed_mps_, maximum_lateral_speed_mps_);
      command.velocity_body_flu.z = std::clamp(
        proposal_.velocity_body_flu.z * scale,
        -maximum_vertical_speed_mps_, maximum_vertical_speed_mps_);
      command.yaw_rate_flu = static_cast<float>(std::clamp(
        static_cast<double>(proposal_.yaw_rate_flu),
        -maximum_yaw_rate_rps_, maximum_yaw_rate_rps_));
    }
    command_pub_->publish(command);

    falcon_intercept_msgs::msg::SafetyStatus safety;
    safety.header.stamp = stamp; safety.sequence = sequence_++; safety.calibration_ok = quality.calibration_ok;
    safety.target_fresh = quality.target_fresh; safety.covariance_ok = quality.covariance_ok;
    safety.innovation_ok = quality.innovation_ok; safety.px4_state_ok = quality.px4_state_ok;
    safety.command_finite = finite; safety.tracking_quality_passed = quality.passed;
    safety.severe_fault = !finite || (has_vehicle_ && vehicle_.failsafe);
    safety.output_allowed = allowed; safety.reason = allowed ? "allowed" : quality.reason; safety_pub_->publish(safety);
  }

  bool calibration_ready_{false}, has_track_{false}, has_mission_{false}, has_proposal_{false}, has_vehicle_{false};
  double valid_for_s_{0.25}; std::uint32_t sequence_{}; CaptureWindow capture_;
  double shot_hard_alignment_rad_{0.034906585};
  double shot_prediction_horizon_s_{0.04}, shot_error_filter_time_constant_s_{0.08};
  double shot_maximum_error_rate_rad_s_{0.5}, projectile_speed_mps_{400.0};
  double shot_maximum_time_of_flight_s_{0.25}, shot_maximum_lead_angle_rad_{0.069813170};
  double shot_maximum_lateral_sigma_m_{0.75};
  double maximum_forward_speed_mps_{0.30}, maximum_lateral_speed_mps_{0.15};
  double maximum_vertical_speed_mps_{0.10}, maximum_yaw_rate_rps_{0.20};
  bool alignment_time_initialized_{false};
  double previous_alignment_time_s_{}, previous_alignment_rad_{};
  double filtered_alignment_rate_rad_s_{};
  double track_received_s_{}, mission_received_s_{}, proposal_received_s_{}, vehicle_received_s_{};
  falcon_intercept_msgs::msg::TargetTrack track_; falcon_intercept_msgs::msg::MissionStatus mission_;
  falcon_intercept_msgs::msg::CommandProposal proposal_; falcon_intercept_msgs::msg::VehicleInterfaceStatus vehicle_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::TargetTrack>::SharedPtr track_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::CommandProposal>::SharedPtr proposal_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::VehicleInterfaceStatus>::SharedPtr vehicle_sub_;
  rclcpp::Publisher<falcon_intercept_msgs::msg::SafetyStatus>::SharedPtr safety_pub_;
  rclcpp::Publisher<falcon_intercept_msgs::msg::CaptureWindowStatus>::SharedPtr capture_pub_;
  rclcpp::Publisher<falcon_intercept_msgs::msg::GuidanceCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace falcon_intercept_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<falcon_intercept_control::SafetyNode>());
  rclcpp::shutdown(); return 0;
}
