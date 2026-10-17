#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "px4_offboard_tracking/acceleration_guidance.hpp"
#include "px4_offboard_tracking/alignment_feasibility.hpp"
#include "px4_offboard_tracking/approach_corridor.hpp"
#include "px4_offboard_tracking/attitude_setpoint_generator.hpp"
#include "px4_offboard_tracking/body_rate_controller.hpp"
#include "px4_offboard_tracking/camera_model.hpp"
#include "px4_offboard_tracking/capture_window.hpp"
#include "px4_offboard_tracking/common_frame_transform.hpp"
#include "px4_offboard_tracking/geometric_control.hpp"
#include "px4_offboard_tracking/msg/alignment_feasibility.hpp"
#include "px4_offboard_tracking/msg/capture_window_status.hpp"
#include "px4_offboard_tracking/msg/relative_estimate.hpp"
#include "px4_offboard_tracking/msg/relative_kinematics.hpp"
#include "px4_offboard_tracking/msg/safety_status.hpp"
#include "px4_offboard_tracking/mission_state_machine.hpp"
#include "px4_offboard_tracking/horizontal_speed_limiter.hpp"
#include "px4_offboard_tracking/mpc_guidance.hpp"
#include "px4_offboard_tracking/msg/mission_status.hpp"
#include "px4_offboard_tracking/msg/pure_interceptor_status.hpp"
#include "px4_offboard_tracking/msg/target_observation.hpp"
#include "px4_offboard_tracking/msg/tracking_quality.hpp"
#include "px4_offboard_tracking/msg/virtual_capture_event.hpp"
#include "px4_offboard_tracking/target_trajectory.hpp"
#include "px4_offboard_tracking/safety_manager.hpp"
#include "px4_offboard_tracking/startup_search.hpp"
#include "px4_offboard_tracking/terminal_guidance.hpp"
#include "px4_offboard_tracking/tracking_quality_gate.hpp"
#include "px4_offboard_tracking/tracking_controller.hpp"
#include "px4_offboard_tracking/z_axis_alignment.hpp"

using namespace std::chrono_literals;

namespace px4_offboard_tracking
{

class OffboardVehicle : public rclcpp::Node
{
public:
  OffboardVehicle()
  : Node("offboard_vehicle"),
    role_(declare_parameter<std::string>("role", "hold")),
    topic_prefix_(declare_parameter<std::string>("topic_prefix", "/fmu")),
    target_state_topic_(declare_parameter<std::string>("target_state_topic", "/tracking/target/state")),
    common_origin_topic_(
      declare_parameter<std::string>("common_origin_topic", "/tracking/common_origin")),
    task_origin_topic_(
      declare_parameter<std::string>("task_origin_topic", "/tracking/task_origin_ned")),
    relative_state_topic_(
      declare_parameter<std::string>("relative_state_topic", "/tracking/relative/state")),
    relative_estimate_topic_(
      declare_parameter<std::string>("relative_estimate_topic", "/tracking/relative/estimate")),
    alignment_feasibility_topic_(declare_parameter<std::string>(
      "alignment_feasibility_topic", "/tracking/alignment/feasibility")),
    mission_status_topic_(declare_parameter<std::string>(
      "mission_status_topic", "/tracking/mission/status")),
    observation_topic_(declare_parameter<std::string>(
      "observation_topic", "/tracking/camera/observation")),
    tracking_quality_topic_(declare_parameter<std::string>(
      "tracking_quality_topic", "/tracking/quality_gate")),
    relative_kinematics_topic_(declare_parameter<std::string>(
      "relative_kinematics_topic", "/tracking/relative/kinematics")),
    capture_window_topic_(declare_parameter<std::string>(
      "capture_window_topic", "/tracking/capture_window")),
    virtual_capture_event_topic_(declare_parameter<std::string>(
      "virtual_capture_event_topic", "/tracking/virtual_capture_event")),
    safety_status_topic_(declare_parameter<std::string>(
      "safety_status_topic", "/tracking/safety/status")),
    mission_complete_topic_(declare_parameter<std::string>(
      "mission_complete_topic", "/tracking/mission/complete")),
    common_frame_id_(declare_parameter<std::string>("common_frame_id", "map_ned")),
    control_level_(declare_parameter<std::string>("control_level", "velocity")),
    target_system_(declare_parameter<int>("target_system", 1)),
    target_scenario_name_(declare_parameter<std::string>("target_scenario", "circle")),
    takeoff_height_(declare_parameter<double>("takeoff_height_m", 5.0)),
    target_radius_(declare_parameter<double>("target_radius_m", 4.0)),
    target_speed_(declare_parameter<double>("target_speed_mps", 1.0)),
    target_heading_(declare_parameter<double>("target_heading_rad", 0.0)),
    target_path_length_(declare_parameter<double>("target_path_length_m", 8.0)),
    target_path_width_(declare_parameter<double>("target_path_width_m", 3.0)),
    target_trajectory_feedforward_enabled_(
      declare_parameter<bool>("target_trajectory_feedforward_enabled", false)),
    target_motion_delay_(declare_parameter<double>("target_motion_delay_s", 10.0)),
    target_motion_ramp_s_(declare_parameter<double>("target_motion_ramp_s", 3.0)),
    target_search_enable_after_motion_s_(declare_parameter<double>(
      "target_search_enable_after_motion_s", 0.2)),
    target_wait_for_pursuer_intercept_(
      declare_parameter<bool>("target_wait_for_pursuer_intercept", false)),
    target_motion_ready_topic_(declare_parameter<std::string>(
      "target_motion_ready_topic", "/simulation/target_motion_ready")),
    target_wait_for_pursuer_takeoff_ready_(declare_parameter<bool>(
      "target_wait_for_pursuer_takeoff_ready", false)),
    pursuer_takeoff_ready_topic_(declare_parameter<std::string>(
      "pursuer_takeoff_ready_topic", "/simulation/pursuer_takeoff_ready")),
    pure_interceptor_status_topic_(declare_parameter<std::string>(
      "pure_interceptor_status_topic", "/tracking/pure_interceptor/status")),
    attack_enable_delay_(declare_parameter<double>("attack_enable_delay_s", 30.0)),
    target_stable_motion_time_s_(declare_parameter<double>("target_stable_motion_time_s", 10.0)),
    target_trajectory_(
      target_scenario_from_string(target_scenario_name_), takeoff_height_, target_radius_,
      target_speed_, target_heading_, target_path_length_, target_path_width_),
    state_timeout_(declare_parameter<double>("state_timeout_s", 0.5)),
    status_timeout_(declare_parameter<double>("status_timeout_s", 1.0)),
    estimate_prediction_coast_s_(declare_parameter<double>("estimate_prediction_coast_s", 1.0)),
    estimate_measurement_hold_s_(
      declare_parameter<double>("estimate_measurement_hold_s", 0.4)),
    offboard_rate_hz_(declare_parameter<double>("offboard_rate_hz", 50.0)),
    offboard_warmup_s_(declare_parameter<double>("offboard_warmup_s", 1.0)),
    preflight_stability_hold_s_(
      declare_parameter<double>("preflight_stability_hold_s", 5.0)),
    altitude_hold_position_gain_(declare_parameter<double>("altitude_hold_position_gain", 0.8)),
    altitude_hold_velocity_gain_(declare_parameter<double>("altitude_hold_velocity_gain", 1.2)),
    altitude_hold_max_speed_(declare_parameter<double>("altitude_hold_max_speed_mps", 2.0)),
    altitude_hold_max_acceleration_(
      declare_parameter<double>("altitude_hold_max_acceleration_mps2", 2.0)),
    controller_(
      declare_parameter<double>("position_gain", 0.6),
      declare_parameter<double>("velocity_feedforward", 1.0),
      declare_parameter<double>("max_horizontal_speed_mps", 5.0),
      declare_parameter<double>("max_vertical_speed_mps", 2.0),
      declare_parameter<double>("standoff_distance_m", 3.0)),
    guidance_(
      declare_parameter<double>("acceleration_position_gain", 0.8),
      declare_parameter<double>("acceleration_velocity_gain", 1.2),
      declare_parameter<double>("max_horizontal_acceleration_mps2", 3.0),
      declare_parameter<double>("max_vertical_acceleration_mps2", 2.0),
      get_parameter("standoff_distance_m").as_double()),
    guidance_controller_(declare_parameter<std::string>("guidance_controller", "mpc_3d")),
    approach_corridor_(ApproachCorridorConfig{
      declare_parameter<double>("corridor_vertical_separation_m", 30.0),
      declare_parameter<double>("corridor_prediction_time_s", 0.8),
      declare_parameter<double>("corridor_maximum_lead_distance_m", 2.0),
      declare_parameter<double>("corridor_horizontal_position_gain", 0.35),
      declare_parameter<double>("corridor_horizontal_velocity_gain", 1.4),
      declare_parameter<double>("corridor_maximum_horizontal_acceleration_mps2", 1.2),
      declare_parameter<double>("corridor_minimum_target_above_m", 0.5)}),
    mpc_guidance_(MpcGuidanceConfig{
      declare_parameter<double>("mpc_time_step_s", 0.1),
      static_cast<int>(declare_parameter<int>("mpc_horizon_steps", 25)),
      declare_parameter<double>("mpc_position_weight", 1.0),
      declare_parameter<double>("mpc_velocity_weight", 4.0),
      declare_parameter<double>("mpc_acceleration_weight", 1.5),
      declare_parameter<double>("mpc_terminal_weight_multiplier", 4.0),
      declare_parameter<double>("mpc_prediction_lookahead_s", 0.5),
      declare_parameter<double>("mpc_maximum_horizontal_acceleration_mps2", 1.3),
      get_parameter("corridor_minimum_target_above_m").as_double(),
      declare_parameter<double>("mpc_desired_vertical_separation_m", 30.0),
      declare_parameter<double>("mpc_maximum_vertical_acceleration_mps2", 0.5),
      declare_parameter<double>("mpc_vertical_position_weight", 0.25),
      declare_parameter<double>("mpc_vertical_velocity_weight", 6.0),
      declare_parameter<double>("mpc_vertical_acceleration_weight", 4.0)}),
    terminal_guidance_(TerminalGuidanceConfig{
      declare_parameter<double>("terminal_navigation_constant", 3.0),
      declare_parameter<double>("terminal_minimum_closing_speed_mps", 0.5),
      declare_parameter<double>("terminal_los_acceleration_mps2", 0.6),
      declare_parameter<double>("terminal_acceleration_feedforward_gain", 0.5),
      declare_parameter<double>("terminal_maximum_horizontal_acceleration_mps2", 2.0),
      declare_parameter<double>("terminal_maximum_vertical_acceleration_mps2", 0.8)}),
    mpc_vertical_absolute_hold_blend_(
      declare_parameter<double>("mpc_vertical_absolute_hold_blend", 0.7)),
    mpc_desired_vertical_separation_m_(
      get_parameter("mpc_desired_vertical_separation_m").as_double()),
    mpc_3d_entry_vertical_tolerance_m_(
      declare_parameter<double>("mpc_3d_entry_vertical_tolerance_m", 1.0)),
    mpc_3d_entry_vertical_speed_tolerance_mps_(
      declare_parameter<double>("mpc_3d_entry_vertical_speed_tolerance_mps", 0.5)),
    mpc_3d_entry_hold_time_s_(
      declare_parameter<double>("mpc_3d_entry_hold_time_s", 2.0)),
    pursuer_yaw_mode_(declare_parameter<std::string>("pursuer_yaw_mode", "velocity")),
    pursuer_fixed_yaw_rad_(declare_parameter<double>("pursuer_fixed_yaw_rad", 0.0)),
    pursuer_yaw_minimum_speed_(declare_parameter<double>(
      "pursuer_yaw_minimum_speed_mps", 0.3)),
    invalid_estimate_velocity_damping_(
      declare_parameter<double>("invalid_estimate_velocity_damping", 1.4)),
    invalid_estimate_maximum_acceleration_(
      declare_parameter<double>("invalid_estimate_maximum_acceleration_mps2", 1.2)),
    horizontal_speed_limiter_(HorizontalSpeedLimiterConfig{
      declare_parameter<double>("command_maximum_horizontal_speed_mps", 4.0),
      declare_parameter<double>("command_speed_soft_zone_mps", 0.5),
      declare_parameter<double>("command_overspeed_braking_gain", 2.0),
      declare_parameter<double>("command_maximum_braking_acceleration_mps2", 2.0)}),
    startup_search_(StartupSearchConfig{
      declare_parameter<bool>("startup_search_enabled", false),
      declare_parameter<double>("startup_search_yaw_rate_rad_s", 0.5),
      declare_parameter<double>("startup_search_turns", 1.0),
      declare_parameter<double>("startup_search_settle_time_s", 0.5),
      declare_parameter<bool>("startup_search_stop_on_acquisition", true),
      declare_parameter<bool>("startup_search_repeat_until_acquired", true),
      declare_parameter<double>("startup_search_acquisition_hold_time_s", 0.2)}),
    search_camera_model_(
      CameraIntrinsics{
        declare_parameter<double>("search_camera_fx", 6089.462020537369),
        declare_parameter<double>("search_camera_fy", 1870.614872174388),
        declare_parameter<double>("search_camera_cx", 1919.5),
        declare_parameter<double>("search_camera_cy", 1079.5),
        static_cast<int>(declare_parameter<int>("search_camera_width", 3840)),
        static_cast<int>(declare_parameter<int>("search_camera_height", 2160)),
        0.05},
      CameraExtrinsics{
        Vector3d{},
        CameraModel::default_search_optical_quaternion()},
      CameraFieldOfView{
        0.5 * declare_parameter<double>("search_camera_horizontal_fov_rad", 0.610865238),
        0.5 * declare_parameter<double>("search_camera_vertical_fov_rad", 1.047197551)}),
    search_handover_alignment_blend_(
      declare_parameter<double>("search_handover_alignment_blend", 0.7)),
    search_handover_maximum_tilt_rad_(
      declare_parameter<double>("search_handover_maximum_tilt_rad", 0.35)),
    depth_confirmation_hold_time_s_(
      declare_parameter<double>("depth_confirmation_hold_time_s", 0.2)),
    depth_confirmation_minimum_frames_(
      declare_parameter<int>("depth_confirmation_minimum_frames", 5)),
    geometric_control_(
      declare_parameter<double>("hover_thrust", 0.42),
      declare_parameter<double>("max_thrust", 0.70),
      declare_parameter<double>("max_tilt_rad", 0.5),
      declare_parameter<double>("attitude_gain", 3.0),
      declare_parameter<double>("max_body_rate_rad_s", 3.0)),
    z_axis_alignment_(ZAxisAlignmentConfig{
      declare_parameter<double>("alignment_prediction_time_s", 0.0),
      declare_parameter<double>("alignment_maximum_tilt_rad", 0.28),
      get_parameter("corridor_minimum_target_above_m").as_double(),
      declare_parameter<double>("alignment_entry_horizontal_distance_m", 1.0),
      declare_parameter<double>("alignment_entry_relative_speed_mps", 0.8)}),
    alignment_feasibility_(AlignmentFeasibilityConfig{
      declare_parameter<double>("feasibility_maximum_tilt_rad", 0.30),
      declare_parameter<double>("feasibility_minimum_thrust_margin", 0.08),
      declare_parameter<double>("feasibility_maximum_body_rate_rad_s", 2.0),
      declare_parameter<double>("feasibility_maximum_altitude_loss_m", 1.2),
      declare_parameter<double>("feasibility_minimum_range_m", 10.0),
      declare_parameter<double>("feasibility_maximum_alignment_time_s", 60.0)}),
    feasibility_maximum_available_thrust_(declare_parameter<double>(
      "feasibility_maximum_available_thrust", 0.9)),
    aim_entry_horizontal_distance_m_(declare_parameter<double>(
      "aim_entry_horizontal_distance_m", 10.0)),
    aim_exit_horizontal_distance_m_(declare_parameter<double>(
      "aim_exit_horizontal_distance_m", 12.0)),
    capture_minimum_range_m_(declare_parameter<double>(
      "capture_minimum_range_m", 10.0)),
    capture_maximum_range_m_(declare_parameter<double>(
      "capture_maximum_range_m", 35.0)),
    midcourse_alignment_blend_(declare_parameter<double>(
      "midcourse_alignment_blend", 0.3)),
    terminal_alignment_blend_(declare_parameter<double>(
      "terminal_alignment_blend", 0.7)),
    terminal_capture_alignment_blend_(declare_parameter<double>(
      "terminal_capture_alignment_blend", 0.95)),
    terminal_alignment_weight_slew_rate_(declare_parameter<double>(
      "terminal_alignment_weight_slew_rate_per_s", 0.8)),
    collision_exit_range_m_(declare_parameter<double>(
      "collision_exit_range_m", 5.0)),
    alignment_ready_error_rad_(declare_parameter<double>(
      "alignment_ready_error_rad", 0.12)),
    takeoff_ready_tolerance_m_(declare_parameter<double>(
      "takeoff_ready_tolerance_m", 0.6)),
    altitude_correction_threshold_m_(declare_parameter<double>(
      "altitude_correction_threshold_m", 0.8)),
    mission_minimum_target_above_m_(declare_parameter<double>(
      "mission_minimum_target_above_m", 0.5)),
    required_virtual_captures_(declare_parameter<int>("required_virtual_captures", 2)),
    reattack_cooldown_s_(declare_parameter<double>("reattack_cooldown_s", 3.0)),
    post_capture_recoil_duration_s_(
      declare_parameter<double>("post_capture_recoil_duration_s", 0.6)),
    post_capture_recoil_acceleration_mps2_(
      declare_parameter<double>("post_capture_recoil_acceleration_mps2", 1.0)),
    tracking_quality_gate_(TrackingQualityGateConfig{
      declare_parameter<double>("quality_maximum_position_variance", 4.0),
      declare_parameter<double>("quality_maximum_innovation_mahalanobis", 11.345),
      declare_parameter<double>("quality_maximum_observation_age_s", 0.2),
      declare_parameter<double>("quality_maximum_estimate_age_s", 0.2)}),
    capture_window_(CaptureWindowConfig{
      declare_parameter<double>("capture_maximum_pixel_error_px", 448.628),
      declare_parameter<double>("capture_maximum_alignment_error_rad", 0.034906585),
      declare_parameter<double>("capture_maximum_los_rate_rad_s", 0.25),
      declare_parameter<double>("capture_maximum_predicted_miss_distance_m", 35.0),
      declare_parameter<double>("capture_maximum_time_to_closest_approach_s", 15.0),
      declare_parameter<double>("capture_required_hold_time_s", 0.2),
      static_cast<int>(declare_parameter<int>("capture_minimum_distinct_frames", 5))}),
    capture_camera_cx_(declare_parameter<double>("capture_camera_cx", 1919.5)),
    capture_camera_cy_(declare_parameter<double>("capture_camera_cy", 1079.5)),
    safety_manager_(SafetyManagerConfig{
      declare_parameter<double>("safety_minimum_range_m", 5.0),
      declare_parameter<double>("safety_maximum_tilt_rad", 0.45),
      declare_parameter<double>("safety_maximum_speed_mps", 4.0),
      declare_parameter<double>("safety_maximum_altitude_error_m", 20.0),
      declare_parameter<double>("recovery_maximum_tilt_rad", 0.12),
      declare_parameter<double>("recovery_maximum_speed_mps", 0.5),
      declare_parameter<double>("recovery_maximum_altitude_error_m", 0.6),
      declare_parameter<double>("recovery_required_hold_time_s", 1.0)}),
    attitude_setpoint_generator_(AttitudeSetpointGeneratorConfig{
      get_parameter("hover_thrust").as_double(),
      declare_parameter<double>("minimum_thrust", 0.1),
      get_parameter("max_thrust").as_double(),
      declare_parameter<double>("attitude_smoothing_time_constant_s", 0.35),
      declare_parameter<double>("maximum_attitude_slew_rate_rad_s", 1.0)}),
    body_rate_controller_(BodyRateControllerConfig{
      declare_parameter<double>("body_rate_roll_pitch_gain", 4.0),
      declare_parameter<double>("body_rate_yaw_gain", 2.0),
      declare_parameter<double>("body_rate_maximum_roll_pitch_rate_rad_s", 2.5),
      declare_parameter<double>("body_rate_maximum_yaw_rate_rad_s", 1.5)})
  {
    if (role_ != "hold" && role_ != "target" && role_ != "pursuer") {
      throw std::invalid_argument("role must be hold, target, or pursuer");
    }
    if (control_level_ != "velocity" && control_level_ != "attitude" &&
      control_level_ != "body_rate")
    {
      throw std::invalid_argument("control_level must be velocity, attitude, or body_rate");
    }
    if (guidance_controller_ != "pd" && guidance_controller_ != "mpc" &&
      guidance_controller_ != "mpc_3d")
    {
      throw std::invalid_argument("guidance_controller must be pd, mpc, or mpc_3d");
    }
    if (mpc_vertical_absolute_hold_blend_ < 0.0 || mpc_vertical_absolute_hold_blend_ > 1.0) {
      throw std::invalid_argument("mpc_vertical_absolute_hold_blend must be within [0, 1]");
    }
    if (state_timeout_ <= 0.0 || status_timeout_ <= 0.0 ||
      estimate_prediction_coast_s_ < state_timeout_ ||
      estimate_measurement_hold_s_ < 0.0 ||
      estimate_measurement_hold_s_ > estimate_prediction_coast_s_)
    {
      throw std::invalid_argument(
              "estimate hold must be within prediction coast and coast >= positive timeout");
    }
    if (depth_confirmation_hold_time_s_ < 0.0 || depth_confirmation_minimum_frames_ < 1) {
      throw std::invalid_argument(
              "depth confirmation hold must be non-negative and frame count positive");
    }
    if (search_handover_alignment_blend_ < 0.0 ||
      search_handover_alignment_blend_ > 1.0 ||
      search_handover_maximum_tilt_rad_ <= 0.0 ||
      search_handover_maximum_tilt_rad_ >= 1.5707963267948966)
    {
      throw std::invalid_argument(
              "search handover blend must be within [0, 1] and tilt within (0, pi/2)");
    }
    if (mpc_3d_entry_vertical_tolerance_m_ <= 0.0 ||
      mpc_3d_entry_vertical_speed_tolerance_mps_ <= 0.0 ||
      mpc_3d_entry_hold_time_s_ < 0.0)
    {
      throw std::invalid_argument("mpc_3d entry tolerances must be positive and hold time non-negative");
    }
    if (offboard_rate_hz_ < 2.0 || offboard_warmup_s_ < 1.0 ||
      preflight_stability_hold_s_ < 0.0)
    {
      throw std::invalid_argument(
              "offboard_rate_hz must be >= 2, offboard_warmup_s >= 1, and "
              "preflight_stability_hold_s non-negative");
    }
    if (target_motion_delay_ < 0.0 || target_motion_ramp_s_ < 0.0 ||
      attack_enable_delay_ < 0.0 ||
      target_stable_motion_time_s_ < 0.0)
    {
      throw std::invalid_argument(
              "target motion delay/ramp, stable-motion, and attack delays must be non-negative");
    }
    if (aim_entry_horizontal_distance_m_ <= 0.0 ||
      aim_exit_horizontal_distance_m_ < aim_entry_horizontal_distance_m_ ||
      capture_minimum_range_m_ <= collision_exit_range_m_ ||
      capture_maximum_range_m_ <= capture_minimum_range_m_ ||
      midcourse_alignment_blend_ < 0.0 ||
      midcourse_alignment_blend_ > terminal_alignment_blend_ ||
      terminal_alignment_blend_ < 0.0 || terminal_alignment_blend_ > 1.0 ||
      terminal_capture_alignment_blend_ < terminal_alignment_blend_ ||
      terminal_capture_alignment_blend_ > 1.0 ||
      terminal_alignment_weight_slew_rate_ <= 0.0 ||
      collision_exit_range_m_ <= 0.0 ||
      alignment_ready_error_rad_ <= 0.0 || takeoff_ready_tolerance_m_ < 0.0 ||
      altitude_correction_threshold_m_ < 0.0 || mission_minimum_target_above_m_ <= 0.0)
    {
      throw std::invalid_argument("invalid mission transition configuration");
    }
    if (required_virtual_captures_ < 1 || reattack_cooldown_s_ < 0.0 ||
      post_capture_recoil_duration_s_ < 0.0 || post_capture_recoil_acceleration_mps2_ < 0.0)
    {
      throw std::invalid_argument(
              "capture count must be positive and cooldown/recoil parameters non-negative");
    }
    if ((pursuer_yaw_mode_ != "fixed" && pursuer_yaw_mode_ != "los" &&
      pursuer_yaw_mode_ != "velocity") || pursuer_yaw_minimum_speed_ < 0.0)
    {
      throw std::invalid_argument(
              "pursuer_yaw_mode must be fixed, los, or velocity with a non-negative speed threshold");
    }
    last_motion_yaw_rad_ = pursuer_fixed_yaw_rad_;

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();

    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      topic("in/offboard_control_mode"), px4_qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic("in/trajectory_setpoint"), px4_qos);
    attitude_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      topic("in/vehicle_attitude_setpoint"), px4_qos);
    rates_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      topic("in/vehicle_rates_setpoint"), px4_qos);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      topic("in/vehicle_command"), px4_qos);
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topic("out/vehicle_local_position"), px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {on_local_position(*msg);});
    if (role_ == "target" || role_ == "pursuer") {
      global_position_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        topic("out/vehicle_global_position"), px4_qos,
        [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
          on_global_position(*msg);
        });
    }
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      topic("out/vehicle_status"), px4_qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        status_ = *msg;
        status_valid_ = true;
        status_update_ = now();
        if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
          lock_task_origin("PX4 armed transition");
        }
      });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      topic("out/vehicle_attitude"), px4_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
        const Quaternion next{msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        const auto sample_time = now();
        if (attitude_valid_) {
          const double dt = (sample_time - attitude_update_).seconds();
          const double dot = std::clamp(std::abs(
            attitude_.w * next.w + attitude_.x * next.x +
            attitude_.y * next.y + attitude_.z * next.z), 0.0, 1.0);
          if (dt > 1e-4 && dt < state_timeout_) {
            const double estimated_rate = 2.0 * std::acos(dot) / dt;
            if (std::isfinite(estimated_rate)) {
              body_rate_rad_s_ = angular_velocity_valid_ ?
                0.8 * body_rate_rad_s_ + 0.2 * estimated_rate : estimated_rate;
              angular_velocity_valid_ = true;
              angular_velocity_update_ = sample_time;
            }
          }
        }
        attitude_ = next;
        attitude_valid_ = true;
        attitude_update_ = sample_time;
        attitude_steady_update_ = std::chrono::steady_clock::now();
      });
    angular_velocity_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      topic("out/vehicle_angular_velocity"), px4_qos,
      [this](px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg) {
        body_rate_rad_s_ = std::sqrt(
          msg->xyz[0] * msg->xyz[0] + msg->xyz[1] * msg->xyz[1] +
          msg->xyz[2] * msg->xyz[2]);
        angular_velocity_valid_ = std::isfinite(body_rate_rad_s_);
        angular_velocity_update_ = now();
      });

    target_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(target_state_topic_, 10);
    if (role_ == "target") {
      auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      ready_qos.reliable().transient_local();
      target_motion_ready_pub_ =
        create_publisher<std_msgs::msg::Bool>(target_motion_ready_topic_, ready_qos);
      if (target_wait_for_pursuer_takeoff_ready_) {
        pursuer_takeoff_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
          pursuer_takeoff_ready_topic_, ready_qos,
          [this](std_msgs::msg::Bool::UniquePtr message) {
            if (message->data && !pursuer_takeoff_ready_) {
              pursuer_takeoff_ready_ = true;
              pursuer_takeoff_ready_time_s_ = now().seconds();
              RCLCPP_INFO(
                get_logger(),
                "pursuer safe takeoff confirmed; target trajectory timer started");
            }
          });
      }
    }
    if (role_ == "target" && target_wait_for_pursuer_intercept_) {
      pure_interceptor_status_sub_ =
        create_subscription<px4_offboard_tracking::msg::PureInterceptorStatus>(
        pure_interceptor_status_topic_, 10,
        [this](px4_offboard_tracking::msg::PureInterceptorStatus::UniquePtr msg) {
          if (!target_motion_gate_open_ &&
            msg->state == px4_offboard_tracking::msg::PureInterceptorStatus::INTERCEPT)
          {
            target_motion_gate_open_ = true;
            target_motion_gate_time_s_ = now().seconds();
            RCLCPP_INFO(
              get_logger(),
              "pursuer entered INTERCEPT; target motion gate opened");
          }
        });
    }
    auto origin_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    origin_qos.reliable().transient_local();
    if (role_ == "pursuer") {
      common_origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
        common_origin_topic_, origin_qos);
      task_origin_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        task_origin_topic_, origin_qos);
    }
    if (role_ == "target") {
      common_origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        common_origin_topic_, origin_qos,
        [this](sensor_msgs::msg::NavSatFix::UniquePtr msg) {on_common_origin(*msg);});
    }
    if (role_ == "pursuer") {
      target_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        target_state_topic_, 10,
        [this](nav_msgs::msg::Odometry::UniquePtr msg) {on_target_state(*msg);});
      relative_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(relative_state_topic_, 10);
      alignment_feasibility_pub_ =
        create_publisher<px4_offboard_tracking::msg::AlignmentFeasibility>(
        alignment_feasibility_topic_, 10);
      mission_status_pub_ = create_publisher<px4_offboard_tracking::msg::MissionStatus>(
        mission_status_topic_, 10);
      tracking_quality_pub_ = create_publisher<px4_offboard_tracking::msg::TrackingQuality>(
        tracking_quality_topic_, 10);
      capture_window_pub_ = create_publisher<px4_offboard_tracking::msg::CaptureWindowStatus>(
        capture_window_topic_, 10);
      virtual_capture_event_pub_ =
        create_publisher<px4_offboard_tracking::msg::VirtualCaptureEvent>(
        virtual_capture_event_topic_, 10);
      safety_status_pub_ = create_publisher<px4_offboard_tracking::msg::SafetyStatus>(
        safety_status_topic_, 10);
      auto mission_complete_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      mission_complete_qos.reliable().transient_local();
      mission_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
        mission_complete_topic_, mission_complete_qos);
      relative_estimate_sub_ =
        create_subscription<px4_offboard_tracking::msg::RelativeEstimate>(
        relative_estimate_topic_, 10,
        [this](px4_offboard_tracking::msg::RelativeEstimate::UniquePtr msg) {
          on_relative_estimate(*msg);
        });
      observation_sub_ = create_subscription<px4_offboard_tracking::msg::TargetObservation>(
        observation_topic_, 10,
        [this](px4_offboard_tracking::msg::TargetObservation::UniquePtr msg) {
          observation_visible_ = msg->visible;
          observation_pixel_u_ = msg->pixel_u;
          observation_pixel_v_ = msg->pixel_v;
          observation_depth_m_ = msg->depth;
          observation_update_ = rclcpp::Time(msg->header.stamp);
          observation_valid_ = observation_update_.nanoseconds() > 0 &&
            (!observation_visible_ ||
            (std::isfinite(observation_pixel_u_) && std::isfinite(observation_pixel_v_) &&
            std::isfinite(observation_depth_m_) && observation_depth_m_ > 0.0));
        });
      relative_kinematics_sub_ =
        create_subscription<px4_offboard_tracking::msg::RelativeKinematics>(
        relative_kinematics_topic_, 10,
        [this](px4_offboard_tracking::msg::RelativeKinematics::UniquePtr msg) {
          relative_kinematics_update_ = rclcpp::Time(msg->header.stamp);
          kinematics_range_ = msg->range;
          kinematics_relative_speed_ = std::sqrt(
            msg->relative_velocity.x * msg->relative_velocity.x +
            msg->relative_velocity.y * msg->relative_velocity.y +
            msg->relative_velocity.z * msg->relative_velocity.z);
          kinematics_los_rate_ = std::sqrt(
            msg->los_rate.x * msg->los_rate.x + msg->los_rate.y * msg->los_rate.y +
            msg->los_rate.z * msg->los_rate.z);
          kinematics_tcpa_ = msg->time_to_closest_approach;
          kinematics_miss_distance_ = msg->predicted_miss_distance;
          relative_kinematics_valid_ = msg->valid &&
            relative_kinematics_update_.nanoseconds() > 0 &&
            std::isfinite(kinematics_range_) && std::isfinite(kinematics_relative_speed_) &&
            std::isfinite(kinematics_los_rate_) && std::isfinite(kinematics_tcpa_) &&
            std::isfinite(kinematics_miss_distance_);
        });
    }

    warmup_setpoint_count_ = static_cast<std::size_t>(
      std::ceil(offboard_rate_hz_ * offboard_warmup_s_));
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / offboard_rate_hz_));
    timer_ = create_wall_timer(timer_period, [this]() {on_timer();});
    RCLCPP_INFO(get_logger(), "role=%s, control=%s, PX4 prefix=%s, target system=%d",
      role_.c_str(), control_level_.c_str(), topic_prefix_.c_str(), target_system_);
    RCLCPP_INFO(get_logger(), "offboard setpoint rate=%.1f Hz, warmup=%.1f s",
      offboard_rate_hz_, offboard_warmup_s_);
    if (role_ == "target") {
      RCLCPP_INFO(
        get_logger(), "target scenario=%s, height=%.1f m, radius=%.1f m, speed=%.1f m/s",
        target_scenario_name_.c_str(), takeoff_height_, target_radius_, target_speed_);
    }
  }

private:
  std::string topic(const std::string & suffix) const
  {
    std::string prefix = topic_prefix_;
    while (prefix.size() > 1 && prefix.back() == '/') {prefix.pop_back();}
    if (prefix.empty() || prefix.front() != '/') {prefix.insert(prefix.begin(), '/');}
    return prefix + "/" + suffix;
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  double altitude_hold_velocity() const
  {
    const double desired_z = local_origin_.z - takeoff_height_;
    return std::clamp(
      altitude_hold_position_gain_ * (desired_z - local_.z) -
      altitude_hold_velocity_gain_ * local_.vz,
      -altitude_hold_max_speed_, altitude_hold_max_speed_);
  }

  double altitude_hold_acceleration() const
  {
    const double desired_z = local_origin_.z - takeoff_height_;
    return std::clamp(
      altitude_hold_position_gain_ * (desired_z - local_.z) -
      altitude_hold_velocity_gain_ * local_.vz,
      -altitude_hold_max_acceleration_, altitude_hold_max_acceleration_);
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    if (!msg.xy_valid || !msg.z_valid || !msg.v_xy_valid || !msg.v_z_valid) {
      return;
    }
    local_ = {msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz};
    local_sample_time_s_ = static_cast<double>(msg.timestamp) * 1.0e-6;
    if (!local_reset_counters_set_) {
      local_xy_reset_counter_ = msg.xy_reset_counter;
      local_z_reset_counter_ = msg.z_reset_counter;
      local_reset_counters_set_ = true;
    } else {
      if (msg.xy_reset_counter != local_xy_reset_counter_) {
        local_continuity_offset_x_ -= msg.delta_xy[0];
        local_continuity_offset_y_ -= msg.delta_xy[1];
        // PX4 reports new_local = old_local + delta. Shift every saved local
        // reference by the same delta so a position setpoint continues to
        // represent the same physical point after an EKF origin reset.
        if (local_origin_set_) {
          local_origin_.x += msg.delta_xy[0];
          local_origin_.y += msg.delta_xy[1];
          publish_task_origin();
        }
        if (latest_global_local_valid_) {
          latest_global_local_position_.x += msg.delta_xy[0];
          latest_global_local_position_.y += msg.delta_xy[1];
        }
        local_xy_reset_counter_ = msg.xy_reset_counter;
        RCLCPP_INFO(
          get_logger(), "compensated PX4 local XY reset by [%.3f, %.3f] m",
          msg.delta_xy[0], msg.delta_xy[1]);
      }
      if (msg.z_reset_counter != local_z_reset_counter_) {
        local_continuity_offset_z_ -= msg.delta_z;
        if (local_origin_set_) {
          local_origin_.z += msg.delta_z;
          publish_task_origin();
        }
        if (latest_global_local_valid_) {
          latest_global_local_position_.z += msg.delta_z;
        }
        local_z_reset_counter_ = msg.z_reset_counter;
        RCLCPP_INFO(
          get_logger(), "compensated PX4 local Z reset by %.3f m", msg.delta_z);
      }
    }
    local_continuous_ = {
      local_.x + local_continuity_offset_x_,
      local_.y + local_continuity_offset_y_,
      local_.z + local_continuity_offset_z_,
      local_.vx, local_.vy, local_.vz};
    local_valid_ = true;
    local_update_ = now();
    local_steady_update_ = std::chrono::steady_clock::now();
    own_.vx = msg.vx;
    own_.vy = msg.vy;
    own_.vz = msg.vz;
    if (!local_origin_set_) {
      local_origin_ = local_;
      local_origin_set_ = true;
      start_time_ = now();
      RCLCPP_INFO(get_logger(), "provisional takeoff origin initialized at [%.2f, %.2f, %.2f]",
        local_.x, local_.y, local_.z);
    } else if (!local_origin_locked_ &&
      (!status_valid_ ||
      status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED))
    {
      local_origin_ = local_;
    }
    update_common_position();
  }

  void on_global_position(const px4_msgs::msg::VehicleGlobalPosition & msg)
  {
    const GeodeticPosition position{msg.lat, msg.lon, msg.alt_ellipsoid};
    if (!CommonFrameTransform::is_valid(position)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "waiting for valid PX4 global position");
      return;
    }

    latest_global_position_ = position;
    if (local_valid_) {
      latest_global_local_position_ = local_;
      latest_global_local_valid_ = true;
    }
    latest_global_eph_ = std::isfinite(msg.eph) ? msg.eph : 0.0;
    latest_global_epv_ = std::isfinite(msg.epv) ? msg.epv : 0.0;
    global_position_received_ = true;

    if (role_ == "pursuer" && local_origin_locked_ && !common_transform_) {
      const CommonFrameTransform current_global_frame(position);
      set_common_origin(current_global_frame.from_ned({
        local_origin_.x - latest_global_local_position_.x,
        local_origin_.y - latest_global_local_position_.y,
        local_origin_.z - latest_global_local_position_.z}));
      publish_common_origin();
    }
    update_common_position();
  }

  void on_common_origin(const sensor_msgs::msg::NavSatFix & msg)
  {
    const GeodeticPosition reference{msg.latitude, msg.longitude, msg.altitude};
    if (!CommonFrameTransform::is_valid(reference)) {
      RCLCPP_WARN(get_logger(), "rejected invalid common NED origin");
      return;
    }
    if (!common_transform_) {
      set_common_origin(reference);
      update_common_position();
    }
  }

  void set_common_origin(const GeodeticPosition & reference)
  {
    common_transform_ = std::make_unique<CommonFrameTransform>(reference);
    RCLCPP_INFO(
      get_logger(), "common NED origin set to lat=%.8f lon=%.8f alt=%.3f m",
      reference.latitude_deg, reference.longitude_deg, reference.altitude_ellipsoid_m);
  }

  void publish_common_origin()
  {
    if (!common_origin_pub_ || !common_transform_) {return;}
    sensor_msgs::msg::NavSatFix out;
    out.header.stamp = now();
    out.header.frame_id = common_frame_id_ + "_origin";
    out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    out.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    out.latitude = common_transform_->reference().latitude_deg;
    out.longitude = common_transform_->reference().longitude_deg;
    out.altitude = common_transform_->reference().altitude_ellipsoid_m;
    out.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    common_origin_pub_->publish(out);
  }

  void publish_task_origin()
  {
    if (!task_origin_pub_ || !local_origin_locked_) {return;}
    geometry_msgs::msg::PointStamped out;
    out.header.stamp = now();
    out.header.frame_id = "px4_local_ned";
    out.point.x = local_origin_.x;
    out.point.y = local_origin_.y;
    out.point.z = local_origin_.z;
    task_origin_pub_->publish(out);
  }

  void lock_task_origin(const char * reason)
  {
    if (local_origin_locked_ || !local_valid_) {return;}
    local_origin_ = local_;
    local_origin_locked_ = true;
    start_time_ = now();
    publish_task_origin();
    if (role_ == "pursuer" && global_position_received_ &&
      latest_global_local_valid_ && !common_transform_)
    {
      const CommonFrameTransform current_global_frame(latest_global_position_);
      set_common_origin(current_global_frame.from_ned({
        local_origin_.x - latest_global_local_position_.x,
        local_origin_.y - latest_global_local_position_.y,
        local_origin_.z - latest_global_local_position_.z}));
      publish_common_origin();
    }
    RCLCPP_INFO(
      get_logger(), "task NED origin locked at propeller start [%.2f, %.2f, %.2f]: %s",
      local_origin_.x, local_origin_.y, local_origin_.z, reason);
  }

  void update_common_position()
  {
    if (!common_transform_ || !global_position_received_ || !local_valid_) {return;}
    if (!common_local_anchor_set_) {
      const auto ned = common_transform_->to_ned(latest_global_position_);
      common_frame_anchor_ = {ned.north_m, ned.east_m, ned.down_m, 0.0, 0.0, 0.0};
      common_local_anchor_ = local_continuous_;
      common_local_anchor_set_ = true;
      RCLCPP_INFO(
        get_logger(), "common/local anchor captured at NED [%.2f, %.2f, %.2f]",
        common_frame_anchor_.x, common_frame_anchor_.y, common_frame_anchor_.z);
    }
    own_.x = common_frame_anchor_.x + local_continuous_.x - common_local_anchor_.x;
    own_.y = common_frame_anchor_.y + local_continuous_.y - common_local_anchor_.y;
    own_.z = common_frame_anchor_.z + local_continuous_.z - common_local_anchor_.z;
    common_position_valid_ = true;
    common_position_update_ = now();
    if (role_ == "target" && local_valid_) {
      publish_target_state();
    }
  }

  void publish_target_state()
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp = common_position_update_;
    out.header.frame_id = common_frame_id_;
    out.child_frame_id = "target/base_link_frd";
    static rclcpp::Time last_state_log{0, 0, RCL_ROS_TIME};
    if ((now() - last_state_log).seconds() > 5.0) {
      last_state_log = now();
      RCLCPP_INFO(get_logger(), "publishing target state on %s", target_state_topic_.c_str());
    }
    out.pose.pose.position.x = own_.x;
    out.pose.pose.position.y = own_.y;
    out.pose.pose.position.z = own_.z;
    out.pose.pose.orientation.w = attitude_.w;
    out.pose.pose.orientation.x = attitude_.x;
    out.pose.pose.orientation.y = attitude_.y;
    out.pose.pose.orientation.z = attitude_.z;
    out.pose.covariance[0] = latest_global_eph_ * latest_global_eph_;
    out.pose.covariance[7] = out.pose.covariance[0];
    out.pose.covariance[14] = latest_global_epv_ * latest_global_epv_;
    out.twist.twist.linear.x = own_.vx;
    out.twist.twist.linear.y = own_.vy;
    out.twist.twist.linear.z = own_.vz;
    target_state_pub_->publish(out);
  }

  void on_target_state(const nav_msgs::msg::Odometry & msg)
  {
    if (msg.header.frame_id != common_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "rejected target state outside common NED frame");
      target_truth_valid_ = false;
      return;
    }
    target_truth_ = {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
      msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z};
    if (!std::isfinite(target_truth_.x) || !std::isfinite(target_truth_.y) ||
      !std::isfinite(target_truth_.z) || !std::isfinite(target_truth_.vx) ||
      !std::isfinite(target_truth_.vy) || !std::isfinite(target_truth_.vz))
    {
      target_truth_valid_ = false;
      return;
    }
    target_truth_valid_ = true;
    target_truth_update_ = rclcpp::Time(msg.header.stamp);
  }

  void on_relative_estimate(const px4_offboard_tracking::msg::RelativeEstimate & msg)
  {
    relative_estimate_valid_ = false;
    relative_estimate_measurement_valid_ = false;
    if (msg.header.frame_id != common_frame_id_ || !std::isfinite(msg.data_age) ||
      msg.data_age < 0.0 || msg.data_age > estimate_prediction_coast_s_)
    {
      return;
    }
    relative_estimate_ = {msg.position.x, msg.position.y, msg.position.z,
      msg.velocity.x, msg.velocity.y, msg.velocity.z};
    relative_acceleration_ = {
      msg.acceleration.x, msg.acceleration.y, msg.acceleration.z};
    const std::array<std::size_t, 6> diagonal{0, 7, 14, 21, 28, 35};
    for (const auto index : diagonal) {
      if (!std::isfinite(msg.covariance[index]) || msg.covariance[index] < 0.0) {return;}
    }
    maximum_position_variance_ = std::max({
      msg.covariance[0], msg.covariance[7], msg.covariance[14]});
    innovation_mahalanobis_ = msg.innovation_mahalanobis;
    if (!std::isfinite(innovation_mahalanobis_) || innovation_mahalanobis_ < 0.0) {return;}
    if (!std::isfinite(relative_estimate_.x) || !std::isfinite(relative_estimate_.y) ||
      !std::isfinite(relative_estimate_.z) || !std::isfinite(relative_estimate_.vx) ||
      !std::isfinite(relative_estimate_.vy) || !std::isfinite(relative_estimate_.vz) ||
      !std::isfinite(relative_acceleration_[0]) ||
      !std::isfinite(relative_acceleration_[1]) ||
      !std::isfinite(relative_acceleration_[2]))
    {
      return;
    }
    relative_estimate_update_ = rclcpp::Time(msg.header.stamp);
    relative_estimate_valid_ = relative_estimate_update_.nanoseconds() > 0;
    relative_estimate_measurement_valid_ = msg.valid && msg.data_age <= state_timeout_;
    if (relative_estimate_measurement_valid_) {
      last_valid_estimate_measurement_time_ = now();
    }
  }

  bool common_position_fresh() const
  {
    return common_position_valid_ &&
           (now() - common_position_update_).seconds() <= state_timeout_;
  }

  bool target_truth_fresh() const
  {
    return target_truth_valid_ && target_truth_update_.nanoseconds() > 0 &&
           (now() - target_truth_update_).seconds() >= 0.0 &&
           (now() - target_truth_update_).seconds() <= state_timeout_;
  }

  bool relative_estimate_fresh() const
  {
    return relative_estimate_valid_ && relative_estimate_update_.nanoseconds() > 0 &&
           (now() - relative_estimate_update_).seconds() >= 0.0 &&
           (now() - relative_estimate_update_).seconds() <= estimate_prediction_coast_s_;
  }

  State3d estimated_target_state() const
  {
    return {own_.x + relative_estimate_.x, own_.y + relative_estimate_.y,
      own_.z + relative_estimate_.z, own_.vx + relative_estimate_.vx,
      own_.vy + relative_estimate_.vy, own_.vz + relative_estimate_.vz};
  }

  void publish_relative_state()
  {
    if (!relative_state_pub_ || !common_position_fresh() || !target_truth_fresh()) {return;}
    nav_msgs::msg::Odometry out;
    out.header.stamp = now();
    out.header.frame_id = common_frame_id_;
    out.child_frame_id = "target_relative_to_pursuer";
    out.pose.pose.position.x = target_truth_.x - own_.x;
    out.pose.pose.position.y = target_truth_.y - own_.y;
    out.pose.pose.position.z = target_truth_.z - own_.z;
    out.twist.twist.linear.x = target_truth_.vx - own_.vx;
    out.twist.twist.linear.y = target_truth_.vy - own_.vy;
    out.twist.twist.linear.z = target_truth_.vz - own_.vz;
    relative_state_pub_->publish(out);
  }

  void on_timer()
  {
    // Keep the PX4 Offboard heartbeat alive even while an estimator sample is
    // temporarily invalid. Motion setpoints remain gated by the checks below.
    publish_control_mode();
    const auto steady_now = std::chrono::steady_clock::now();
    const bool local_fresh = local_valid_ &&
      std::chrono::duration<double>(steady_now - local_steady_update_).count() <= state_timeout_;
    const bool attitude_fresh = attitude_valid_ &&
      std::chrono::duration<double>(steady_now - attitude_steady_update_).count() <= state_timeout_;
    if (!local_fresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for valid PX4 local position");
      if (role_ == "pursuer") {handle_critical_state_timeout(false, attitude_fresh);}
      return;
    }
    if (role_ == "pursuer" &&
      (control_level_ == "attitude" || control_level_ == "body_rate") &&
      !attitude_fresh)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for fresh PX4 attitude");
      handle_critical_state_timeout(true, false);
      return;
    }

    publish_setpoint();
    if (role_ == "pursuer") {
      publish_relative_state();
    }

    if (warmup_count_ < warmup_setpoint_count_) {
      ++warmup_count_;
      return;
    }

    const bool already_armed =
      status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool status_fresh = status_valid_ && status_update_.nanoseconds() > 0 &&
      (now() - status_update_).seconds() >= 0.0 &&
      (now() - status_update_).seconds() <= status_timeout_;
    const bool preflight_ready = status_fresh && status_.pre_flight_checks_pass && local_fresh;
    if (!already_armed) {
      if (!preflight_ready) {
        preflight_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "waiting for continuous PX4 preflight and local-position stability");
        return;
      }
      if (preflight_stable_since_.nanoseconds() == 0) {
        preflight_stable_since_ = now();
      }
      if ((now() - preflight_stable_since_).seconds() < preflight_stability_hold_s_) {
        return;
      }
    }
    if ((now() - last_command_).seconds() > 1.0) {
      if (status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        last_command_ = now();
        RCLCPP_INFO(get_logger(), "requested Offboard mode");
      } else if (!already_armed) {
        lock_task_origin("automatic arm command");
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
        last_command_ = now();
        RCLCPP_INFO(get_logger(), "requested arm after Offboard confirmation");
      }
    }
  }

  void handle_critical_state_timeout(bool local_ok, bool attitude_ok)
  {
    const auto state = mission_state_machine_.state();
    const bool active = state == MissionState::B3_APPROACH || state == MissionState::B4_ALIGN ||
      state == MissionState::B5_ALTITUDE_CORRECTION ||
      state == MissionState::B6_LATERAL_CORRECTION ||
      state == MissionState::B7_QUALITY_GATE || state == MissionState::B8_CAPTURE_WINDOW ||
      state == MissionState::B9_VIRTUAL_CAPTURE || state == MissionState::B10_POST_CAPTURE ||
      state == MissionState::B11_SAFETY_ABORT || state == MissionState::B12_RECOVERY ||
      state == MissionState::B13_SEARCH_SCAN || state == MissionState::B14_SEARCH_HANDOVER ||
      state == MissionState::B15_DEPTH_CONFIRM || state == MissionState::B16_REATTACK_TRACK;
    if (!active) {return;}
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto safety = safety_manager_.update({
      now().seconds(), true, false, local_ok, attitude_ok, false, nan, nan, nan, nan});
    publish_safety_status(safety, nan, nan, nan, nan);
    MissionInputs inputs;
    inputs.safety_abort = safety.abort_requested;
    const auto mission = mission_state_machine_.update(inputs, now().seconds());
    if (mission.transitioned) {
      RCLCPP_WARN(get_logger(), "mission -> %s: %s", mission.state_name.c_str(),
        mission.transition_reason.c_str());
    }
    if (!mission_status_pub_) {return;}
    px4_offboard_tracking::msg::MissionStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = common_frame_id_;
    msg.state = static_cast<std::uint8_t>(mission.state);
    msg.state_name = mission.state_name;
    msg.transition_reason = mission.transition_reason;
    msg.state_elapsed = mission.state_elapsed_s;
    msg.safety_abort = safety.abort_requested;
    mission_status_pub_->publish(msg);
  }

  void publish_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestamp_us();
    msg.position = role_ != "pursuer";
    msg.velocity = role_ == "pursuer" && control_level_ == "velocity";
    msg.attitude = role_ == "pursuer" && control_level_ == "attitude";
    msg.body_rate = role_ == "pursuer" && control_level_ == "body_rate";
    offboard_pub_->publish(msg);
  }

  void publish_setpoint()
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = timestamp_us();
    msg.position = {nan, nan, nan};
    msg.velocity = {nan, nan, nan};
    msg.acceleration = {nan, nan, nan};
    msg.jerk = {nan, nan, nan};
    msg.yaw = nan;
    msg.yawspeed = nan;

    if (role_ == "pursuer") {
      update_search_observation();
      update_startup_search();
    }
    if (role_ == "hold") {
      msg.position = {static_cast<float>(local_origin_.x), static_cast<float>(local_origin_.y),
        static_cast<float>(local_origin_.z - takeoff_height_)};
      msg.yaw = 0.0F;
    } else if (role_ == "target") {
      const double altitude = local_origin_.z - local_.z;
      const bool target_at_flight_height = local_origin_set_ && local_valid_ &&
        std::abs(takeoff_height_ - altitude) <= takeoff_ready_tolerance_m_;
      if (target_at_flight_height && !std::isfinite(target_flight_ready_time_s_)) {
        target_flight_ready_time_s_ = now().seconds();
        RCLCPP_INFO(get_logger(),
          "target reached flight height; trajectory motion starts after %.1f s",
          target_motion_delay_);
      }
      double motion_reference_time_s = target_flight_ready_time_s_;
      if (target_wait_for_pursuer_takeoff_ready_) {
        motion_reference_time_s =
          pursuer_takeoff_ready_ && std::isfinite(target_flight_ready_time_s_) ?
          std::max(pursuer_takeoff_ready_time_s_, target_flight_ready_time_s_) :
          std::numeric_limits<double>::quiet_NaN();
      }
      if (target_wait_for_pursuer_intercept_) {
        motion_reference_time_s =
          target_motion_gate_open_ && std::isfinite(target_flight_ready_time_s_) ?
          std::max(target_motion_gate_time_s_, target_flight_ready_time_s_) :
          std::numeric_limits<double>::quiet_NaN();
      }
      const double wall_elapsed = std::isfinite(motion_reference_time_s) ?
        std::max(0.0, now().seconds() - motion_reference_time_s -
        target_motion_delay_) : 0.0;
      if (wall_elapsed > 0.0 && std::isfinite(local_sample_time_s_)) {
        // ROS wall time and Gazebo/PX4 simulation time diverge whenever the
        // simulator runs below real time. Advancing a moving position target
        // with wall time then creates metre-scale setpoint jumps and turns a
        // nominal 5 m/s target into an 8--11 m/s impulse. Anchor the trajectory
        // to the target PX4 sample clock after the wall-time launch gate opens.
        // PX4 timestamps can themselves jump during SITL clock resynchronization,
        // so integrate only bounded positive increments instead of subtracting
        // two absolute timestamps.
        if (!std::isfinite(target_motion_last_px4_sample_time_s_)) {
          target_motion_last_px4_sample_time_s_ = local_sample_time_s_;
        } else if (local_sample_time_s_ > target_motion_last_px4_sample_time_s_) {
          const double sample_dt_s =
            local_sample_time_s_ - target_motion_last_px4_sample_time_s_;
          target_motion_last_px4_sample_time_s_ = local_sample_time_s_;
          target_trajectory_elapsed_s_ += std::min(sample_dt_s, 0.1);
        }
      }
      const double elapsed = target_trajectory_elapsed_s_;
      if (!target_motion_ready_published_ &&
        elapsed >= target_search_enable_after_motion_s_ && target_motion_ready_pub_)
      {
        std_msgs::msg::Bool ready;
        ready.data = true;
        target_motion_ready_pub_->publish(ready);
        target_motion_ready_published_ = true;
        RCLCPP_INFO(
          get_logger(),
          "target trajectory motion started; pursuer search mission enabled");
      }
      const auto target_setpoint = target_trajectory_.sample_with_speed_ramp(
        local_origin_.x, local_origin_.y, local_origin_.z,
        elapsed, target_motion_ramp_s_);
      msg.position = {
        static_cast<float>(target_setpoint.x), static_cast<float>(target_setpoint.y),
        static_cast<float>(target_setpoint.z)};
      if (target_trajectory_feedforward_enabled_ && elapsed > 0.0) {
        msg.velocity = {
          static_cast<float>(target_setpoint.vx), static_cast<float>(target_setpoint.vy),
          static_cast<float>(target_setpoint.vz)};
        msg.acceleration = {
          static_cast<float>(target_setpoint.ax), static_cast<float>(target_setpoint.ay),
          static_cast<float>(target_setpoint.az)};
      }
      msg.yaw = static_cast<float>(target_setpoint.yaw);
    } else if (control_level_ == "velocity") {
      VelocityCommand command;
      if (startup_search_result_.pending) {
        command.yaw = startup_search_result_.desired_yaw_rad;
      } else if (common_position_fresh() && relative_estimate_fresh()) {
        command = controller_.update(own_, estimated_target_state());
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "valid relative EKF estimate unavailable or stale; holding horizontal velocity at zero");
      }
      command.vz = altitude_hold_velocity();
      msg.velocity = {static_cast<float>(command.vx), static_cast<float>(command.vy),
        static_cast<float>(command.vz)};
      msg.yaw = static_cast<float>(command.yaw);
    } else {
      publish_low_level_setpoint();
      return;
    }
    setpoint_pub_->publish(msg);
  }

  double desired_pursuer_yaw(double los_yaw)
  {
    if (pursuer_yaw_mode_ == "fixed") {return pursuer_fixed_yaw_rad_;}
    if (pursuer_yaw_mode_ == "los") {return los_yaw;}
    const double horizontal_speed = std::hypot(local_.vx, local_.vy);
    if (horizontal_speed >= pursuer_yaw_minimum_speed_) {
      last_motion_yaw_rad_ = std::atan2(local_.vy, local_.vx);
    }
    return last_motion_yaw_rad_;
  }

  double current_yaw() const
  {
    return std::atan2(
      2.0 * (attitude_.w * attitude_.z + attitude_.x * attitude_.y),
      1.0 - 2.0 * (attitude_.y * attitude_.y + attitude_.z * attitude_.z));
  }

  void update_search_observation()
  {
    search_observation_ = {};
    search_los_ned_valid_ = false;
    if (!startup_search_.enabled() || !target_truth_fresh() ||
      !common_position_fresh() || !attitude_valid_)
    {
      return;
    }
    const Vector3d relative_ned{
      target_truth_.x - own_.x,
      target_truth_.y - own_.y,
      target_truth_.z - own_.z};
    search_observation_ = search_camera_model_.observe(
      relative_ned,
      Quaterniond{attitude_.w, attitude_.x, attitude_.y, attitude_.z});
    if (!search_observation_.visible) {return;}

    const auto & optical = search_observation_.unit_line_of_sight_optical;
    constexpr double sine = 0.17364817766693033;
    constexpr double cosine = 0.98480775301220802;
    const std::array<double, 3> body{
      cosine * optical.y + sine * optical.z,
      optical.x,
      sine * optical.y - cosine * optical.z};
    const double w = attitude_.w;
    const double x = attitude_.x;
    const double y = attitude_.y;
    const double z = attitude_.z;
    search_los_ned_ = {
      (1.0 - 2.0 * (y * y + z * z)) * body[0] +
      2.0 * (x * y - w * z) * body[1] +
      2.0 * (x * z + w * y) * body[2],
      2.0 * (x * y + w * z) * body[0] +
      (1.0 - 2.0 * (x * x + z * z)) * body[1] +
      2.0 * (y * z - w * x) * body[2],
      2.0 * (x * z - w * y) * body[0] +
      2.0 * (y * z + w * x) * body[1] +
      (1.0 - 2.0 * (x * x + y * y)) * body[2]};
    const double norm = std::sqrt(
      search_los_ned_[0] * search_los_ned_[0] +
      search_los_ned_[1] * search_los_ned_[1] +
      search_los_ned_[2] * search_los_ned_[2]);
    if (std::isfinite(norm) && norm > 1e-9) {
      for (double & component : search_los_ned_) {component /= norm;}
      search_los_ned_valid_ = true;
    }
  }

  void update_startup_search()
  {
    const double altitude = local_origin_.z - local_.z;
    const bool takeoff_ready = local_origin_set_ && attitude_valid_ &&
      std::abs(takeoff_height_ - altitude) <= takeoff_ready_tolerance_m_;
    const bool was_completed = startup_search_result_.completed;
    startup_search_result_ = startup_search_.update(
      takeoff_ready, search_observation_.visible, now().seconds(), current_yaw());
    if (startup_search_result_.active && !startup_search_started_logged_) {
      startup_search_started_logged_ = true;
      RCLCPP_INFO(get_logger(), "startup search started");
    }
    if (!was_completed && startup_search_result_.completed) {
      RCLCPP_INFO(
        get_logger(), "startup search completed (%s); handing control to tracking chain",
        startup_search_result_.target_acquired ? "target acquired" : "sweep complete");
    }
  }

  AccelerationCommand acceleration_command()
  {
    if (startup_search_result_.pending) {
      AccelerationCommand search;
      search.ax = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vx,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      search.ay = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vy,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      search.az = altitude_hold_acceleration();
      search.yaw = startup_search_result_.desired_yaw_rad;
      return search;
    }
    const auto mission_state = mission_state_machine_.state();
    if (mission_state == MissionState::B13_SEARCH_SCAN ||
      mission_state == MissionState::B14_SEARCH_HANDOVER ||
      mission_state == MissionState::B15_DEPTH_CONFIRM ||
      mission_state == MissionState::B17_MISSION_COMPLETE)
    {
      AccelerationCommand hold;
      hold.ax = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vx,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      hold.ay = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vy,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      hold.az = altitude_hold_acceleration();
      hold.yaw = desired_pursuer_yaw(last_motion_yaw_rad_);
      return hold;
    }
    if (mission_state == MissionState::B10_POST_CAPTURE) {
      AccelerationCommand recoil;
      const double norm = std::sqrt(
        relative_estimate_.x * relative_estimate_.x +
        relative_estimate_.y * relative_estimate_.y +
        relative_estimate_.z * relative_estimate_.z);
      if (relative_estimate_fresh() && norm > 1e-6) {
        const double scale = -post_capture_recoil_acceleration_mps2_ / norm;
        recoil.ax = scale * relative_estimate_.x;
        recoil.ay = scale * relative_estimate_.y;
        recoil.az = scale * relative_estimate_.z;
      } else {
        recoil.ax = std::clamp(
          -invalid_estimate_velocity_damping_ * local_.vx,
          -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
        recoil.ay = std::clamp(
          -invalid_estimate_velocity_damping_ * local_.vy,
          -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
        recoil.az = altitude_hold_acceleration();
      }
      recoil.yaw = desired_pursuer_yaw(last_motion_yaw_rad_);
      return recoil;
    }
    if (mission_state == MissionState::B11_SAFETY_ABORT ||
      (mission_state == MissionState::B12_RECOVERY && !post_capture_recovery_active_))
    {
      AccelerationCommand recovery;
      recovery.ax = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vx,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      recovery.ay = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vy,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      recovery.az = altitude_hold_acceleration();
      recovery.yaw = desired_pursuer_yaw(last_motion_yaw_rad_);
      return recovery;
    }
    if (mission_state == MissionState::B16_REATTACK_TRACK &&
      virtual_capture_event_id_ > 0 &&
      (now() - last_virtual_capture_time_).seconds() < reattack_cooldown_s_)
    {
      // A capture event terminates the collision-like course immediately. Remove
      // all three velocity components during the mandatory cooldown so vertical
      // momentum cannot carry the vehicle through the safety envelope before the
      // next MPC/APN entry.
      AccelerationCommand cooldown_brake;
      cooldown_brake.ax = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vx,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      cooldown_brake.ay = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vy,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      cooldown_brake.az = std::clamp(
        -invalid_estimate_velocity_damping_ * local_.vz,
        -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
      cooldown_brake.yaw = desired_pursuer_yaw(last_motion_yaw_rad_);
      return cooldown_brake;
    }
    if (common_position_fresh() && relative_estimate_fresh()) {
      const bool terminal_intercept =
        mission_state == MissionState::B4_ALIGN ||
        mission_state == MissionState::B7_QUALITY_GATE ||
        mission_state == MissionState::B8_CAPTURE_WINDOW;
      if (terminal_intercept) {
        const auto result = terminal_guidance_.update(
          {relative_estimate_.x, relative_estimate_.y, relative_estimate_.z},
          {relative_estimate_.vx, relative_estimate_.vy, relative_estimate_.vz},
          relative_acceleration_);
        if (result.valid) {
          auto command = result.acceleration;
          command.yaw = desired_pursuer_yaw(command.yaw);
          return command;
        }
      }
      if (guidance_controller_ == "mpc" || guidance_controller_ == "mpc_3d") {
        const auto result = mpc_guidance_.update(
          {relative_estimate_.x, relative_estimate_.y, relative_estimate_.z},
          {relative_estimate_.vx, relative_estimate_.vy, relative_estimate_.vz},
          relative_acceleration_);
        if (result.valid) {
          auto command = result.acceleration;
          command.yaw = desired_pursuer_yaw(command.yaw);
          const bool tracking_phase = mission_state == MissionState::B3_APPROACH ||
            mission_state == MissionState::B4_ALIGN ||
            mission_state == MissionState::B5_ALTITUDE_CORRECTION ||
            mission_state == MissionState::B6_LATERAL_CORRECTION ||
            mission_state == MissionState::B7_QUALITY_GATE ||
            mission_state == MissionState::B8_CAPTURE_WINDOW ||
            mission_state == MissionState::B9_VIRTUAL_CAPTURE ||
            mission_state == MissionState::B16_REATTACK_TRACK;
          if (guidance_controller_ != "mpc_3d" || !tracking_phase) {
            command.az = altitude_hold_acceleration();
          } else {
            command.az =
              (1.0 - mpc_vertical_absolute_hold_blend_) * command.az +
              mpc_vertical_absolute_hold_blend_ * altitude_hold_acceleration();
          }
          return command;
        }
      }
      const auto corridor = approach_corridor_.update(
        {relative_estimate_.x, relative_estimate_.y, relative_estimate_.z},
        {relative_estimate_.vx, relative_estimate_.vy, relative_estimate_.vz});
      if (corridor.valid) {
        auto command = corridor.acceleration;
        command.yaw = desired_pursuer_yaw(command.yaw);
        command.az = altitude_hold_acceleration();
        return command;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "relative estimate does not satisfy upward-camera corridor geometry");
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "valid corridor estimate unavailable; braking horizontally and holding altitude");
    AccelerationCommand command;
    command.ax = std::clamp(
      -invalid_estimate_velocity_damping_ * local_.vx,
      -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
    command.ay = std::clamp(
      -invalid_estimate_velocity_damping_ * local_.vy,
      -invalid_estimate_maximum_acceleration_, invalid_estimate_maximum_acceleration_);
    command.az = altitude_hold_acceleration();
    command.yaw = desired_pursuer_yaw(last_motion_yaw_rad_);
    return command;
  }

  void publish_low_level_setpoint()
  {
    const auto requested_acceleration = acceleration_command();
    const auto speed_limited = horizontal_speed_limiter_.apply(
      requested_acceleration, local_.vx, local_.vy);
    const auto acceleration = speed_limited.acceleration;
    if (speed_limited.overspeed_braking_active) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "horizontal speed %.2f m/s above command envelope; bounded braking active",
        speed_limited.horizontal_speed_mps);
    }
    const auto fallback = geometric_control_.attitude_from_acceleration(acceleration);
    auto desired_camera_axis = ZAxisAlignment::body_minus_z_axis_ned(fallback.attitude);
    const auto current_mission_state = mission_state_machine_.state();
    if ((current_mission_state == MissionState::B14_SEARCH_HANDOVER ||
      current_mission_state == MissionState::B15_DEPTH_CONFIRM) &&
      search_los_ned_valid_)
    {
      auto limited_search_axis = search_los_ned_;
      const double horizontal_norm = std::hypot(
        limited_search_axis[0], limited_search_axis[1]);
      const double maximum_horizontal =
        std::tan(search_handover_maximum_tilt_rad_) *
        std::max(1e-6, -limited_search_axis[2]);
      if (horizontal_norm > maximum_horizontal && horizontal_norm > 1e-9) {
        const double scale = maximum_horizontal / horizontal_norm;
        limited_search_axis[0] *= scale;
        limited_search_axis[1] *= scale;
      }
      const double limited_norm = std::sqrt(
        limited_search_axis[0] * limited_search_axis[0] +
        limited_search_axis[1] * limited_search_axis[1] +
        limited_search_axis[2] * limited_search_axis[2]);
      if (limited_norm > 1e-9) {
        for (double & component : limited_search_axis) {
          component /= limited_norm;
        }
      }

      const double search_weight =
        (speed_limited.outward_acceleration_limited ||
        speed_limited.overspeed_braking_active) ? 0.0 :
        search_handover_alignment_blend_;
      for (std::size_t axis = 0; axis < desired_camera_axis.size(); ++axis) {
        desired_camera_axis[axis] =
          (1.0 - search_weight) * desired_camera_axis[axis] +
          search_weight * limited_search_axis[axis];
      }
      const double blended_norm = std::sqrt(
        desired_camera_axis[0] * desired_camera_axis[0] +
        desired_camera_axis[1] * desired_camera_axis[1] +
        desired_camera_axis[2] * desired_camera_axis[2]);
      if (blended_norm > 1e-9) {
        for (double & component : desired_camera_axis) {
          component /= blended_norm;
        }
      }
    }
    ZAxisAlignmentResult alignment;
    if (common_position_fresh() && relative_estimate_fresh()) {
      alignment = z_axis_alignment_.update(
        {relative_estimate_.x, relative_estimate_.y, relative_estimate_.z},
        {relative_estimate_.vx, relative_estimate_.vy, relative_estimate_.vz}, attitude_);
      if (alignment.valid) {
        const auto mission_state = mission_state_machine_.state();
        const bool aiming_active = mission_state == MissionState::B3_APPROACH ||
          mission_state == MissionState::B4_ALIGN ||
          mission_state == MissionState::B7_QUALITY_GATE ||
          mission_state == MissionState::B8_CAPTURE_WINDOW;
        // Keep APN translation authority during terminal entry, then increase
        // camera centering continuously after the quality/capture phases begin.
        // The slew limiter prevents a rigid-camera attitude step at state changes.
        const double target_weight = !aiming_active ? 0.0 :
          (mission_state == MissionState::B3_APPROACH ? midcourse_alignment_blend_ :
          ((mission_state == MissionState::B7_QUALITY_GATE ||
          mission_state == MissionState::B8_CAPTURE_WINDOW) ?
          terminal_capture_alignment_blend_ : terminal_alignment_blend_));
        const double maximum_weight_step =
          terminal_alignment_weight_slew_rate_ / offboard_rate_hz_;
        terminal_alignment_weight_ += std::clamp(
          target_weight - terminal_alignment_weight_,
          -maximum_weight_step, maximum_weight_step);
        const double weight = std::clamp(terminal_alignment_weight_, 0.0, 1.0);
        for (std::size_t axis = 0; axis < desired_camera_axis.size(); ++axis) {
          desired_camera_axis[axis] =
            (1.0 - weight) * desired_camera_axis[axis] +
            weight * alignment.desired_camera_axis_ned[axis];
        }
        const double axis_norm = std::sqrt(
          desired_camera_axis[0] * desired_camera_axis[0] +
          desired_camera_axis[1] * desired_camera_axis[1] +
          desired_camera_axis[2] * desired_camera_axis[2]);
        if (axis_norm > 1e-9) {
          for (double & component : desired_camera_axis) {component /= axis_norm;}
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "predicted LOS is invalid for body -Z alignment; using braking attitude");
      }
    } else {
      const double maximum_weight_step =
        terminal_alignment_weight_slew_rate_ / offboard_rate_hz_;
      terminal_alignment_weight_ = std::max(
        0.0, terminal_alignment_weight_ - maximum_weight_step);
    }
    const auto desired = attitude_setpoint_generator_.update(
      desired_camera_axis, acceleration.yaw, acceleration.az, attitude_,
      1.0 / offboard_rate_hz_);
    if (control_level_ == "attitude") {
      px4_msgs::msg::VehicleAttitudeSetpoint msg;
      msg.timestamp = timestamp_us();
      msg.roll_body = static_cast<float>(desired.roll);
      msg.pitch_body = static_cast<float>(desired.pitch);
      msg.yaw_body = static_cast<float>(desired.yaw);
      msg.q_d = {static_cast<float>(desired.attitude.w), static_cast<float>(desired.attitude.x),
        static_cast<float>(desired.attitude.y), static_cast<float>(desired.attitude.z)};
      msg.thrust_body = {0.0F, 0.0F, static_cast<float>(-desired.thrust)};
      attitude_setpoint_pub_->publish(msg);
    } else {
      const auto rates = body_rate_controller_.update(attitude_, desired);
      px4_msgs::msg::VehicleRatesSetpoint msg;
      msg.timestamp = timestamp_us();
      msg.roll = static_cast<float>(rates.roll_rate);
      msg.pitch = static_cast<float>(rates.pitch_rate);
      msg.yaw = static_cast<float>(rates.yaw_rate);
      msg.thrust_body = {0.0F, 0.0F, static_cast<float>(-rates.thrust)};
      rates_setpoint_pub_->publish(msg);
    }
    publish_alignment_feasibility(desired, alignment);
    update_and_publish_mission_status();
  }

  void publish_alignment_feasibility(
    const AttitudeThrustCommand & desired, const ZAxisAlignmentResult & alignment)
  {
    if (!alignment_feasibility_pub_) {return;}
    const auto mission_state = mission_state_machine_.state();
    const bool alignment_active = alignment.valid &&
      (mission_state == MissionState::B4_ALIGN || mission_state == MissionState::B7_QUALITY_GATE ||
      mission_state == MissionState::B8_CAPTURE_WINDOW);
    if (alignment_active && alignment_start_.nanoseconds() == 0) {alignment_start_ = now();}
    if (!alignment_active) {alignment_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);}
    const double alignment_duration = alignment_start_.nanoseconds() > 0 ?
      std::max(0.0, (now() - alignment_start_).seconds()) : 0.0;
    const auto commanded_axis = ZAxisAlignment::body_minus_z_axis_ned(desired.attitude);
    const double commanded_tilt = std::acos(std::clamp(-commanded_axis[2], -1.0, 1.0));
    const double altitude = local_origin_.z - local_.z;
    const double altitude_loss = std::max(0.0, takeoff_height_ - altitude);
    const double range = relative_estimate_fresh() ? std::sqrt(
      relative_estimate_.x * relative_estimate_.x + relative_estimate_.y * relative_estimate_.y +
      relative_estimate_.z * relative_estimate_.z) : 0.0;
    const bool body_rate_fresh = angular_velocity_valid_ &&
      (now() - angular_velocity_update_).seconds() <= state_timeout_;
    auto result = alignment_feasibility_.evaluate({
      commanded_tilt, desired.thrust, feasibility_maximum_available_thrust_,
      body_rate_fresh ? body_rate_rad_s_ : std::numeric_limits<double>::quiet_NaN(),
      altitude_loss, range, alignment_duration});

    px4_offboard_tracking::msg::AlignmentFeasibility msg;
    msg.header.stamp = now();
    msg.header.frame_id = common_frame_id_;
    msg.feasible = result.feasible && alignment.valid && relative_estimate_fresh();
    msg.tilt_ok = result.tilt_ok;
    msg.thrust_margin_ok = result.thrust_margin_ok;
    msg.body_rate_ok = result.body_rate_ok;
    msg.altitude_loss_ok = result.altitude_loss_ok;
    msg.minimum_range_ok = result.minimum_range_ok;
    msg.alignment_time_ok = result.alignment_time_ok;
    msg.commanded_tilt = commanded_tilt;
    msg.commanded_thrust = desired.thrust;
    msg.thrust_margin = result.thrust_margin;
    msg.body_rate = body_rate_fresh ? body_rate_rad_s_ :
      std::numeric_limits<double>::quiet_NaN();
    msg.altitude_loss = altitude_loss;
    msg.range = range;
    msg.alignment_duration = alignment_duration;
    alignment_feasibility_pub_->publish(msg);
    last_alignment_feasibility_ = result;
    last_alignment_feasibility_.feasible = msg.feasible;
    last_alignment_ = alignment;
  }

  void update_and_publish_mission_status()
  {
    if (!mission_status_pub_) {return;}
    const bool estimate_prediction_valid = common_position_fresh() && relative_estimate_fresh();
    const bool estimate_measurement_held =
      last_valid_estimate_measurement_time_.nanoseconds() > 0 &&
      (now() - last_valid_estimate_measurement_time_).seconds() >= 0.0 &&
      (now() - last_valid_estimate_measurement_time_).seconds() <=
      estimate_measurement_hold_s_;
    const bool estimate_valid = estimate_prediction_valid &&
      (relative_estimate_measurement_valid_ || estimate_measurement_held);
    const double horizontal_distance = estimate_prediction_valid ?
      std::hypot(relative_estimate_.x, relative_estimate_.y) :
      std::numeric_limits<double>::infinity();
    const double range = estimate_prediction_valid ? std::sqrt(
      relative_estimate_.x * relative_estimate_.x + relative_estimate_.y * relative_estimate_.y +
      relative_estimate_.z * relative_estimate_.z) :
      std::numeric_limits<double>::infinity();
    const bool capture_geometry_ready = estimate_prediction_valid &&
      range >= capture_minimum_range_m_ && range <= capture_maximum_range_m_;
    const double altitude = local_origin_.z - local_.z;
    const double altitude_error = std::abs(takeoff_height_ - altitude);
    const auto current_state = mission_state_machine_.state();
    const auto body_minus_z = ZAxisAlignment::body_minus_z_axis_ned(attitude_);
    const double actual_tilt = std::acos(std::clamp(-body_minus_z[2], -1.0, 1.0));
    const double own_speed = std::sqrt(
      local_.vx * local_.vx + local_.vy * local_.vy + local_.vz * local_.vz);
    const double direct_depth_age = observation_valid_ ?
      (now() - observation_update_).seconds() : std::numeric_limits<double>::infinity();
    const bool direct_depth_fresh = observation_valid_ && observation_visible_ &&
      direct_depth_age >= 0.0 && direct_depth_age <= 0.2;
    const double safety_range = direct_depth_fresh ? observation_depth_m_ :
      std::numeric_limits<double>::quiet_NaN();
    const bool collision_avoidance_trigger =
      direct_depth_fresh && observation_depth_m_ < collision_exit_range_m_;
    if (collision_avoidance_trigger) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "depth range %.2f m is below %.2f m collision exit threshold; braking",
        observation_depth_m_, collision_exit_range_m_);
    }
    const bool safety_monitoring_active =
      current_state == MissionState::B3_APPROACH || current_state == MissionState::B4_ALIGN ||
      current_state == MissionState::B5_ALTITUDE_CORRECTION ||
      current_state == MissionState::B6_LATERAL_CORRECTION ||
      current_state == MissionState::B7_QUALITY_GATE ||
      current_state == MissionState::B8_CAPTURE_WINDOW ||
      current_state == MissionState::B9_VIRTUAL_CAPTURE ||
      current_state == MissionState::B10_POST_CAPTURE ||
      current_state == MissionState::B11_SAFETY_ABORT ||
      current_state == MissionState::B12_RECOVERY ||
      current_state == MissionState::B13_SEARCH_SCAN ||
      current_state == MissionState::B14_SEARCH_HANDOVER ||
      current_state == MissionState::B15_DEPTH_CONFIRM ||
      current_state == MissionState::B16_REATTACK_TRACK;
    const auto safety = safety_manager_.update({
      now().seconds(), safety_monitoring_active,
      current_state == MissionState::B12_RECOVERY,
      local_valid_ && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - local_steady_update_).count() <= state_timeout_,
      attitude_valid_ && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - attitude_steady_update_).count() <= state_timeout_,
      direct_depth_fresh, safety_range, actual_tilt, own_speed, altitude_error});
    if (safety.abort_requested) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "safety abort: %s (range=%.2f m, tilt=%.3f rad, speed=%.2f m/s, "
        "altitude_error=%.2f m)",
        safety.reason.c_str(), safety_range, actual_tilt, own_speed, altitude_error);
      post_capture_recovery_active_ = false;
    }
    publish_safety_status(safety, safety_range, actual_tilt, own_speed, altitude_error);
    const double mission_elapsed_s = local_origin_set_ ?
      std::max(0.0, (now() - start_time_).seconds()) : 0.0;
    const double target_horizontal_speed = target_truth_valid_ ?
      std::hypot(target_truth_.vx, target_truth_.vy) : 0.0;
    const bool target_motion_raw_stable = target_truth_fresh() &&
      (target_speed_ <= 0.1 || target_horizontal_speed >= 0.5 * target_speed_);
    if (virtual_capture_event_id_ == 0 && target_motion_raw_stable) {
      if (!std::isfinite(target_motion_stable_since_s_)) {
        target_motion_stable_since_s_ = now().seconds();
      }
    } else if (virtual_capture_event_id_ == 0) {
      target_motion_stable_since_s_ = std::numeric_limits<double>::quiet_NaN();
    }
    const bool target_motion_stable = virtual_capture_event_id_ > 0 ||
      (std::isfinite(target_motion_stable_since_s_) &&
      now().seconds() - target_motion_stable_since_s_ >= target_stable_motion_time_s_);
    const bool attack_enabled = mission_elapsed_s >= attack_enable_delay_ && target_motion_stable;
    const bool reattack_ready = virtual_capture_event_id_ == 0 ||
      (now() - last_virtual_capture_time_).seconds() >= reattack_cooldown_s_;
    const bool approach_ready = startup_search_result_.completed && attack_enabled &&
      reattack_ready && estimate_valid &&
      horizontal_distance <= aim_entry_horizontal_distance_m_;
    const bool lateral_correction_needed =
      (current_state == MissionState::B4_ALIGN ||
      current_state == MissionState::B6_LATERAL_CORRECTION ||
      current_state == MissionState::B7_QUALITY_GATE) &&
      horizontal_distance > aim_exit_horizontal_distance_m_;
    const bool alignment_ready = last_alignment_.valid &&
      last_alignment_.alignment_error_rad <= alignment_ready_error_rad_ &&
      last_alignment_feasibility_.feasible && capture_geometry_ready;
    const double observation_age = observation_valid_ ?
      (now() - observation_update_).seconds() : std::numeric_limits<double>::quiet_NaN();
    const double estimate_age = relative_estimate_valid_ ?
      (now() - relative_estimate_update_).seconds() : std::numeric_limits<double>::quiet_NaN();
    const auto quality = tracking_quality_gate_.evaluate({
      estimate_valid, maximum_position_variance_, innovation_mahalanobis_,
      observation_valid_ && observation_visible_, observation_age, estimate_age,
      last_alignment_.tilt_limited, last_alignment_feasibility_.feasible});
    publish_tracking_quality(quality, observation_age, estimate_age);
    const bool kinematics_fresh = relative_kinematics_valid_ &&
      (now() - relative_kinematics_update_).seconds() >= 0.0 &&
      (now() - relative_kinematics_update_).seconds() <= state_timeout_;
    const double pixel_error = observation_valid_ && observation_visible_ ? std::hypot(
      observation_pixel_u_ - capture_camera_cx_, observation_pixel_v_ - capture_camera_cy_) :
      std::numeric_limits<double>::quiet_NaN();
    const bool capture_active = current_state == MissionState::B8_CAPTURE_WINDOW;
    const auto capture = capture_window_.update({
      now().seconds(), observation_valid_ && observation_visible_, observation_update_.seconds(),
      capture_active && quality.passed, capture_geometry_ready, pixel_error,
      last_alignment_.valid ? last_alignment_.alignment_error_rad :
      std::numeric_limits<double>::quiet_NaN(),
      kinematics_fresh ? kinematics_los_rate_ : std::numeric_limits<double>::quiet_NaN(),
      kinematics_fresh ? kinematics_miss_distance_ : std::numeric_limits<double>::quiet_NaN(),
      kinematics_fresh ? kinematics_tcpa_ : std::numeric_limits<double>::quiet_NaN()});
    publish_capture_window(capture, pixel_error);

    const bool depth_observation_visible = observation_valid_ && observation_visible_ &&
      observation_age >= 0.0 && observation_age <= 0.2;
    const bool depth_confirmation_active =
      current_state == MissionState::B14_SEARCH_HANDOVER ||
      current_state == MissionState::B15_DEPTH_CONFIRM;
    if (!depth_confirmation_active || !depth_observation_visible) {
      depth_confirmation_started_s_ = std::numeric_limits<double>::quiet_NaN();
      depth_confirmation_frames_ = 0;
      last_depth_confirmation_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    } else if (observation_update_ != last_depth_confirmation_stamp_) {
      if (!std::isfinite(depth_confirmation_started_s_) ||
        (last_depth_confirmation_stamp_.nanoseconds() > 0 &&
        (observation_update_ - last_depth_confirmation_stamp_).seconds() > state_timeout_))
      {
        depth_confirmation_started_s_ = observation_update_.seconds();
        depth_confirmation_frames_ = 0;
      }
      last_depth_confirmation_stamp_ = observation_update_;
      ++depth_confirmation_frames_;
    }
    const bool depth_confirmed = depth_confirmation_active &&
      depth_observation_visible && std::isfinite(depth_confirmation_started_s_) &&
      observation_update_.seconds() - depth_confirmation_started_s_ >=
      depth_confirmation_hold_time_s_ &&
      depth_confirmation_frames_ >= depth_confirmation_minimum_frames_;

    MissionInputs inputs;
    inputs.system_ready = local_valid_ && attitude_valid_;
    inputs.estimate_valid = estimate_valid;
    inputs.estimate_prediction_valid = estimate_prediction_valid;
    inputs.search_enabled = startup_search_.enabled();
    inputs.search_target_acquired =
      (current_state == MissionState::B1_WAIT_ESTIMATE ||
      current_state == MissionState::B2_TAKEOFF_HOLD ||
      current_state == MissionState::B13_SEARCH_SCAN) ?
      startup_search_result_.target_acquired : search_observation_.visible;
    inputs.depth_observation_visible = depth_observation_visible;
    inputs.depth_confirmed = depth_confirmed;
    const bool mpc_3d_first_entry_gate_active = guidance_controller_ == "mpc_3d" &&
      (current_state == MissionState::B1_WAIT_ESTIMATE ||
      current_state == MissionState::B2_TAKEOFF_HOLD) && virtual_capture_event_id_ == 0;
    const bool mpc_3d_vertical_raw_ready = estimate_valid &&
      std::abs(-relative_estimate_.z - mpc_desired_vertical_separation_m_) <=
      mpc_3d_entry_vertical_tolerance_m_ &&
      std::abs(relative_estimate_.vz) <= mpc_3d_entry_vertical_speed_tolerance_mps_;
    if (mpc_3d_first_entry_gate_active && mpc_3d_vertical_raw_ready) {
      if (!std::isfinite(mpc_3d_entry_ready_since_s_)) {
        mpc_3d_entry_ready_since_s_ = now().seconds();
      }
    } else if (mpc_3d_first_entry_gate_active) {
      mpc_3d_entry_ready_since_s_ = std::numeric_limits<double>::quiet_NaN();
    }
    const bool mpc_3d_vertical_entry_ready = !mpc_3d_first_entry_gate_active ||
      (std::isfinite(mpc_3d_entry_ready_since_s_) &&
      now().seconds() - mpc_3d_entry_ready_since_s_ >= mpc_3d_entry_hold_time_s_);
    inputs.takeoff_ready = altitude_error <= takeoff_ready_tolerance_m_ &&
      (startup_search_.enabled() || mpc_3d_vertical_entry_ready);
    inputs.target_above = estimate_prediction_valid &&
      relative_estimate_.z <= -mission_minimum_target_above_m_;
    inputs.approach_ready = approach_ready;
    inputs.alignment_ready = alignment_ready;
    // B2 already guarantees a stable takeoff height. Once relative tracking starts,
    // three-dimensional MPC and terminal APN own the vertical channel.
    inputs.altitude_correction_needed = false;
    inputs.lateral_correction_needed = lateral_correction_needed;
    inputs.alignment_feasible = last_alignment_feasibility_.feasible;
    inputs.quality_gate_passed = quality.passed && capture_geometry_ready;
    inputs.capture_window_passed = capture.held;
    inputs.virtual_capture_published = virtual_capture_published_;
    inputs.post_capture_recoil_complete = virtual_capture_event_id_ > 0 &&
      (now() - last_virtual_capture_time_).seconds() >= post_capture_recoil_duration_s_;
    inputs.reattack_ready = reattack_ready;
    inputs.mission_complete = mission_complete_;
    inputs.safety_abort = safety.abort_requested || collision_avoidance_trigger;
    // Post-capture recovery only waits for the mandatory cooldown and a usable
    // predicted estimate. Geometry, speed, altitude and alignment are then
    // re-evaluated by B2/B3/B4 instead of trapping the mission in B12 until all
    // entry gates happen to become true at the same instant.
    const bool post_capture_tracking_recovered =
      reattack_ready && estimate_prediction_valid;
    inputs.recovery_complete = post_capture_recovery_active_ ?
      post_capture_tracking_recovered : safety.recovery_complete;

    const auto mission = mission_state_machine_.update(inputs, now().seconds());
    if (mission.state == MissionState::B9_VIRTUAL_CAPTURE && !virtual_capture_published_) {
      publish_virtual_capture_event(pixel_error);
      virtual_capture_published_ = true;
      mission_complete_ = virtual_capture_event_id_ >=
        static_cast<std::uint64_t>(required_virtual_captures_);
      post_capture_recovery_active_ = !mission_complete_;
    } else if (mission.state != MissionState::B9_VIRTUAL_CAPTURE &&
      mission.state != MissionState::B10_POST_CAPTURE)
    {
      virtual_capture_published_ = false;
    }
    if (mission.transitioned) {
      RCLCPP_INFO(get_logger(), "mission -> %s: %s", mission.state_name.c_str(),
        mission.transition_reason.c_str());
      if (current_state == MissionState::B12_RECOVERY &&
        mission.state == MissionState::B1_WAIT_ESTIMATE)
      {
        post_capture_recovery_active_ = false;
      }
      if (current_state == MissionState::B16_REATTACK_TRACK &&
        mission.state == MissionState::B3_APPROACH)
      {
        post_capture_recovery_active_ = false;
      }
      if (mission.state == MissionState::B13_SEARCH_SCAN &&
        current_state != MissionState::B13_SEARCH_SCAN &&
        current_state != MissionState::B2_TAKEOFF_HOLD)
      {
        startup_search_.reset();
        startup_search_result_ = {};
        startup_search_result_.completed = false;
        startup_search_result_.pending = true;
        startup_search_started_logged_ = false;
      }
    }
    px4_offboard_tracking::msg::MissionStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = common_frame_id_;
    msg.state = static_cast<std::uint8_t>(mission.state);
    msg.state_name = mission.state_name;
    msg.transition_reason = mission.transition_reason;
    msg.state_elapsed = mission.state_elapsed_s;
    msg.estimate_valid = estimate_valid;
    msg.approach_ready = approach_ready;
    msg.alignment_ready = alignment_ready;
    msg.alignment_feasible = last_alignment_feasibility_.feasible;
    msg.quality_gate_passed = quality.passed;
    msg.capture_window_passed = capture.held;
    msg.virtual_capture_published = virtual_capture_published_;
    msg.safety_abort = safety.abort_requested;
    msg.recovery_complete = safety.recovery_complete;
    msg.capture_count = static_cast<std::uint32_t>(virtual_capture_event_id_);
    msg.required_capture_count = static_cast<std::uint32_t>(required_virtual_captures_);
    msg.mission_complete = mission_complete_;
    mission_status_pub_->publish(msg);
    std_msgs::msg::Bool complete_msg;
    complete_msg.data = mission_complete_;
    mission_complete_pub_->publish(complete_msg);
  }

  void publish_safety_status(
    const SafetyManagerResult & safety, double range, double tilt, double speed,
    double altitude_error)
  {
    if (!safety_status_pub_) {return;}
    px4_offboard_tracking::msg::SafetyStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = common_frame_id_;
    msg.abort_requested = safety.abort_requested;
    msg.recovery_complete = safety.recovery_complete;
    msg.local_state_ok = safety.local_state_ok;
    msg.attitude_ok = safety.attitude_ok;
    msg.range_ok = safety.range_ok;
    msg.tilt_ok = safety.tilt_ok;
    msg.speed_ok = safety.speed_ok;
    msg.altitude_ok = safety.altitude_ok;
    msg.recovery_instantaneous_pass = safety.recovery_instantaneous_pass;
    msg.range = range;
    msg.tilt = tilt;
    msg.speed = speed;
    msg.altitude_error = altitude_error;
    msg.recovery_elapsed = safety.recovery_elapsed_s;
    msg.reason = safety.reason;
    safety_status_pub_->publish(msg);
    last_safety_status_ = safety;
  }

  void publish_tracking_quality(
    const TrackingQualityGateResult & quality, double observation_age, double estimate_age)
  {
    if (!tracking_quality_pub_) {return;}
    px4_offboard_tracking::msg::TrackingQuality msg;
    msg.header.stamp = now();
    msg.header.frame_id = common_frame_id_;
    msg.passed = quality.passed;
    msg.estimate_ok = quality.estimate_ok;
    msg.covariance_ok = quality.covariance_ok;
    msg.innovation_ok = quality.innovation_ok;
    msg.visibility_ok = quality.visibility_ok;
    msg.timestamp_ok = quality.timestamp_ok;
    msg.attitude_ok = quality.attitude_ok;
    msg.alignment_ok = quality.alignment_ok;
    msg.maximum_position_variance = maximum_position_variance_;
    msg.innovation_mahalanobis = innovation_mahalanobis_;
    msg.observation_age = observation_age;
    msg.estimate_age = estimate_age;
    tracking_quality_pub_->publish(msg);
    last_tracking_quality_ = quality;
  }

  void publish_capture_window(const CaptureWindowResult & capture, double pixel_error)
  {
    if (!capture_window_pub_) {return;}
    px4_offboard_tracking::msg::CaptureWindowStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = common_frame_id_;
    msg.held = capture.held;
    msg.instantaneous_pass = capture.instantaneous_pass;
    msg.quality_ok = capture.quality_ok;
    msg.pixel_ok = capture.pixel_ok;
    msg.alignment_ok = capture.alignment_ok;
    msg.range_ok = capture.range_ok;
    msg.los_rate_ok = capture.los_rate_ok;
    msg.relative_speed_ok = capture.relative_speed_ok;
    msg.prediction_ok = capture.prediction_ok;
    msg.frame_count_ok = capture.frame_count_ok;
    msg.distinct_observation_frames =
      static_cast<std::uint32_t>(capture.distinct_observation_frames);
    msg.pixel_error = pixel_error;
    msg.alignment_error = last_alignment_.valid ? last_alignment_.alignment_error_rad :
      std::numeric_limits<double>::quiet_NaN();
    msg.range = kinematics_range_;
    msg.los_rate = kinematics_los_rate_;
    msg.relative_speed = kinematics_relative_speed_;
    msg.predicted_miss_distance = kinematics_miss_distance_;
    msg.time_to_closest_approach = kinematics_tcpa_;
    msg.hold_elapsed = capture.hold_elapsed_s;
    capture_window_pub_->publish(msg);
    last_capture_window_ = capture;
  }

  void publish_virtual_capture_event(double pixel_error)
  {
    if (!virtual_capture_event_pub_) {return;}
    px4_offboard_tracking::msg::VirtualCaptureEvent msg;
    msg.header.stamp = now();
    msg.header.frame_id = common_frame_id_;
    msg.event_id = ++virtual_capture_event_id_;
    msg.pursuer_position.x = own_.x;
    msg.pursuer_position.y = own_.y;
    msg.pursuer_position.z = own_.z;
    const auto target = estimated_target_state();
    msg.estimated_target_position.x = target.x;
    msg.estimated_target_position.y = target.y;
    msg.estimated_target_position.z = target.z;
    msg.relative_position.x = relative_estimate_.x;
    msg.relative_position.y = relative_estimate_.y;
    msg.relative_position.z = relative_estimate_.z;
    msg.relative_velocity.x = relative_estimate_.vx;
    msg.relative_velocity.y = relative_estimate_.vy;
    msg.relative_velocity.z = relative_estimate_.vz;
    msg.pixel_u = observation_pixel_u_;
    msg.pixel_v = observation_pixel_v_;
    msg.pixel_error = pixel_error;
    msg.alignment_error = last_alignment_.alignment_error_rad;
    msg.range = kinematics_range_;
    msg.los_rate = kinematics_los_rate_;
    msg.relative_speed = kinematics_relative_speed_;
    msg.predicted_miss_distance = kinematics_miss_distance_;
    msg.time_to_closest_approach = kinematics_tcpa_;
    msg.maximum_position_variance = maximum_position_variance_;
    msg.innovation_mahalanobis = innovation_mahalanobis_;
    msg.capture_hold_time = last_capture_window_.hold_elapsed_s;
    msg.distinct_observation_frames =
      static_cast<std::uint32_t>(last_capture_window_.distinct_observation_frames);
    msg.quality_passed = last_tracking_quality_.passed;
    msg.capture_window_held = last_capture_window_.held;
    virtual_capture_event_pub_->publish(msg);
    last_virtual_capture_time_ = now();
    RCLCPP_INFO(get_logger(), "published virtual capture event %lu",
      static_cast<unsigned long>(msg.event_id));
  }

  void publish_vehicle_command(uint32_t command, float param1, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestamp_us();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = static_cast<uint8_t>(target_system_);
    msg.target_component = 1;
    msg.source_system = 255;
    msg.source_component = 0;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  std::string role_;
  std::string topic_prefix_;
  std::string target_state_topic_;
  std::string common_origin_topic_;
  std::string task_origin_topic_;
  std::string relative_state_topic_;
  std::string relative_estimate_topic_;
  std::string alignment_feasibility_topic_;
  std::string mission_status_topic_;
  std::string observation_topic_;
  std::string tracking_quality_topic_;
  std::string relative_kinematics_topic_;
  std::string capture_window_topic_;
  std::string virtual_capture_event_topic_;
  std::string safety_status_topic_;
  std::string mission_complete_topic_;
  std::string common_frame_id_;
  std::string control_level_;
  int target_system_;
  std::string target_scenario_name_;
  double takeoff_height_;
  double target_radius_;
  double target_speed_;
  double target_heading_;
  double target_path_length_;
  double target_path_width_;
  bool target_trajectory_feedforward_enabled_;
  double target_motion_delay_;
  double target_motion_ramp_s_;
  double target_search_enable_after_motion_s_;
  bool target_wait_for_pursuer_intercept_;
  std::string target_motion_ready_topic_;
  bool target_wait_for_pursuer_takeoff_ready_;
  std::string pursuer_takeoff_ready_topic_;
  std::string pure_interceptor_status_topic_;
  double attack_enable_delay_;
  double target_stable_motion_time_s_;
  TargetTrajectory target_trajectory_;
  double state_timeout_;
  double status_timeout_;
  double estimate_prediction_coast_s_;
  double estimate_measurement_hold_s_;
  double offboard_rate_hz_;
  double offboard_warmup_s_;
  double preflight_stability_hold_s_;
  double altitude_hold_position_gain_;
  double altitude_hold_velocity_gain_;
  double altitude_hold_max_speed_;
  double altitude_hold_max_acceleration_;
  TrackingController controller_;
  AccelerationGuidance guidance_;
  std::string guidance_controller_;
  ApproachCorridor approach_corridor_;
  MpcGuidance mpc_guidance_;
  TerminalGuidance terminal_guidance_;
  double mpc_vertical_absolute_hold_blend_;
  double mpc_desired_vertical_separation_m_;
  double mpc_3d_entry_vertical_tolerance_m_;
  double mpc_3d_entry_vertical_speed_tolerance_mps_;
  double mpc_3d_entry_hold_time_s_;
  double mpc_3d_entry_ready_since_s_{std::numeric_limits<double>::quiet_NaN()};
  double target_flight_ready_time_s_{std::numeric_limits<double>::quiet_NaN()};
  double target_motion_gate_time_s_{std::numeric_limits<double>::quiet_NaN()};
  bool target_motion_gate_open_{false};
  bool target_motion_ready_published_{false};
  bool pursuer_takeoff_ready_{false};
  double pursuer_takeoff_ready_time_s_{std::numeric_limits<double>::quiet_NaN()};
  double target_motion_stable_since_s_{std::numeric_limits<double>::quiet_NaN()};
  std::string pursuer_yaw_mode_;
  double pursuer_fixed_yaw_rad_;
  double pursuer_yaw_minimum_speed_;
  double last_motion_yaw_rad_{};
  double invalid_estimate_velocity_damping_;
  double invalid_estimate_maximum_acceleration_;
  HorizontalSpeedLimiter horizontal_speed_limiter_;
  StartupSearch startup_search_;
  BearingCameraModel search_camera_model_;
  double search_handover_alignment_blend_;
  double search_handover_maximum_tilt_rad_;
  double depth_confirmation_hold_time_s_;
  int depth_confirmation_minimum_frames_;
  StartupSearchResult startup_search_result_{};
  BearingObservation search_observation_{};
  std::array<double, 3> search_los_ned_{};
  bool search_los_ned_valid_{false};
  double depth_confirmation_started_s_{std::numeric_limits<double>::quiet_NaN()};
  int depth_confirmation_frames_{0};
  rclcpp::Time last_depth_confirmation_stamp_{0, 0, RCL_ROS_TIME};
  bool startup_search_started_logged_{false};
  GeometricControl geometric_control_;
  ZAxisAlignment z_axis_alignment_;
  AlignmentFeasibility alignment_feasibility_;
  double feasibility_maximum_available_thrust_;
  double aim_entry_horizontal_distance_m_;
  double aim_exit_horizontal_distance_m_;
  double capture_minimum_range_m_;
  double capture_maximum_range_m_;
  double midcourse_alignment_blend_;
  double terminal_alignment_blend_;
  double terminal_capture_alignment_blend_;
  double terminal_alignment_weight_slew_rate_;
  double terminal_alignment_weight_{};
  double collision_exit_range_m_;
  double alignment_ready_error_rad_;
  double takeoff_ready_tolerance_m_;
  double altitude_correction_threshold_m_;
  double mission_minimum_target_above_m_;
  int required_virtual_captures_;
  double reattack_cooldown_s_;
  double post_capture_recoil_duration_s_;
  double post_capture_recoil_acceleration_mps2_;
  TrackingQualityGate tracking_quality_gate_;
  CaptureWindow capture_window_;
  double capture_camera_cx_;
  double capture_camera_cy_;
  SafetyManager safety_manager_;
  AttitudeSetpointGenerator attitude_setpoint_generator_;
  BodyRateController body_rate_controller_;
  MissionStateMachine mission_state_machine_;
  State3d local_{};
  State3d local_origin_{};
  State3d latest_global_local_position_{};
  State3d local_continuous_{};
  State3d common_local_anchor_{};
  State3d common_frame_anchor_{};
  State3d own_{};
  State3d target_truth_{};
  State3d relative_estimate_{};
  std::array<double, 3> relative_acceleration_{};
  GeodeticPosition latest_global_position_{};
  std::unique_ptr<CommonFrameTransform> common_transform_;
  double latest_global_eph_{};
  double latest_global_epv_{};
  bool local_valid_{false};
  bool status_valid_{false};
  bool local_origin_set_{false};
  bool local_origin_locked_{false};
  bool global_position_received_{false};
  bool latest_global_local_valid_{false};
  bool common_position_valid_{false};
  bool common_local_anchor_set_{false};
  bool local_reset_counters_set_{false};
  std::uint8_t local_xy_reset_counter_{0};
  std::uint8_t local_z_reset_counter_{0};
  double local_continuity_offset_x_{0.0};
  double local_continuity_offset_y_{0.0};
  double local_continuity_offset_z_{0.0};
  bool target_truth_valid_{false};
  bool relative_estimate_valid_{false};
  bool relative_estimate_measurement_valid_{false};
  bool attitude_valid_{false};
  bool angular_velocity_valid_{false};
  bool observation_valid_{false};
  bool observation_visible_{false};
  bool relative_kinematics_valid_{false};
  bool virtual_capture_published_{false};
  bool post_capture_recovery_active_{false};
  bool mission_complete_{false};
  std::size_t warmup_count_{0};
  std::size_t warmup_setpoint_count_{0};
  px4_msgs::msg::VehicleStatus status_{};
  Quaternion attitude_{};
  double body_rate_rad_s_{};
  double observation_pixel_u_{};
  double observation_pixel_v_{};
  double observation_depth_m_{std::numeric_limits<double>::quiet_NaN()};
  double maximum_position_variance_{std::numeric_limits<double>::infinity()};
  double innovation_mahalanobis_{std::numeric_limits<double>::infinity()};
  double kinematics_range_{std::numeric_limits<double>::quiet_NaN()};
  double kinematics_relative_speed_{std::numeric_limits<double>::quiet_NaN()};
  double kinematics_los_rate_{std::numeric_limits<double>::quiet_NaN()};
  double kinematics_tcpa_{std::numeric_limits<double>::quiet_NaN()};
  double kinematics_miss_distance_{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t virtual_capture_event_id_{0};
  AlignmentFeasibilityResult last_alignment_feasibility_{};
  ZAxisAlignmentResult last_alignment_{};
  TrackingQualityGateResult last_tracking_quality_{};
  CaptureWindowResult last_capture_window_{};
  SafetyManagerResult last_safety_status_{};
  rclcpp::Time local_update_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point local_steady_update_{};
  rclcpp::Time status_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time preflight_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time common_position_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_truth_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time relative_estimate_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_estimate_measurement_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time attitude_update_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point attitude_steady_update_{};
  rclcpp::Time angular_velocity_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time alignment_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time observation_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time relative_kinematics_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_virtual_capture_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  double local_sample_time_s_{std::numeric_limits<double>::quiet_NaN()};
  double target_motion_last_px4_sample_time_s_{
    std::numeric_limits<double>::quiet_NaN()};
  double target_trajectory_elapsed_s_{0.0};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_motion_ready_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_state_sub_;
  rclcpp::Subscription<px4_offboard_tracking::msg::PureInterceptorStatus>::SharedPtr
    pure_interceptor_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pursuer_takeoff_ready_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr common_origin_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr common_origin_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr task_origin_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr relative_state_pub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::AlignmentFeasibility>::SharedPtr
    alignment_feasibility_pub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::MissionStatus>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::TrackingQuality>::SharedPtr
    tracking_quality_pub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::CaptureWindowStatus>::SharedPtr
    capture_window_pub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::VirtualCaptureEvent>::SharedPtr
    virtual_capture_event_pub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::SafetyStatus>::SharedPtr safety_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_pub_;
  rclcpp::Subscription<px4_offboard_tracking::msg::RelativeEstimate>::SharedPtr
    relative_estimate_sub_;
  rclcpp::Subscription<px4_offboard_tracking::msg::TargetObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<px4_offboard_tracking::msg::RelativeKinematics>::SharedPtr
    relative_kinematics_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::OffboardVehicle>());
  rclcpp::shutdown();
  return 0;
}
