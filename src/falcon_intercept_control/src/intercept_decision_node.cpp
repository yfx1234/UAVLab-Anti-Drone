#include "falcon_intercept_control/control_core.hpp"

#include <falcon_intercept_msgs/msg/guidance_command.hpp>
#include <falcon_intercept_msgs/msg/target_camera_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>

namespace falcon_intercept_control
{
namespace
{

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

struct DetectionCue
{
  double stamp_s{};
  double horizontal_error_px{};
  double vertical_error_px{};
  double confidence{};
  bool valid{false};
};

std::optional<double> match_number(const std::string & text, const std::regex & expression)
{
  std::smatch match;
  if (!std::regex_search(text, match, expression) || match.size() < 2) {return std::nullopt;}
  try {
    const double value = std::stod(match[1].str());
    return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

DetectionCue parse_detection_cue(
  const std::string & json, double width, double height, double minimum_confidence)
{
  static const std::regex sec_regex("\\\"sec\\\"\\s*:\\s*(-?[0-9]+)");
  static const std::regex nanosec_regex("\\\"nanosec\\\"\\s*:\\s*([0-9]+)");
  static const std::regex confidence_regex(
    "\\\"confidence\\\"\\s*:\\s*([-+0-9.eE]+)");
  static const std::regex center_regex(
    "\\\"bbox_center\\\"\\s*:\\s*\\[\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*\\]");
  DetectionCue output;
  const auto sec = match_number(json, sec_regex);
  const auto nanosec = match_number(json, nanosec_regex);
  const auto confidence = match_number(json, confidence_regex);
  std::smatch center_match;
  if (!sec || !nanosec || !confidence || *confidence < minimum_confidence ||
    !std::regex_search(json, center_match, center_regex) || center_match.size() < 3)
  {
    return output;
  }
  try {
    const double center_x = std::stod(center_match[1].str());
    const double center_y = std::stod(center_match[2].str());
    output.stamp_s = *sec + *nanosec * 1e-9;
    output.horizontal_error_px = center_x - width * 0.5;
    output.vertical_error_px = center_y - height * 0.5;
    output.confidence = *confidence;
    output.valid = std::isfinite(output.stamp_s) && std::isfinite(center_x) &&
      std::isfinite(center_y);
  } catch (const std::exception &) {
    return {};
  }
  return output;
}

bool parse_health_valid(const std::string & json, double target_stamp_s, double tolerance_s)
{
  static const std::regex valid_regex("\\\"valid\\\"\\s*:\\s*true");
  static const std::regex stamp_regex(
    "\\\"last_measurement_timestamp_s\\\"\\s*:\\s*([-+0-9.eE]+)");
  if (!std::regex_search(json, valid_regex)) {return false;}
  const auto stamp = match_number(json, stamp_regex);
  return stamp && std::abs(*stamp - target_stamp_s) <= tolerance_s;
}

double maximum_diagonal(const std::array<double, 9> & covariance)
{
  return std::max({covariance[0], covariance[4], covariance[8]});
}

}  // namespace

class InterceptDecisionNode final : public rclcpp::Node
{
public:
  InterceptDecisionNode()
  : Node("intercept_decision_node"), core_(load_control_config())
  {
    target_topic_ = declare_parameter<std::string>(
      "target_topic", "/perception/target/camera_position");
    health_topic_ = declare_parameter<std::string>("health_topic", "/target/state_health");
    search_topic_ = declare_parameter<std::string>(
      "search_detections_topic", "/search/detections_json");
    front_topic_ = declare_parameter<std::string>(
      "front_detections_topic", "/stereo/front/detections_json");
    back_topic_ = declare_parameter<std::string>(
      "back_detections_topic", "/stereo/back/detections_json");
    command_topic_ = declare_parameter<std::string>(
      "guidance_command_topic", "/intercept/guidance_command");
    state_topic_ = declare_parameter<std::string>("state_topic", "/intercept/state");
    fire_topic_ = declare_parameter<std::string>("fire_event_topic", "/intercept/fire_event");
    calibration_ready_ = declare_parameter<bool>("calibration_ready", false);
    require_health_ = declare_parameter<bool>("require_target_health", true);
    health_tolerance_s_ = declare_parameter<double>("health_stamp_tolerance_s", 0.05);
    detection_timeout_s_ = declare_parameter<double>("detection_timeout_s", 0.5);
    detection_confidence_ = declare_parameter<double>("detection_confidence", 0.50);
    search_width_ = declare_parameter<double>("search_image_width", 2560.0);
    search_height_ = declare_parameter<double>("search_image_height", 1440.0);
    stereo_width_ = declare_parameter<double>("stereo_image_width", 3840.0);
    stereo_height_ = declare_parameter<double>("stereo_image_height", 2160.0);
    center_threshold_px_ = declare_parameter<double>("center_threshold_px", 80.0);
    command_frame_id_ = declare_parameter<std::string>(
      "command_frame_id", "pursuer_base_flu");
    mission_enabled_ = declare_parameter<bool>("start_enabled", false);
    velocity_filter_alpha_ = std::clamp(
      declare_parameter<double>("velocity_filter_alpha", 0.35), 0.0, 1.0);

    transform_.camera_translation_body_frd_m = {
      declare_parameter<double>("camera_translation_body_frd_x_m", 0.0),
      declare_parameter<double>("camera_translation_body_frd_y_m", 0.0),
      declare_parameter<double>("camera_translation_body_frd_z_m", 0.0)};
    transform_.q_body_to_optical = {
      declare_parameter<double>("camera_q_body_to_optical_w", 1.0),
      declare_parameter<double>("camera_q_body_to_optical_x", 0.0),
      declare_parameter<double>("camera_q_body_to_optical_y", 0.0),
      declare_parameter<double>("camera_q_body_to_optical_z", 0.0)};

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    target_sub_ = create_subscription<falcon_intercept_msgs::msg::TargetCameraPosition>(
      target_topic_, sensor_qos,
      [this](falcon_intercept_msgs::msg::TargetCameraPosition::ConstSharedPtr message) {
        on_target(*message);
      });
    health_sub_ = create_subscription<std_msgs::msg::String>(
      health_topic_, 10, [this](std_msgs::msg::String::ConstSharedPtr message) {
        health_json_ = message->data;
      });
    search_sub_ = create_subscription<std_msgs::msg::String>(
      search_topic_, sensor_qos, [this](std_msgs::msg::String::ConstSharedPtr message) {
        search_cue_ = parse_detection_cue(
          message->data, search_width_, search_height_, detection_confidence_);
      });
    front_sub_ = create_subscription<std_msgs::msg::String>(
      front_topic_, sensor_qos, [this](std_msgs::msg::String::ConstSharedPtr message) {
        front_cue_ = parse_detection_cue(
          message->data, stereo_width_, stereo_height_, detection_confidence_);
      });
    back_sub_ = create_subscription<std_msgs::msg::String>(
      back_topic_, sensor_qos, [this](std_msgs::msg::String::ConstSharedPtr message) {
        back_cue_ = parse_detection_cue(
          message->data, stereo_width_, stereo_height_, detection_confidence_);
      });
    mission_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/intercept/mission_enable", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {mission_enabled_ = message->data;});
    abort_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/intercept/abort", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data) {abort_requested_ = true;}
      });
    landed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/vehicle/landed", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {landed_ = message->data;});

    command_pub_ = create_publisher<falcon_intercept_msgs::msg::GuidanceCommand>(
      command_topic_, rclcpp::QoS(1).reliable());
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    fire_pub_ = create_publisher<std_msgs::msg::String>(
      fire_topic_, rclcpp::QoS(10).reliable().transient_local());

    const double rate_hz = std::max(20.0, declare_parameter<double>("rate_hz", 20.0));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {on_timer();});
    RCLCPP_WARN(
      get_logger(),
      "Jetson decision ready calibration_ready=%s start_enabled=%s; live authority remains adapter-gated",
      calibration_ready_ ? "true" : "false", mission_enabled_ ? "true" : "false");
  }

private:
  ControlConfig load_control_config()
  {
    ControlConfig config;
    config.target_timeout_s = declare_parameter<double>("target_timeout_s", 0.5);
    config.maximum_position_variance_m2 = declare_parameter<double>(
      "maximum_position_variance_m2", 4.0);
    config.minimum_confidence = declare_parameter<double>("minimum_confidence", 0.50);
    config.fire_distance_m = declare_parameter<double>("fire_distance_m", 2.0);
    config.minimum_safe_range_m = declare_parameter<double>("minimum_safe_range_m", 1.5);
    config.center_threshold_px = declare_parameter<double>("fire_center_threshold_px", 80.0);
    config.fire_hold_frames = declare_parameter<int>("fire_hold_frames", 3);
    config.track_confirm_frames = declare_parameter<int>("track_confirm_frames", 2);
    config.land_hold_s = declare_parameter<double>("land_hold_s", 1.0);
    config.max_task_time_s = declare_parameter<double>("max_task_time_s", 120.0);
    config.position_gain_xy = declare_parameter<double>("position_gain_xy", 0.4);
    config.position_gain_z = declare_parameter<double>("position_gain_z", 0.3);
    config.maximum_speed_xy_mps = declare_parameter<double>("maximum_speed_xy_mps", 0.8);
    config.maximum_speed_z_mps = declare_parameter<double>("maximum_speed_z_mps", 0.5);
    config.search_yaw_gain = declare_parameter<double>("search_yaw_gain", 0.004);
    config.maximum_search_yaw_rate_rps = declare_parameter<double>(
      "maximum_search_yaw_rate_rps", 0.35);
    config.scan_yaw_rate_rps = declare_parameter<double>("scan_yaw_rate_rps", 0.20);
    config.startup_search_enabled = declare_parameter<bool>("startup_search_enabled", false);
    config.startup_search_yaw_rate_rps = declare_parameter<double>(
      "startup_search_yaw_rate_rps", 0.20);
    config.startup_search_turns = declare_parameter<double>("startup_search_turns", 1.0);
    config.startup_search_settle_time_s = declare_parameter<double>(
      "startup_search_settle_time_s", 0.5);
    return config;
  }

  double now_seconds() {return get_clock()->now().seconds();}

  void on_target(const falcon_intercept_msgs::msg::TargetCameraPosition & message)
  {
    const double source_stamp_s = stamp_seconds(message.header.stamp);
    if (has_target_ && message.sequence <= target_sequence_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected non-increasing target sequence current=%u received=%u",
        target_sequence_, message.sequence);
      return;
    }
    target_sequence_ = message.sequence;
    has_target_ = true;
    target_stamp_s_ = source_stamp_s;
    target_confidence_ = message.confidence;
    target_valid_ = message.valid && message.position_camera.z > 0.0 && source_stamp_s > 0.0;
    const std::array<double, 3> optical_position{
      message.position_camera.x, message.position_camera.y, message.position_camera.z};
    std::array<double, 9> optical_covariance{};
    std::copy(
      message.position_covariance.begin(), message.position_covariance.end(),
      optical_covariance.begin());
    try {
      const auto body_position = optical_position_to_body_frd(optical_position, transform_);
      const auto body_covariance = optical_covariance_to_body_frd(
        optical_covariance, transform_);
      if (target_valid_ && previous_target_valid_ && source_stamp_s > previous_target_stamp_s_) {
        const double dt = source_stamp_s - previous_target_stamp_s_;
        if (dt <= 1.0) {
          for (int axis = 0; axis < 3; ++axis) {
            const double measured = (body_position[axis] - previous_target_position_[axis]) / dt;
            target_velocity_[axis] = velocity_filter_alpha_ * measured +
              (1.0 - velocity_filter_alpha_) * target_velocity_[axis];
          }
        } else {
          target_velocity_ = {};
        }
      }
      target_position_ = body_position;
      target_maximum_variance_ = maximum_diagonal(body_covariance);
      previous_target_position_ = body_position;
      previous_target_stamp_s_ = source_stamp_s;
      previous_target_valid_ = target_valid_;
    } catch (const std::exception & error) {
      target_valid_ = false;
      previous_target_valid_ = false;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Target transform rejected: %s", error.what());
    }
  }

  bool cue_fresh_and_centered(const DetectionCue & cue, double reference_s) const
  {
    const double age = reference_s - cue.stamp_s;
    return cue.valid && age >= 0.0 && age <= detection_timeout_s_ &&
      std::abs(cue.horizontal_error_px) <= center_threshold_px_ &&
      std::abs(cue.vertical_error_px) <= center_threshold_px_;
  }

  void on_timer()
  {
    const double now_s = now_seconds();
    const double target_age_s = has_target_ ? now_s - target_stamp_s_ :
      std::numeric_limits<double>::infinity();
    const bool health_valid = !require_health_ ||
      parse_health_valid(health_json_, target_stamp_s_, health_tolerance_s_);
    const bool search_fresh = search_cue_.valid && now_s >= search_cue_.stamp_s &&
      now_s - search_cue_.stamp_s <= detection_timeout_s_;

    ControlInput input;
    input.now_s = now_s;
    input.mission_enabled = mission_enabled_;
    input.abort_requested = abort_requested_;
    input.landed = landed_;
    input.target_valid = calibration_ready_ && target_valid_;
    input.target_position_body_frd_m = target_position_;
    input.target_velocity_body_frd_mps = target_velocity_;
    input.maximum_position_variance_m2 = target_maximum_variance_;
    input.confidence = target_confidence_;
    input.target_age_s = target_age_s;
    input.health_valid = health_valid;
    input.left_centered = cue_fresh_and_centered(front_cue_, target_stamp_s_);
    input.right_centered = cue_fresh_and_centered(back_cue_, target_stamp_s_);
    input.search_cue_valid = search_fresh;
    input.search_horizontal_error_px = search_cue_.horizontal_error_px;

    const auto output = core_.update(input);
    falcon_intercept_msgs::msg::GuidanceCommand command;
    command.header.stamp = get_clock()->now();
    command.header.frame_id = command_frame_id_;
    command.sequence = command_sequence_++;
    command.mission_state = static_cast<std::uint8_t>(output.state);
    command.velocity_body_flu.x = output.velocity_body_frd_mps[0];
    command.velocity_body_flu.y = -output.velocity_body_frd_mps[1];
    command.velocity_body_flu.z = -output.velocity_body_frd_mps[2];
    command.yaw_rate_flu = -static_cast<float>(output.yaw_rate_rps);
    command.valid_for_s = 0.25f;
    command.valid = output.valid;
    command.land_request = output.land_request;
    command.reason = output.reason;
    command_pub_->publish(command);

    std::ostringstream state;
    state << std::fixed << std::setprecision(6)
          << "{\"state\":\"" << ControlCore::state_name(output.state)
          << "\",\"reason\":\"" << output.reason
          << "\",\"timestamp\":" << now_s
          << ",\"target_valid\":" << (input.target_valid ? "true" : "false")
          << ",\"health_valid\":" << (health_valid ? "true" : "false")
          << ",\"calibration_ready\":" << (calibration_ready_ ? "true" : "false")
          << ",\"range\":";
    if (output.kinematics.valid) {state << output.kinematics.range_m;} else {state << "null";}
    state << '}';
    std_msgs::msg::String state_message;
    state_message.data = state.str();
    state_pub_->publish(state_message);

    const std::string state_name = ControlCore::state_name(output.state);
    if (state_name != last_state_name_) {
      RCLCPP_INFO(get_logger(), "FSM_STATE %s reason=%s", state_name.c_str(), output.reason.c_str());
      last_state_name_ = state_name;
    }
    if (output.fire_event) {
      std_msgs::msg::String event;
      event.data = "{\"event\":\"FIRE\",\"sequence\":" +
        std::to_string(command.sequence) + ",\"timestamp\":" + std::to_string(now_s) + "}";
      fire_pub_->publish(event);
      RCLCPP_WARN(get_logger(), "FIRE_EVENT %s", event.data.c_str());
    }
  }

  ControlCore core_;
  RigidTransform transform_;
  std::string target_topic_;
  std::string health_topic_;
  std::string search_topic_;
  std::string front_topic_;
  std::string back_topic_;
  std::string command_topic_;
  std::string state_topic_;
  std::string fire_topic_;
  std::string command_frame_id_;
  bool calibration_ready_{false};
  bool require_health_{true};
  bool mission_enabled_{false};
  bool abort_requested_{false};
  bool landed_{false};
  bool has_target_{false};
  bool target_valid_{false};
  bool previous_target_valid_{false};
  double health_tolerance_s_{0.05};
  double detection_timeout_s_{0.5};
  double detection_confidence_{0.50};
  double search_width_{2560.0};
  double search_height_{1440.0};
  double stereo_width_{3840.0};
  double stereo_height_{2160.0};
  double center_threshold_px_{80.0};
  double velocity_filter_alpha_{0.35};
  double target_stamp_s_{};
  double previous_target_stamp_s_{};
  double target_confidence_{};
  double target_maximum_variance_{std::numeric_limits<double>::infinity()};
  std::uint32_t target_sequence_{};
  std::uint32_t command_sequence_{};
  std::array<double, 3> target_position_{};
  std::array<double, 3> previous_target_position_{};
  std::array<double, 3> target_velocity_{};
  std::string health_json_;
  std::string last_state_name_;
  DetectionCue search_cue_;
  DetectionCue front_cue_;
  DetectionCue back_cue_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::TargetCameraPosition>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr front_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr back_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr abort_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landed_sub_;
  rclcpp::Publisher<falcon_intercept_msgs::msg::GuidanceCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fire_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace falcon_intercept_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<falcon_intercept_control::InterceptDecisionNode>());
  rclcpp::shutdown();
  return 0;
}
