#include "falcon_intercept_control/real_flight_core.hpp"

#include <falcon_intercept_msgs/msg/mission_status.hpp>
#include <falcon_intercept_msgs/msg/search_command.hpp>
#include <falcon_intercept_msgs/msg/vehicle_interface_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <regex>
#include <string>

namespace falcon_intercept_control
{
namespace
{
double now_s(rclcpp::Node & node) {return node.get_clock()->now().seconds();}

std::optional<double> number(const std::string & text, const std::regex & expression)
{
  std::smatch match;
  if (!std::regex_search(text, match, expression) || match.size() < 2) {return std::nullopt;}
  try {
    const double value = std::stod(match[1].str());
    return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
  } catch (...) {return std::nullopt;}
}
}  // namespace

class SearchControllerNode final : public rclcpp::Node
{
public:
  SearchControllerNode()
  : Node("search_controller_node"), controller_(SearchConfig{
      declare_parameter<double>("scan_yaw_rate_flu_rps", 0.20),
      declare_parameter<double>("settle_time_s", 0.5),
      declare_parameter<double>("search_timeout_s", 120.0),
      declare_parameter<double>("cue_gain", 0.004),
      declare_parameter<double>("maximum_cue_yaw_rate_rps", 0.20)})
  {
    image_width_ = declare_parameter<double>("search_image_width", 2560.0);
    minimum_confidence_ = declare_parameter<double>("minimum_confidence", 0.50);
    cue_timeout_s_ = declare_parameter<double>("cue_timeout_s", 0.5);
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    detection_sub_ = create_subscription<std_msgs::msg::String>(
      "/search/detections_json", sensor_qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_detection(message->data);});
    vehicle_sub_ = create_subscription<falcon_intercept_msgs::msg::VehicleInterfaceStatus>(
      "/control/px4_status", 10,
      [this](falcon_intercept_msgs::msg::VehicleInterfaceStatus::ConstSharedPtr message) {
        vehicle_ = *message; has_vehicle_ = true; vehicle_received_s_ = now_s(*this);
      });
    mission_sub_ = create_subscription<falcon_intercept_msgs::msg::MissionStatus>(
      "/decision/mission_status", 10,
      [this](falcon_intercept_msgs::msg::MissionStatus::ConstSharedPtr message) {
        const bool was_search = mission_state_ == message->STATE_SEARCH;
        mission_state_ = message->state;
        if (was_search && mission_state_ != message->STATE_SEARCH) {controller_.reset();}
      });
    publisher_ = create_publisher<falcon_intercept_msgs::msg::SearchCommand>(
      "/decision/search_command", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  void on_detection(const std::string & json)
  {
    static const std::regex sec_regex("\\\"sec\\\"\\s*:\\s*(-?[0-9]+)");
    static const std::regex nanosec_regex("\\\"nanosec\\\"\\s*:\\s*([0-9]+)");
    static const std::regex confidence_regex("\\\"confidence\\\"\\s*:\\s*([-+0-9.eE]+)");
    static const std::regex center_regex(
      "\\\"bbox_center\\\"\\s*:\\s*\\[\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*\\]");
    const auto sec = number(json, sec_regex);
    const auto nanosec = number(json, nanosec_regex);
    const auto confidence = number(json, confidence_regex);
    std::smatch center;
    cue_valid_ = false;
    if (!sec || !nanosec || !confidence || *confidence < minimum_confidence_ ||
      !std::regex_search(json, center, center_regex) || center.size() < 3) {return;}
    try {
      cue_stamp_s_ = *sec + *nanosec * 1e-9;
      cue_horizontal_error_px_ = std::stod(center[1].str()) - image_width_ * 0.5;
      cue_valid_ = std::isfinite(cue_stamp_s_) && std::isfinite(cue_horizontal_error_px_);
    } catch (...) {cue_valid_ = false;}
  }

  void tick()
  {
    const double current = now_s(*this);
    SearchInput input;
    input.now_s = current;
    input.enabled = mission_state_ == falcon_intercept_msgs::msg::MissionStatus::STATE_SEARCH;
    input.vehicle_ready = has_vehicle_ && current - vehicle_received_s_ <= 0.5 && vehicle_.ready_for_mission;
    input.yaw_ned_rad = has_vehicle_ ? vehicle_.yaw_ned : 0.0;
    input.cue_valid = cue_valid_ && current >= cue_stamp_s_ && current - cue_stamp_s_ <= cue_timeout_s_;
    input.cue_horizontal_error_px = cue_horizontal_error_px_;
    const auto result = controller_.update(input);
    falcon_intercept_msgs::msg::SearchCommand message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "pursuer_base_flu";
    message.sequence = sequence_++;
    message.yaw_rate_flu = static_cast<float>(result.yaw_rate_flu_rps);
    message.progress = static_cast<float>(result.progress);
    message.completed_turns = result.completed_turns;
    message.active = result.active;
    message.settling = result.settling;
    message.target_cued = result.target_cued;
    message.timed_out = result.timed_out;
    message.valid = result.valid;
    message.reason = result.reason;
    publisher_->publish(message);
  }

  SearchController controller_;
  double image_width_{};
  double minimum_confidence_{};
  double cue_timeout_s_{};
  bool cue_valid_{false};
  double cue_stamp_s_{};
  double vehicle_received_s_{};
  double cue_horizontal_error_px_{};
  bool has_vehicle_{false};
  std::uint8_t mission_state_{falcon_intercept_msgs::msg::MissionStatus::STATE_INIT};
  std::uint32_t sequence_{};
  falcon_intercept_msgs::msg::VehicleInterfaceStatus vehicle_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::VehicleInterfaceStatus>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Publisher<falcon_intercept_msgs::msg::SearchCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace falcon_intercept_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<falcon_intercept_control::SearchControllerNode>());
  rclcpp::shutdown();
  return 0;
}
