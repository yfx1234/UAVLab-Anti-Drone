#include "falcon_intercept_control/real_flight_core.hpp"

#include <falcon_intercept_msgs/msg/command_proposal.hpp>
#include <falcon_intercept_msgs/msg/mission_status.hpp>
#include <falcon_intercept_msgs/msg/target_track.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace falcon_intercept_control
{
class GuidanceNode final : public rclcpp::Node
{
public:
  GuidanceNode()
  : Node("guidance_node"), controller_(GuidanceConfig{
      declare_parameter<double>("position_gain_forward", 0.25),
      declare_parameter<double>("position_gain_lateral", 0.30),
      declare_parameter<double>("position_gain_vertical", 0.25),
      declare_parameter<double>("velocity_feedforward", 0.35),
      declare_parameter<double>("desired_distance_m", 1.7),
      declare_parameter<double>("maximum_forward_speed_mps", 0.30),
      declare_parameter<double>("maximum_lateral_speed_mps", 0.15),
      declare_parameter<double>("maximum_vertical_speed_mps", 0.10),
      declare_parameter<double>("maximum_yaw_rate_rps", 0.20),
      declare_parameter<double>("yaw_gain", 1.2),
      declare_parameter<double>("maximum_acceleration_mps2", 0.30)})
  {
    transform_.camera_translation_body_flu_m = {
      declare_parameter<double>("camera_translation_body_flu_x_m", 0.0),
      declare_parameter<double>("camera_translation_body_flu_y_m", 0.0),
      declare_parameter<double>("camera_translation_body_flu_z_m", 0.0)};
    transform_.q_body_flu_to_optical = {
      declare_parameter<double>("camera_q_body_flu_to_optical_w", 0.5),
      declare_parameter<double>("camera_q_body_flu_to_optical_x", 0.5),
      declare_parameter<double>("camera_q_body_flu_to_optical_y", -0.5),
      declare_parameter<double>("camera_q_body_flu_to_optical_z", 0.5)};
    track_sub_ = create_subscription<falcon_intercept_msgs::msg::TargetTrack>(
      "/target/track", rclcpp::SensorDataQoS().keep_last(1),
      [this](falcon_intercept_msgs::msg::TargetTrack::ConstSharedPtr message) {
        track_ = *message; has_track_ = true; track_received_s_ = get_clock()->now().seconds();
      });
    mission_sub_ = create_subscription<falcon_intercept_msgs::msg::MissionStatus>(
      "/decision/mission_status", 10,
      [this](falcon_intercept_msgs::msg::MissionStatus::ConstSharedPtr message) {
        mission_ = *message; has_mission_ = true; mission_received_s_ = get_clock()->now().seconds();
      });
    publisher_ = create_publisher<falcon_intercept_msgs::msg::CommandProposal>(
      "/decision/guidance_proposal", 10);
    last_tick_s_ = get_clock()->now().seconds();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  void tick()
  {
    const double current = get_clock()->now().seconds();
    const double dt = std::clamp(current - last_tick_s_, 0.001, 0.2);
    last_tick_s_ = current;
    falcon_intercept_msgs::msg::CommandProposal message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "pursuer_base_flu";
    message.sequence = sequence_++;
    message.reason = "guidance unavailable";
    const double track_age = has_track_ ? std::max({static_cast<double>(track_.age_s),
      current - track_received_s_, current - rclcpp::Time(track_.header.stamp).seconds()}) : INFINITY;
    if (!has_track_ || !has_mission_ || current - mission_received_s_ > 0.5 || track_age > 0.5) {
      controller_.reset(); publisher_->publish(message); return;
    }
    const bool track_mode = mission_.state == mission_.STATE_TRACK;
    const bool approach_mode = mission_.state == mission_.STATE_APPROACH;
    if ((!track_mode && !approach_mode) || !track_.valid) {
      controller_.reset();
      message.mission_state = mission_.state;
      publisher_->publish(message);
      return;
    }
    try {
      const Vector3 optical_position{
        track_.position_camera.x, track_.position_camera.y, track_.position_camera.z};
      const Vector3 optical_velocity{
        track_.velocity_camera.x, track_.velocity_camera.y, track_.velocity_camera.z};
      const auto position = optical_to_body_flu(optical_position, transform_);
      auto velocity_transform = transform_;
      velocity_transform.camera_translation_body_flu_m = {};
      const auto velocity = optical_to_body_flu(optical_velocity, velocity_transform);
      const auto result = controller_.update(position, velocity, dt, approach_mode);
      message.mission_state = mission_.state;
      message.velocity_body_flu.x = result.velocity_body_flu_mps[0];
      message.velocity_body_flu.y = result.velocity_body_flu_mps[1];
      message.velocity_body_flu.z = result.velocity_body_flu_mps[2];
      message.yaw_rate_flu = static_cast<float>(result.yaw_rate_flu_rps);
      message.valid = result.valid;
      message.reason = approach_mode ? "approach guidance" : "track yaw alignment";
    } catch (const std::exception & error) {
      message.reason = error.what();
    }
    publisher_->publish(message);
  }

  GuidanceController controller_;
  CameraToFluTransform transform_;
  bool has_track_{false};
  bool has_mission_{false};
  double last_tick_s_{};
  double track_received_s_{}, mission_received_s_{};
  std::uint32_t sequence_{};
  falcon_intercept_msgs::msg::TargetTrack track_;
  falcon_intercept_msgs::msg::MissionStatus mission_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::TargetTrack>::SharedPtr track_sub_;
  rclcpp::Subscription<falcon_intercept_msgs::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Publisher<falcon_intercept_msgs::msg::CommandProposal>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace falcon_intercept_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<falcon_intercept_control::GuidanceNode>());
  rclcpp::shutdown();
  return 0;
}
