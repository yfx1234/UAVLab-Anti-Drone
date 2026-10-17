#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "px4_offboard_tracking/msg/relative_estimate.hpp"
#include "px4_offboard_tracking/msg/relative_kinematics.hpp"
#include "px4_offboard_tracking/relative_kinematics.hpp"

namespace px4_offboard_tracking
{

class RelativeKinematicsNode : public rclcpp::Node
{
public:
  RelativeKinematicsNode()
  : Node("relative_kinematics"),
    estimate_topic_(
      declare_parameter<std::string>("estimate_topic", "/tracking/relative/estimate")),
    kinematics_topic_(
      declare_parameter<std::string>("kinematics_topic", "/tracking/relative/kinematics"))
  {
    publisher_ = create_publisher<px4_offboard_tracking::msg::RelativeKinematics>(
      kinematics_topic_, 10);
    subscription_ = create_subscription<px4_offboard_tracking::msg::RelativeEstimate>(
      estimate_topic_, 10,
      [this](px4_offboard_tracking::msg::RelativeEstimate::UniquePtr msg) {on_estimate(*msg);});
  }

private:
  void on_estimate(const px4_offboard_tracking::msg::RelativeEstimate & msg)
  {
    const auto result = compute_relative_kinematics(
      {msg.position.x, msg.position.y, msg.position.z},
      {msg.velocity.x, msg.velocity.y, msg.velocity.z});
    px4_offboard_tracking::msg::RelativeKinematics out;
    out.header = msg.header;
    out.range = result.range_m;
    out.relative_velocity.x = result.relative_velocity_mps[0];
    out.relative_velocity.y = result.relative_velocity_mps[1];
    out.relative_velocity.z = result.relative_velocity_mps[2];
    out.closing_speed = result.closing_speed_mps;
    out.los_rate.x = result.los_rate_rad_s[0];
    out.los_rate.y = result.los_rate_rad_s[1];
    out.los_rate.z = result.los_rate_rad_s[2];
    out.time_to_closest_approach = result.time_to_closest_approach_s;
    out.predicted_miss_distance = result.predicted_miss_distance_m;
    out.valid = msg.valid && result.valid;
    publisher_->publish(out);
  }

  std::string estimate_topic_;
  std::string kinematics_topic_;
  rclcpp::Publisher<px4_offboard_tracking::msg::RelativeKinematics>::SharedPtr publisher_;
  rclcpp::Subscription<px4_offboard_tracking::msg::RelativeEstimate>::SharedPtr subscription_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::RelativeKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
