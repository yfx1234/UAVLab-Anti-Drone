#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "px4_offboard_tracking/common_frame_transform.hpp"
#include "px4_offboard_tracking/task_frame.hpp"

namespace px4_offboard_tracking
{

// Simulation/evaluation-only node. It is deliberately a separate executable so
// the real-flight controller can be statically checked for zero truth dependency.
class SimulationTruthBridge : public rclcpp::Node
{
public:
  SimulationTruthBridge()
  : Node("simulation_truth_bridge"),
    target_state_topic_(declare_parameter<std::string>(
      "target_state_topic", "/tracking/target/state")),
    common_origin_topic_(declare_parameter<std::string>(
      "common_origin_topic", "/tracking/common_origin")),
    task_origin_topic_(declare_parameter<std::string>(
      "task_origin_topic", "/tracking/task_origin_ned")),
    pursuer_global_topic_(declare_parameter<std::string>(
      "pursuer_global_topic", "/px4_1/fmu/out/vehicle_global_position")),
    pursuer_local_topic_(declare_parameter<std::string>(
      "pursuer_local_topic", "/px4_1/fmu/out/vehicle_local_position")),
    output_topic_(declare_parameter<std::string>(
      "output_topic", "/evaluation/relative_truth")),
    common_frame_id_(declare_parameter<std::string>("common_frame_id", "map_ned")),
    pursuer_position_source_(declare_parameter<std::string>(
      "pursuer_position_source", "global")),
    maximum_age_s_(declare_parameter<double>("maximum_age_s", 0.25)),
    publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 50.0))
  {
    if (maximum_age_s_ <= 0.0 || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("invalid simulation truth timing configuration");
    }
    if (pursuer_position_source_ != "global" &&
      pursuer_position_source_ != "local")
    {
      throw std::invalid_argument(
              "pursuer_position_source must be 'global' or 'local'");
    }
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    auto origin_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    origin_qos.reliable().transient_local();
    if (pursuer_position_source_ == "global") {
      origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        common_origin_topic_, origin_qos,
        [this](sensor_msgs::msg::NavSatFix::UniquePtr msg) {
          const GeodeticPosition reference{msg->latitude, msg->longitude, msg->altitude};
          if (CommonFrameTransform::is_valid(reference)) {
            transform_ = std::make_unique<CommonFrameTransform>(reference);
          }
        });
    }
    task_origin_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      task_origin_topic_, origin_qos,
      [this](geometry_msgs::msg::PointStamped::UniquePtr msg) {
        if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) ||
          !std::isfinite(msg->point.z))
        {
          return;
        }
        task_origin_local_ = {msg->point.x, msg->point.y, msg->point.z};
        task_origin_local_valid_ = true;
      });
    target_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      target_state_topic_, 10,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        static rclcpp::Time last_receipt_log{0, 0, RCL_ROS_TIME};
        if ((now() - last_receipt_log).seconds() > 5.0) {
          last_receipt_log = now();
          RCLCPP_INFO(get_logger(), "truth bridge: target state received");
        }
        if (msg->header.frame_id != common_frame_id_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "truth bridge: target state frame mismatch got='%s' want='%s'",
            msg->header.frame_id.c_str(), common_frame_id_.c_str());
          return;
        }
        target_ = *msg;
        target_update_ = now();
      });
    if (pursuer_position_source_ == "global") {
      global_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        pursuer_global_topic_, px4_qos,
        [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
          const GeodeticPosition position{msg->lat, msg->lon, msg->alt_ellipsoid};
          if (!CommonFrameTransform::is_valid(position)) {
            return;
          }
          pursuer_global_ = position;
          global_update_ = now();
        });
    }
    local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      pursuer_local_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        if (!msg->xy_valid || !msg->z_valid ||
          !msg->v_xy_valid || !msg->v_z_valid ||
          !std::isfinite(msg->x) || !std::isfinite(msg->y) ||
          !std::isfinite(msg->z) || !std::isfinite(msg->vx) ||
          !std::isfinite(msg->vy) || !std::isfinite(msg->vz))
        {
          return;
        }
        pursuer_local_position_ = {msg->x, msg->y, msg->z};
        pursuer_velocity_ = {msg->vx, msg->vy, msg->vz};
        local_update_ = now();
      });
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {publish_truth();});
  }

private:
  bool fresh(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() > 0 && (now() - stamp).seconds() >= 0.0 &&
           (now() - stamp).seconds() <= maximum_age_s_;
  }

  void publish_truth()
  {
    // Global mode uses a static transient-local WGS84 origin. Local mode
    // subtracts target and pursuer positions directly in the same PX4 NED.
    if (!fresh(target_update_) || !fresh(local_update_)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "truth bridge gated: target_fresh=%d local_fresh=%d (src=%s)",
        static_cast<int>(fresh(target_update_)),
        static_cast<int>(fresh(local_update_)), pursuer_position_source_.c_str());
      return;
    }
    NedPosition pursuer;
    if (pursuer_position_source_ == "local") {
      if (!task_origin_local_valid_) {
        return;
      }
      const auto pursuer_task = to_task_ned(
        pursuer_local_position_, task_origin_local_);
      pursuer = {pursuer_task[0], pursuer_task[1], pursuer_task[2]};
    } else {
      if (!transform_ || !fresh(global_update_)) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "truth bridge gated: transform=%d global_fresh=%d",
          static_cast<int>(transform_ != nullptr),
          static_cast<int>(fresh(global_update_)));
        return;
      }
      pursuer = transform_->to_ned(pursuer_global_);
    }
    nav_msgs::msg::Odometry output;
    output.header.stamp = now();
    output.header.frame_id = common_frame_id_;
    output.child_frame_id = "target_relative_to_pursuer_truth";
    output.pose.pose.position.x = target_.pose.pose.position.x - pursuer.north_m;
    output.pose.pose.position.y = target_.pose.pose.position.y - pursuer.east_m;
    output.pose.pose.position.z = target_.pose.pose.position.z - pursuer.down_m;
    output.twist.twist.linear.x =
      target_.twist.twist.linear.x - pursuer_velocity_[0];
    output.twist.twist.linear.y =
      target_.twist.twist.linear.y - pursuer_velocity_[1];
    output.twist.twist.linear.z =
      target_.twist.twist.linear.z - pursuer_velocity_[2];
    truth_pub_->publish(output);
  }

  std::string target_state_topic_;
  std::string common_origin_topic_;
  std::string task_origin_topic_;
  std::string pursuer_global_topic_;
  std::string pursuer_local_topic_;
  std::string output_topic_;
  std::string common_frame_id_;
  std::string pursuer_position_source_;
  double maximum_age_s_;
  double publish_rate_hz_;
  std::unique_ptr<CommonFrameTransform> transform_;
  nav_msgs::msg::Odometry target_{};
  GeodeticPosition pursuer_global_{};
  std::array<double, 3> pursuer_local_position_{};
  std::array<double, 3> task_origin_local_{};
  std::array<double, 3> pursuer_velocity_{};
  bool task_origin_local_valid_{false};
  rclcpp::Time target_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time global_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr task_origin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::SimulationTruthBridge>());
  rclcpp::shutdown();
  return 0;
}
