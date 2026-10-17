#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "px4_offboard_tracking/common_frame_transform.hpp"
#include "px4_offboard_tracking/task_frame.hpp"
#include "px4_offboard_tracking/target_trajectory.hpp"

namespace px4_offboard_tracking
{
namespace
{

enum class TargetSourceMode { VIRTUAL, EXTERNAL_PX4 };

TargetSourceMode target_source_mode_from_string(const std::string & value)
{
  if (value == "virtual") {
    return TargetSourceMode::VIRTUAL;
  }
  if (value == "external_px4") {
    return TargetSourceMode::EXTERNAL_PX4;
  }
  throw std::invalid_argument(
          "target_source must be 'virtual' or 'external_px4'");
}

} // namespace

// Test-only source boundary. Target coordinates terminate here and are
// converted to a shared NED test frame for the logical camera simulator. The
// virtual real-aircraft entry uses the pursuer's PX4 local NED directly; other
// coordinate entries retain their WGS84-derived common NED.
// The production estimator/controller never subscribe to this node.
class TargetStateSource : public rclcpp::Node
{
public:
  TargetStateSource()
  : Node("target_state_source"),
    mode_(target_source_mode_from_string(
        declare_parameter<std::string>("target_source", "virtual"))),
    target_state_topic_(declare_parameter<std::string>(
        "target_state_topic", "/tracking/test_target/state")),
    common_origin_topic_(declare_parameter<std::string>(
        "common_origin_topic", "/tracking/test_target/common_origin")),
    task_origin_topic_(declare_parameter<std::string>(
        "task_origin_topic", "/tracking/task_origin_ned")),
    mission_ready_topic_(declare_parameter<std::string>(
        "mission_ready_topic", "/simulation/target_motion_ready")),
    pursuer_takeoff_ready_topic_(declare_parameter<std::string>(
        "pursuer_takeoff_ready_topic",
        "/simulation/pursuer_takeoff_ready")),
    pursuer_global_topic_(declare_parameter<std::string>(
      "pursuer_global_topic", "/fmu/out/vehicle_global_position")),
    pursuer_local_topic_(declare_parameter<std::string>(
      "pursuer_local_topic", "/fmu/out/vehicle_local_position")),
    external_target_global_topic_(declare_parameter<std::string>(
        "external_target_global_topic",
        "/target/fmu/out/vehicle_global_position")),
    external_target_local_topic_(declare_parameter<std::string>(
        "external_target_local_topic",
        "/target/fmu/out/vehicle_local_position")),
    common_frame_id_(
      declare_parameter<std::string>("common_frame_id", "map_ned")),
    wait_for_pursuer_takeoff_ready_(
      declare_parameter<bool>("wait_for_pursuer_takeoff_ready", true)),
    publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 50.0)),
    maximum_input_age_s_(
      declare_parameter<double>("maximum_input_age_s", 0.5)),
    motion_delay_s_(
      declare_parameter<double>("target_motion_delay_s", 5.0)),
    motion_ramp_s_(declare_parameter<double>("target_motion_ramp_s", 3.0)),
    mission_ready_motion_s_(
      declare_parameter<double>("mission_ready_motion_s", 0.2)),
    virtual_initial_north_m_(
      declare_parameter<double>("virtual_initial_north_m", 0.0)),
    virtual_initial_east_m_(
      declare_parameter<double>("virtual_initial_east_m", 0.0)),
    virtual_spawn_relative_to_pursuer_on_ready_(declare_parameter<bool>(
        "virtual_spawn_relative_to_pursuer_on_ready", false)),
    virtual_publish_before_pursuer_ready_(declare_parameter<bool>(
        "virtual_publish_before_pursuer_ready", true)),
    virtual_use_pursuer_local_position_(declare_parameter<bool>(
        "virtual_use_pursuer_local_position", false)),
    virtual_height_above_anchor_m_(
      declare_parameter<double>("target_height_m", 30.0)),
    virtual_trajectory_(
      target_scenario_from_string(
        declare_parameter<std::string>("target_scenario", "straight")),
      virtual_height_above_anchor_m_,
      declare_parameter<double>("target_radius_m", 50.0),
      declare_parameter<double>("target_speed_mps", 5.0),
      declare_parameter<double>("target_heading_rad", 0.0),
      declare_parameter<double>("target_path_length_m", 100.0),
      declare_parameter<double>("target_path_width_m", 20.0))
  {
    if (publish_rate_hz_ <= 0.0 || maximum_input_age_s_ <= 0.0 ||
      motion_delay_s_ < 0.0 || motion_ramp_s_ < 0.0 ||
      mission_ready_motion_s_ < 0.0)
    {
      throw std::invalid_argument("invalid target state source timing");
    }

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latched_qos.reliable().transient_local();
    if (mode_ == TargetSourceMode::EXTERNAL_PX4 ||
      !virtual_use_pursuer_local_position_)
    {
      pursuer_global_sub_ =
        create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        pursuer_global_topic_, px4_qos,
        [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr message) {
          const GeodeticPosition position{message->lat, message->lon,
            message->alt_ellipsoid};
          if (!CommonFrameTransform::is_valid(position)) {
            return;
          }
          pursuer_global_ = position;
          pursuer_global_update_ = now();
        });
    }
    if (mode_ == TargetSourceMode::VIRTUAL &&
      virtual_use_pursuer_local_position_)
    {
      pursuer_local_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        pursuer_local_topic_, px4_qos,
        [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
          if (!message->xy_valid || !message->z_valid ||
            !std::isfinite(message->x) || !std::isfinite(message->y) ||
            !std::isfinite(message->z))
          {
            return;
          }
          pursuer_local_position_ = {message->x, message->y, message->z};
          pursuer_local_update_ = now();
        });
    }
    common_origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      common_origin_topic_, latched_qos,
      [this](sensor_msgs::msg::NavSatFix::UniquePtr message) {
        const GeodeticPosition reference{
          message->latitude, message->longitude, message->altitude};
        if (!CommonFrameTransform::is_valid(reference)) {
          return;
        }
        transform_ = std::make_unique<CommonFrameTransform>(reference);
        source_ready_time_ = now();
        RCLCPP_INFO(
          get_logger(),
          "task common origin received: %.8f %.8f %.2f",
          reference.latitude_deg, reference.longitude_deg,
          reference.altitude_ellipsoid_m);
      });
    task_origin_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      task_origin_topic_, latched_qos,
      [this](geometry_msgs::msg::PointStamped::UniquePtr message) {
        if (!std::isfinite(message->point.x) ||
          !std::isfinite(message->point.y) ||
          !std::isfinite(message->point.z))
        {
          return;
        }
        task_origin_local_ = {
          message->point.x, message->point.y, message->point.z};
        task_origin_local_valid_ = true;
      });
    pursuer_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      pursuer_takeoff_ready_topic_, latched_qos,
      [this](std_msgs::msg::Bool::UniquePtr message) {
        if (message->data && !pursuer_takeoff_ready_) {
          pursuer_takeoff_ready_ = true;
          pursuer_ready_time_ = now();
          RCLCPP_INFO(
            get_logger(), "pursuer Offboard/takeoff gate opened; "
            "virtual target may spawn");
        }
      });

    if (mode_ == TargetSourceMode::EXTERNAL_PX4) {
      external_global_sub_ =
        create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        external_target_global_topic_, px4_qos,
        [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr message) {
          const GeodeticPosition position{message->lat, message->lon,
            message->alt_ellipsoid};
          if (!CommonFrameTransform::is_valid(position)) {
            return;
          }
          external_target_global_ = position;
          external_global_update_ = now();
        });
      external_local_sub_ = create_subscription<
        px4_msgs::msg::VehicleLocalPosition>(
        external_target_local_topic_, px4_qos,
        [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
          if (!message->v_xy_valid || !message->v_z_valid ||
          !std::isfinite(message->vx) || !std::isfinite(message->vy) ||
          !std::isfinite(message->vz))
          {
            return;
          }
          external_velocity_ned_ = {message->vx, message->vy, message->vz};
          external_local_update_ = now();
        });
    }

    target_state_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(target_state_topic_, 10);
    mission_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      mission_ready_topic_, latched_qos);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {on_timer();});
    RCLCPP_INFO(
      get_logger(), "target state source ready: mode=%s, output=%s",
      mode_ == TargetSourceMode::VIRTUAL ? "virtual" : "external_px4",
      target_state_topic_.c_str());
  }

private:
  bool fresh(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() > 0 && (now() - stamp).seconds() >= 0.0 &&
           (now() - stamp).seconds() <= maximum_input_age_s_;
  }

  bool pursuer_gate_open() const
  {
    return !wait_for_pursuer_takeoff_ready_ || pursuer_takeoff_ready_;
  }

  bool capture_virtual_spawn_anchor()
  {
    if (!virtual_spawn_relative_to_pursuer_on_ready_) {
      return true;
    }
    if (virtual_spawn_anchor_captured_) {
      return true;
    }
    if (!pursuer_gate_open()) {
      return false;
    }
    if (virtual_use_pursuer_local_position_) {
      if (!fresh(pursuer_local_update_)) {
        return false;
      }
      if (!task_origin_local_valid_) {
        return false;
      }
      const auto pursuer_task_position = to_task_ned(
        pursuer_local_position_, task_origin_local_);
      virtual_spawn_north_m_ =
        pursuer_task_position[0] + virtual_initial_north_m_;
      virtual_spawn_east_m_ =
        pursuer_task_position[1] + virtual_initial_east_m_;
      virtual_spawn_down_m_ = pursuer_task_position[2];
    } else {
      if (!transform_ || !fresh(pursuer_global_update_)) {
        return false;
      }
      const auto pursuer_ned = transform_->to_ned(pursuer_global_);
      virtual_spawn_north_m_ =
        pursuer_ned.north_m + virtual_initial_north_m_;
      virtual_spawn_east_m_ =
        pursuer_ned.east_m + virtual_initial_east_m_;
      virtual_spawn_down_m_ = 0.0;
    }
    virtual_spawn_anchor_captured_ = true;
    virtual_first_sample_pending_ = true;
    virtual_spawn_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "virtual target anchor captured from pursuer: N=%.2f m E=%.2f m "
      "D=%.2f m; initial target N/E offsets=[%.2f, %.2f] m, "
      "target D=%.2f m (%.2f m above anchor)",
      virtual_spawn_north_m_, virtual_spawn_east_m_, virtual_spawn_down_m_,
      virtual_initial_north_m_, virtual_initial_east_m_,
      virtual_spawn_down_m_ - virtual_height_above_anchor_m_,
      virtual_height_above_anchor_m_);
    return true;
  }

  void publish_ready()
  {
    if (mission_ready_published_) {
      return;
    }
    std_msgs::msg::Bool output;
    output.data = true;
    mission_ready_pub_->publish(output);
    mission_ready_published_ = true;
    RCLCPP_INFO(
      get_logger(),
      "test target source ready; camera-only mission may start");
  }

  void publish_target(const TargetTrajectorySetpoint & sample)
  {
    nav_msgs::msg::Odometry output;
    output.header.stamp = now();
    output.header.frame_id = common_frame_id_;
    output.child_frame_id = mode_ == TargetSourceMode::VIRTUAL ?
      "virtual_target" :
      "external_px4_target";
    output.pose.pose.position.x = sample.x;
    output.pose.pose.position.y = sample.y;
    output.pose.pose.position.z = sample.z;
    output.twist.twist.linear.x = sample.vx;
    output.twist.twist.linear.y = sample.vy;
    output.twist.twist.linear.z = sample.vz;
    target_state_pub_->publish(output);
  }

  void on_timer()
  {
    if (mode_ == TargetSourceMode::VIRTUAL &&
      virtual_use_pursuer_local_position_)
    {
      if (!fresh(pursuer_local_update_)) {
        return;
      }
    } else {
      if (!transform_ || !fresh(pursuer_global_update_)) {
        return;
      }
    }
    if (mode_ == TargetSourceMode::VIRTUAL) {
      if (!pursuer_gate_open() && !virtual_publish_before_pursuer_ready_) {
        return;
      }
      if (!capture_virtual_spawn_anchor()) {
        return;
      }
      const double initial_north_m = virtual_spawn_relative_to_pursuer_on_ready_ ?
        virtual_spawn_north_m_ :
        virtual_initial_north_m_;
      const double initial_east_m = virtual_spawn_relative_to_pursuer_on_ready_ ?
        virtual_spawn_east_m_ :
        virtual_initial_east_m_;
      const double initial_down_m =
        virtual_spawn_relative_to_pursuer_on_ready_ ?
        virtual_spawn_down_m_ : 0.0;
      // 兜底锚定：新架构（netgun 链）只发布 task_origin_ned，无人发布
      // common_origin（NavSatFix，旧 simple_below_tracker 的隐性服务）。
      // 不锚定时 motion_elapsed = now()-0 = epoch 秒，虚拟目标会飞到
      // ~1.8e9 m 外，相机 12m 截断导致全链观测无效（2026-10-12 定位）。
      if (source_ready_time_.nanoseconds() <= 0) {
        source_ready_time_ = now();
        RCLCPP_INFO(
          get_logger(),
          "virtual clock anchored at node start (no common origin received)");
      }
      double motion_elapsed_s = 0.0;
      if (pursuer_gate_open()) {
        const rclcpp::Time gate_time =
          virtual_spawn_relative_to_pursuer_on_ready_ ?
          virtual_spawn_time_ :
          (wait_for_pursuer_takeoff_ready_ ? pursuer_ready_time_ :
          source_ready_time_);
        motion_elapsed_s =
          std::max(0.0, (now() - gate_time).seconds() - motion_delay_s_);
      }
      if (virtual_first_sample_pending_) {
        // The first published pose is exactly the latched Offboard-entry
        // position. With a zero ramp its velocity is already the configured
        // trajectory velocity, without advancing the position by one timer
        // period before observers see the anchor.
        motion_elapsed_s = 0.0;
      }
      publish_target(
        virtual_trajectory_.sample_with_speed_ramp(
          initial_north_m, initial_east_m, initial_down_m, motion_elapsed_s,
          motion_ramp_s_));
      virtual_first_sample_pending_ = false;
      if (pursuer_gate_open() && motion_elapsed_s >= mission_ready_motion_s_) {
        publish_ready();
      }
      return;
    }

    if (!fresh(external_global_update_)) {
      return;
    }
    const auto target_ned = transform_->to_ned(external_target_global_);
    TargetTrajectorySetpoint sample;
    sample.x = target_ned.north_m;
    sample.y = target_ned.east_m;
    sample.z = target_ned.down_m;
    if (fresh(external_local_update_)) {
      sample.vx = external_velocity_ned_[0];
      sample.vy = external_velocity_ned_[1];
      sample.vz = external_velocity_ned_[2];
    }
    publish_target(sample);
    if (pursuer_gate_open()) {
      publish_ready();
    }
  }

  TargetSourceMode mode_;
  std::string target_state_topic_;
  std::string common_origin_topic_;
  std::string task_origin_topic_;
  std::string mission_ready_topic_;
  std::string pursuer_takeoff_ready_topic_;
  std::string pursuer_global_topic_;
  std::string pursuer_local_topic_;
  std::string external_target_global_topic_;
  std::string external_target_local_topic_;
  std::string common_frame_id_;
  bool wait_for_pursuer_takeoff_ready_;
  double publish_rate_hz_;
  double maximum_input_age_s_;
  double motion_delay_s_;
  double motion_ramp_s_;
  double mission_ready_motion_s_;
  double virtual_initial_north_m_;
  double virtual_initial_east_m_;
  bool virtual_spawn_relative_to_pursuer_on_ready_;
  bool virtual_publish_before_pursuer_ready_;
  bool virtual_use_pursuer_local_position_;
  double virtual_height_above_anchor_m_;
  TargetTrajectory virtual_trajectory_;
  std::unique_ptr<CommonFrameTransform> transform_;
  GeodeticPosition pursuer_global_{};
  GeodeticPosition external_target_global_{};
  std::array<double, 3> external_velocity_ned_{};
  std::array<double, 3> pursuer_local_position_{};
  std::array<double, 3> task_origin_local_{};
  bool pursuer_takeoff_ready_{false};
  bool task_origin_local_valid_{false};
  bool virtual_spawn_anchor_captured_{false};
  bool virtual_first_sample_pending_{false};
  bool mission_ready_published_{false};
  double virtual_spawn_north_m_{};
  double virtual_spawn_east_m_{};
  double virtual_spawn_down_m_{};
  rclcpp::Time pursuer_global_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pursuer_local_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time external_global_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time external_local_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pursuer_ready_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time virtual_spawn_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time source_ready_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr
    pursuer_global_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    pursuer_local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr
    external_global_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    external_local_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pursuer_ready_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr common_origin_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr task_origin_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::TargetStateSource>());
  rclcpp::shutdown();
  return 0;
}
