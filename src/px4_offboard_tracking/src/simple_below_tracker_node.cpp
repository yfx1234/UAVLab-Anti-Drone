#include <anti_drone_interfaces/msg/depth_observation.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "px4_offboard_tracking/msg/simple_below_tracker_status.hpp"
#include "px4_offboard_tracking/common_frame_transform.hpp"
#include "px4_offboard_tracking/task_frame.hpp"
#include "px4_offboard_tracking/simple_below_tracker.hpp"

namespace px4_offboard_tracking
{
namespace
{

using Vector3 = std::array<double, 3>;

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

bool finite(const Vector3 & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

double norm(const Vector3 & value)
{
  return std::sqrt(
    value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

Vector3 rotate_body_to_ned(const Quaternion & input, const Vector3 & body)
{
  const double quaternion_norm = std::sqrt(
    input.w * input.w + input.x * input.x + input.y * input.y + input.z * input.z);
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-9) {
    return {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  }
  const Quaternion q{
    input.w / quaternion_norm, input.x / quaternion_norm,
    input.y / quaternion_norm, input.z / quaternion_norm};
  const Vector3 qv{q.x, q.y, q.z};
  const Vector3 twice_cross{
    2.0 * (qv[1] * body[2] - qv[2] * body[1]),
    2.0 * (qv[2] * body[0] - qv[0] * body[2]),
    2.0 * (qv[0] * body[1] - qv[1] * body[0])};
  const Vector3 second_cross{
    qv[1] * twice_cross[2] - qv[2] * twice_cross[1],
    qv[2] * twice_cross[0] - qv[0] * twice_cross[2],
    qv[0] * twice_cross[1] - qv[1] * twice_cross[0]};
  return {
    body[0] + q.w * twice_cross[0] + second_cross[0],
    body[1] + q.w * twice_cross[1] + second_cross[1],
    body[2] + q.w * twice_cross[2] + second_cross[2]};
}

}  // namespace

class SimpleBelowTrackerNode : public rclcpp::Node
{
public:
  SimpleBelowTrackerNode()
  : Node("simple_below_tracker"),
    topic_prefix_(declare_parameter<std::string>("topic_prefix", "/px4_1/fmu")),
    depth_topic_(declare_parameter<std::string>(
      "depth_topic", "/perception/depth/observation")),
    enable_topic_(declare_parameter<std::string>(
      "enable_topic", "/tracking/simple_below/enable")),
    status_topic_(declare_parameter<std::string>(
      "status_topic", "/tracking/simple_below/status")),
    task_origin_topic_(declare_parameter<std::string>(
      "task_origin_topic", "/tracking/task_origin_ned")),
    common_origin_topic_(declare_parameter<std::string>(
      "common_origin_topic", "/tracking/common_origin")),
    pursuer_takeoff_ready_topic_(declare_parameter<std::string>(
      "pursuer_takeoff_ready_topic", "/simulation/pursuer_takeoff_ready")),
    mode_(parse_simple_below_tracking_mode(
      declare_parameter<std::string>("tracking_mode", "predictive"))),
    target_system_(declare_parameter<int>("target_system", 2)),
    control_rate_hz_(declare_parameter<double>("control_rate_hz", 50.0)),
    observation_timeout_s_(declare_parameter<double>("observation_timeout_s", 0.25)),
    state_timeout_s_(declare_parameter<double>("state_timeout_s", 0.5)),
    status_timeout_s_(declare_parameter<double>("vehicle_status_timeout_s", 1.0)),
    control_enabled_(declare_parameter<bool>("control_enabled", false)),
    auto_engage_(declare_parameter<bool>("auto_engage", false)),
    auto_disable_on_offboard_exit_(
      declare_parameter<bool>("auto_disable_on_offboard_exit", true)),
    publish_pursuer_takeoff_ready_(
      declare_parameter<bool>("publish_pursuer_takeoff_ready", false)),
    takeoff_enabled_(declare_parameter<bool>("takeoff_enabled", false)),
    takeoff_height_m_(declare_parameter<double>("takeoff_height_m", 5.0)),
    takeoff_tolerance_m_(declare_parameter<double>("takeoff_tolerance_m", 0.5)),
    takeoff_vertical_speed_tolerance_mps_(
      declare_parameter<double>("takeoff_vertical_speed_tolerance_mps", 0.5)),
    altitude_position_gain_(declare_parameter<double>("altitude_position_gain", 0.8)),
    maximum_vertical_speed_mps_(
      declare_parameter<double>("maximum_vertical_speed_mps", 1.0)),
    minimum_flight_height_m_(declare_parameter<double>("minimum_flight_height_m", 3.0)),
    minimum_target_height_above_m_(
      declare_parameter<double>("minimum_target_height_above_m", 2.0)),
    maximum_tracking_range_m_(declare_parameter<double>("maximum_tracking_range_m", 60.0)),
    maximum_vehicle_speed_mps_(declare_parameter<double>("maximum_vehicle_speed_mps", 8.0)),
    warmup_setpoint_count_(static_cast<std::size_t>(std::ceil(
      control_rate_hz_ * declare_parameter<double>("offboard_warmup_s", 1.0)))),
    preflight_stability_hold_s_(
      declare_parameter<double>("preflight_stability_hold_s", 3.0)),
    tracker_(SimpleBelowTrackerConfig{
      declare_parameter<double>("horizontal_position_gain", 0.6),
      declare_parameter<double>("prediction_horizon_s", 0.2),
      declare_parameter<double>("relative_velocity_filter_time_constant_s", 0.15),
      declare_parameter<double>("minimum_observation_dt_s", 0.005),
      declare_parameter<double>("maximum_observation_dt_s", 0.25),
      declare_parameter<double>("maximum_relative_speed_mps", 12.0),
      declare_parameter<double>("maximum_prediction_distance_m", 2.0),
      declare_parameter<double>("maximum_horizontal_speed_mps", 3.0),
      declare_parameter<double>("maximum_horizontal_acceleration_mps2", 1.5)})
  {
    if (target_system_ < 1 || target_system_ > 255 || control_rate_hz_ < 2.0 ||
      observation_timeout_s_ <= 0.0 || state_timeout_s_ <= 0.0 ||
      status_timeout_s_ <= 0.0 || takeoff_height_m_ <= 0.0 ||
      takeoff_tolerance_m_ <= 0.0 || takeoff_vertical_speed_tolerance_mps_ <= 0.0 ||
      altitude_position_gain_ <= 0.0 || maximum_vertical_speed_mps_ <= 0.0 ||
      minimum_flight_height_m_ <= 0.0 || minimum_target_height_above_m_ <= 0.0 ||
      maximum_tracking_range_m_ <= minimum_target_height_above_m_ ||
      maximum_vehicle_speed_mps_ <= 0.0 || warmup_setpoint_count_ == 0 ||
      preflight_stability_hold_s_ < 0.0)
    {
      throw std::invalid_argument("invalid simple below tracker node configuration");
    }

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    auto origin_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    origin_qos.reliable().transient_local();
    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      topic("in/offboard_control_mode"), px4_qos);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic("in/trajectory_setpoint"), px4_qos);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      topic("in/vehicle_command"), px4_qos);
    local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topic("out/vehicle_local_position"), px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
        on_local_position(*message);
      });
    global_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      topic("out/vehicle_global_position"), px4_qos,
      [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr message) {
        const GeodeticPosition position{
          message->lat, message->lon, message->alt_ellipsoid};
        if (!CommonFrameTransform::is_valid(position)) {
          return;
        }
        latest_global_position_ = position;
        global_valid_ = true;
        if (local_valid_) {
          local_position_at_global_ = local_position_;
          local_at_global_valid_ = true;
        }
        publish_common_origin_if_ready();
      });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      topic("out/vehicle_attitude"), px4_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr message) {
        attitude_ = {message->q[0], message->q[1], message->q[2], message->q[3]};
        attitude_valid_ = std::isfinite(attitude_.w) && std::isfinite(attitude_.x) &&
          std::isfinite(attitude_.y) && std::isfinite(attitude_.z);
        attitude_update_ = std::chrono::steady_clock::now();
      });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      topic("out/vehicle_status"), px4_qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr message) {
        vehicle_status_ = *message;
        vehicle_status_valid_ = true;
        vehicle_status_update_ = now();
        if (message->arming_state ==
          px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
        {
          lock_task_origin("PX4 armed transition");
        }
      });
    depth_sub_ = create_subscription<anti_drone_interfaces::msg::DepthObservation>(
      depth_topic_, rclcpp::SensorDataQoS(),
      [this](anti_drone_interfaces::msg::DepthObservation::UniquePtr message) {
        on_depth_observation(*message);
      });

    auto enable_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    enable_qos.reliable().transient_local();
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, enable_qos,
      [this](std_msgs::msg::Bool::UniquePtr message) {
        const bool enabling = message->data && !control_enabled_;
        if (enabling && !auto_engage_ && !local_fresh()) {
          control_enabled_ = false;
          RCLCPP_ERROR(
            get_logger(), "rejected manual enable: PX4 local state is not fresh");
          return;
        }
        if (enabling && !auto_engage_ && offboard_active()) {
          control_enabled_ = false;
          RCLCPP_ERROR(
            get_logger(),
            "rejected manual enable while already in Offboard; "
            "enable pre-stream before switching modes");
          return;
        }
        control_enabled_ = message->data;
        if (enabling) {
          warmup_count_ = 0;
          offboard_session_seen_ = false;
          stopped_after_offboard_exit_ = false;
        }
        if (enabling && !takeoff_enabled_ && local_fresh()) {
          // The real-aircraft entry may be started before manual takeoff.
          // Latch altitude at the explicit enable edge, not process startup,
          // so switching to Offboard cannot command a return to ground level.
          height_hold_z_ = local_position_[2];
          yaw_hold_rad_ = current_heading_rad_;
          height_hold_set_ = true;
          RCLCPP_INFO(
            get_logger(), "re-latched real-flight hold z=%.2f NED at enable",
            height_hold_z_);
        }
        if (!control_enabled_) {
          tracker_.reset_observation();
        }
        RCLCPP_WARN(
          get_logger(), "simple below tracking %s by enable topic",
          control_enabled_ ? "enabled" : "disabled");
      });
    status_pub_ = create_publisher<px4_offboard_tracking::msg::SimpleBelowTrackerStatus>(
      status_topic_, 10);
    task_origin_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      task_origin_topic_, origin_qos);
    common_origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      common_origin_topic_, origin_qos);
    if (publish_pursuer_takeoff_ready_) {
      auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      ready_qos.reliable().transient_local();
      pursuer_takeoff_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
        pursuer_takeoff_ready_topic_, ready_qos);
    }

    control_period_s_ = 1.0 / control_rate_hz_;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_period_s_));
    timer_ = create_wall_timer(period, [this]() {on_timer();});

    RCLCPP_INFO(
      get_logger(),
      "simple below tracker ready: mode=%s horizon=%.3f s, depth=%s, PX4=%s, "
      "auto_engage=%s, enabled=%s, stop_on_offboard_exit=%s",
      to_string(mode_), get_parameter("prediction_horizon_s").as_double(),
      depth_topic_.c_str(), topic_prefix_.c_str(), auto_engage_ ? "true" : "false",
      control_enabled_ ? "true" : "false",
      auto_disable_on_offboard_exit_ ? "true" : "false");
  }

private:
  std::string topic(const std::string & suffix) const
  {
    std::string prefix = topic_prefix_;
    while (prefix.size() > 1 && prefix.back() == '/') {
      prefix.pop_back();
    }
    if (prefix.empty() || prefix.front() != '/') {
      prefix.insert(prefix.begin(), '/');
    }
    return prefix + "/" + suffix;
  }

  std::uint64_t timestamp_us()
  {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  double message_age(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() <= 0) {
      return std::numeric_limits<double>::infinity();
    }
    const double age = (now() - stamp).seconds();
    return age >= 0.0 ? age : std::numeric_limits<double>::infinity();
  }

  bool local_fresh() const
  {
    return local_valid_ &&
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - local_update_).count() <= state_timeout_s_;
  }

  bool attitude_fresh() const
  {
    return attitude_valid_ &&
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - attitude_update_).count() <= state_timeout_s_;
  }

  bool status_fresh() const
  {
    return vehicle_status_valid_ && message_age(vehicle_status_update_) <= status_timeout_s_;
  }

  bool armed() const
  {
    return status_fresh() &&
      vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool offboard_active() const
  {
    return status_fresh() &&
      vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool heartbeat_streaming() const
  {
    return control_enabled_ || auto_engage_;
  }

  bool setpoint_streaming() const
  {
    // OffboardControlMode is the ROS 2 proof-of-life stream. Keep trajectory
    // setpoints completely absent until PX4 confirms Offboard in a fresh
    // VehicleStatus sample. Preserve the confirmed-session latch through a
    // transient status gap; only a fresh explicit exit stops the manual flow.
    return heartbeat_streaming() && (offboard_active() || offboard_session_seen_);
  }

  bool flight_control_active() const
  {
    return auto_engage_ || (control_enabled_ && offboard_active());
  }

  void update_manual_offboard_session()
  {
    if (heartbeat_streaming() && offboard_active()) {
      offboard_session_seen_ = true;
      return;
    }
    if (auto_engage_ || !auto_disable_on_offboard_exit_) {
      return;
    }
    const bool explicit_offboard_exit =
      status_fresh() &&
      vehicle_status_.nav_state !=
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (control_enabled_ && offboard_session_seen_ && explicit_offboard_exit) {
      control_enabled_ = false;
      stopped_after_offboard_exit_ = true;
      tracker_.reset_observation();
      RCLCPP_WARN(
        get_logger(),
        "manual Offboard exit detected: stopped OffboardControlMode and "
        "TrajectorySetpoint publication for this one-shot flight");
    }
  }

  bool takeoff_ready() const
  {
    if (!local_fresh() || !height_hold_set_) {
      return false;
    }
    return std::abs(height_hold_z_ - local_position_[2]) <= takeoff_tolerance_m_ &&
           std::abs(local_velocity_[2]) <= takeoff_vertical_speed_tolerance_mps_;
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition & message)
  {
    if (!message.xy_valid || !message.z_valid || !message.v_xy_valid || !message.v_z_valid) {
      return;
    }
    if (!local_reset_counters_initialized_) {
      local_xy_reset_counter_ = message.xy_reset_counter;
      local_z_reset_counter_ = message.z_reset_counter;
      local_reset_counters_initialized_ = true;
    } else {
      if (message.xy_reset_counter != local_xy_reset_counter_ &&
        std::isfinite(message.delta_xy[0]) &&
        std::isfinite(message.delta_xy[1]))
      {
        if (task_origin_locked_) {
          local_origin_[0] += message.delta_xy[0];
          local_origin_[1] += message.delta_xy[1];
        }
        if (local_at_global_valid_) {
          local_position_at_global_[0] += message.delta_xy[0];
          local_position_at_global_[1] += message.delta_xy[1];
        }
        local_xy_reset_counter_ = message.xy_reset_counter;
        publish_task_origin();
      }
      if (message.z_reset_counter != local_z_reset_counter_ &&
        std::isfinite(message.delta_z))
      {
        if (task_origin_locked_) {
          local_origin_[2] += message.delta_z;
        }
        if (local_at_global_valid_) {
          local_position_at_global_[2] += message.delta_z;
        }
        height_hold_z_ += message.delta_z;
        local_z_reset_counter_ = message.z_reset_counter;
        publish_task_origin();
      }
    }
    local_position_ = {message.x, message.y, message.z};
    local_velocity_ = {message.vx, message.vy, message.vz};
    local_valid_ = finite(local_position_) && finite(local_velocity_);
    if (std::isfinite(message.heading)) {
      current_heading_rad_ = message.heading;
    }
    local_update_ = std::chrono::steady_clock::now();
    if (local_valid_ && !height_hold_set_) {
      local_origin_ = local_position_;
      height_hold_z_ = takeoff_enabled_ ?
        local_origin_[2] - takeoff_height_m_ : local_position_[2];
      yaw_hold_rad_ = current_heading_rad_;
      height_hold_set_ = true;
      RCLCPP_INFO(
        get_logger(), "latched height setpoint z=%.2f NED (takeoff=%s)",
        height_hold_z_, takeoff_enabled_ ? "enabled" : "disabled");
    } else if (local_valid_ && !task_origin_locked_ && !armed()) {
      local_origin_ = local_position_;
      if (takeoff_enabled_) {
        height_hold_z_ = local_origin_[2] - takeoff_height_m_;
      }
    }
    if (local_valid_ && control_enabled_ && !auto_engage_ &&
      auto_disable_on_offboard_exit_ && !offboard_session_seen_ &&
      !offboard_active())
    {
      // Manual nodes start before takeoff and heartbeat throughout the
      // pilot-controlled climb. Follow the live pose until RC enters
      // Offboard, then freeze this hold before the first trajectory setpoint.
      height_hold_z_ = local_position_[2];
      yaw_hold_rad_ = current_heading_rad_;
    }
    publish_common_origin_if_ready();
  }

  void publish_task_origin()
  {
    if (!task_origin_locked_ || !task_origin_pub_) {
      return;
    }
    geometry_msgs::msg::PointStamped output;
    output.header.stamp = now();
    output.header.frame_id = "px4_local_ned";
    output.point.x = local_origin_[0];
    output.point.y = local_origin_[1];
    output.point.z = local_origin_[2];
    task_origin_pub_->publish(output);
  }

  void publish_common_origin_if_ready()
  {
    if (!task_origin_locked_ || common_origin_published_ || !global_valid_ ||
      !local_at_global_valid_ || !common_origin_pub_)
    {
      return;
    }
    const CommonFrameTransform current_global_frame(latest_global_position_);
    common_geodetic_origin_ = current_global_frame.from_ned({
      local_origin_[0] - local_position_at_global_[0],
      local_origin_[1] - local_position_at_global_[1],
      local_origin_[2] - local_position_at_global_[2]});
    sensor_msgs::msg::NavSatFix output;
    output.header.stamp = now();
    output.header.frame_id = "map_ned_origin";
    output.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    output.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    output.latitude = common_geodetic_origin_.latitude_deg;
    output.longitude = common_geodetic_origin_.longitude_deg;
    output.altitude = common_geodetic_origin_.altitude_ellipsoid_m;
    common_origin_pub_->publish(output);
    common_origin_published_ = true;
  }

  void lock_task_origin(const char * reason)
  {
    if (task_origin_locked_ || !local_valid_) {
      return;
    }
    local_origin_ = local_position_;
    task_origin_locked_ = true;
    publish_task_origin();
    publish_common_origin_if_ready();
    RCLCPP_INFO(
      get_logger(),
      "task NED origin locked at propeller start [%.2f, %.2f, %.2f]: %s",
      local_origin_[0], local_origin_[1], local_origin_[2], reason);
  }

  bool valid_depth_message(
    const anti_drone_interfaces::msg::DepthObservation & message) const
  {
    return message.interface_version ==
      anti_drone_interfaces::msg::DepthObservation::INTERFACE_VERSION_V1 &&
      message.visible && message.depth_valid && message.angles_valid &&
      std::isfinite(message.position_optical.x) &&
      std::isfinite(message.position_optical.y) &&
      std::isfinite(message.position_optical.z) &&
      message.position_optical.z > 0.0 && std::isfinite(message.range_m) &&
      message.range_m > minimum_target_height_above_m_ &&
      message.range_m <= maximum_tracking_range_m_;
  }

  void on_depth_observation(
    const anti_drone_interfaces::msg::DepthObservation & message)
  {
    if (!valid_depth_message(message) || !attitude_fresh()) {
      return;
    }
    const rclcpp::Time stamp(message.header.stamp);
    if (stamp.nanoseconds() <= 0 || message_age(stamp) > observation_timeout_s_) {
      return;
    }
    // Approved depth-camera mapping: body = [optical_y, optical_x, -optical_z].
    const Vector3 relative_body{
      message.position_optical.y,
      message.position_optical.x,
      -message.position_optical.z};
    const Vector3 relative_ned = rotate_body_to_ned(attitude_, relative_body);
    if (!finite(relative_ned)) {
      return;
    }
    const bool target_above = -relative_ned[2] >= minimum_target_height_above_m_;
    if (!target_above) {
      target_above_ = false;
      return;
    }
    if (track_id_valid_ && message.track_id != active_track_id_) {
      tracker_.reset_observation();
    }
    active_track_id_ = message.track_id;
    track_id_valid_ = true;
    if (tracker_.update_observation(
        {relative_ned[0], relative_ned[1]}, stamp.seconds()))
    {
      relative_position_ned_ = relative_ned;
      depth_update_ = stamp;
      target_above_ = true;
    }
  }

  void publish_offboard_mode()
  {
    px4_msgs::msg::OffboardControlMode output;
    output.timestamp = timestamp_us();
    output.velocity = true;
    offboard_pub_->publish(output);
  }

  double vertical_velocity_setpoint() const
  {
    if (!local_fresh() || !height_hold_set_) {
      return 0.0;
    }
    double requested = std::clamp(
      altitude_position_gain_ * (height_hold_z_ - local_position_[2]),
      -maximum_vertical_speed_mps_, maximum_vertical_speed_mps_);
    const double height_above_origin = height_above_task_origin_m(
      local_position_, local_origin_);
    if (height_above_origin < minimum_flight_height_m_ && requested > 0.0) {
      requested = -std::min(
        maximum_vertical_speed_mps_,
        altitude_position_gain_ * (minimum_flight_height_m_ - height_above_origin));
    }
    return requested;
  }

  void publish_trajectory_setpoint(const SimpleBelowTrackerResult & tracker_result)
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint output;
    output.timestamp = timestamp_us();
    output.position = {nan, nan, nan};
    output.velocity = {
      static_cast<float>(tracker_result.velocity_setpoint_ned_mps[0]),
      static_cast<float>(tracker_result.velocity_setpoint_ned_mps[1]),
      static_cast<float>(vertical_velocity_setpoint())};
    output.acceleration = {nan, nan, nan};
    output.jerk = {nan, nan, nan};
    output.yaw = static_cast<float>(yaw_hold_rad_);
    output.yawspeed = nan;
    trajectory_pub_->publish(output);
  }

  void publish_vehicle_command(std::uint32_t command, float param1, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand output;
    output.timestamp = timestamp_us();
    output.command = command;
    output.param1 = param1;
    output.param2 = param2;
    output.target_system = static_cast<std::uint8_t>(target_system_);
    output.target_component = 1;
    output.source_system = 255;
    output.source_component = 0;
    output.from_external = true;
    command_pub_->publish(output);
  }

  void manage_auto_engage()
  {
    if (!auto_engage_ || warmup_count_ < warmup_setpoint_count_) {
      return;
    }
    const bool preflight_ready = status_fresh() &&
      vehicle_status_.pre_flight_checks_pass && local_fresh() && attitude_fresh();
    if (!armed() && !preflight_ready) {
      preflight_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }
    if (!armed() && preflight_stable_since_.nanoseconds() == 0) {
      preflight_stable_since_ = now();
    }
    if (!armed() &&
      (now() - preflight_stable_since_).seconds() < preflight_stability_hold_s_)
    {
      return;
    }
    if (message_age(last_command_) <= 1.0) {
      return;
    }
    if (!offboard_active()) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      last_command_ = now();
    } else if (!armed()) {
      lock_task_origin("automatic arm command");
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      last_command_ = now();
    }
  }

  void publish_status(
    const SimpleBelowTrackerResult & tracker_result, bool observation_fresh,
    bool tracking_active)
  {
    px4_offboard_tracking::msg::SimpleBelowTrackerStatus output;
    output.header.stamp = now();
    output.header.frame_id = "map_ned";
    output.mode = to_string(mode_);
    output.control_enabled = control_enabled_;
    output.auto_engage = auto_engage_;
    output.setpoint_streaming = setpoint_streaming();
    output.offboard_session_seen = offboard_session_seen_;
    output.stopped_after_offboard_exit = stopped_after_offboard_exit_;
    output.local_state_valid = local_fresh();
    output.attitude_valid = attitude_fresh();
    output.observation_valid = observation_fresh;
    output.target_above = target_above_;
    output.takeoff_ready = takeoff_ready();
    output.offboard_active = offboard_active();
    output.armed = armed();
    output.tracking_active = tracking_active;
    output.observation_age_s = message_age(depth_update_);
    output.hold_height_m = height_hold_set_ ? local_origin_[2] - height_hold_z_ :
      std::numeric_limits<double>::quiet_NaN();
    output.relative_position_ned.x = relative_position_ned_[0];
    output.relative_position_ned.y = relative_position_ned_[1];
    output.relative_position_ned.z = relative_position_ned_[2];
    output.relative_velocity_ned.x = tracker_result.relative_velocity_ned_mps[0];
    output.relative_velocity_ned.y = tracker_result.relative_velocity_ned_mps[1];
    output.relative_velocity_ned.z = std::numeric_limits<double>::quiet_NaN();
    output.estimated_target_velocity_ned.x =
      tracker_result.estimated_target_velocity_ned_mps[0];
    output.estimated_target_velocity_ned.y =
      tracker_result.estimated_target_velocity_ned_mps[1];
    output.estimated_target_velocity_ned.z =
      std::numeric_limits<double>::quiet_NaN();
    output.predicted_relative_position_ned.x =
      tracker_result.predicted_relative_position_ned_m[0];
    output.predicted_relative_position_ned.y =
      tracker_result.predicted_relative_position_ned_m[1];
    output.predicted_relative_position_ned.z = relative_position_ned_[2];
    output.velocity_setpoint_ned.x = tracker_result.velocity_setpoint_ned_mps[0];
    output.velocity_setpoint_ned.y = tracker_result.velocity_setpoint_ned_mps[1];
    output.velocity_setpoint_ned.z = vertical_velocity_setpoint();
    status_pub_->publish(output);
  }

  void on_timer()
  {
    // The real-aircraft manual workflow starts only the PX4 Offboard heartbeat
    // on process startup. Once an Offboard session has been observed, leaving
    // Offboard disables the publisher before this timer can emit another
    // control sample.
    update_manual_offboard_session();

    const bool observation_fresh = target_above_ &&
      message_age(depth_update_) <= observation_timeout_s_;
    if (!observation_fresh && !observation_was_stale_) {
      tracker_.reset_observation();
      target_above_ = false;
      track_id_valid_ = false;
    }
    observation_was_stale_ = !observation_fresh;

    const bool vehicle_safe = local_fresh() && attitude_fresh() &&
      norm(local_velocity_) <= maximum_vehicle_speed_mps_;
    const bool flight_ready = flight_control_active() && vehicle_safe &&
      takeoff_ready() && armed() && offboard_active();
    const bool tracking_active = flight_ready && observation_fresh;
    if (publish_pursuer_takeoff_ready_ && flight_ready &&
      !pursuer_takeoff_ready_published_ && pursuer_takeoff_ready_pub_)
    {
      std_msgs::msg::Bool ready;
      ready.data = true;
      pursuer_takeoff_ready_pub_->publish(ready);
      pursuer_takeoff_ready_published_ = true;
      RCLCPP_INFO(
        get_logger(), "manual Offboard entered; target trajectory may start");
    }
    const auto tracker_result = tracker_.step(
      mode_, tracking_active, control_period_s_,
      {local_velocity_[0], local_velocity_[1]});
    if (heartbeat_streaming()) {
      publish_offboard_mode();
      if (warmup_count_ < warmup_setpoint_count_) {
        ++warmup_count_;
      }
      manage_auto_engage();
    }
    if (setpoint_streaming()) {
      publish_trajectory_setpoint(tracker_result);
    }
    publish_status(tracker_result, observation_fresh, tracking_active);
  }

  std::string topic_prefix_;
  std::string depth_topic_;
  std::string enable_topic_;
  std::string status_topic_;
  std::string task_origin_topic_;
  std::string common_origin_topic_;
  std::string pursuer_takeoff_ready_topic_;
  SimpleBelowTrackingMode mode_;
  int target_system_;
  double control_rate_hz_;
  double observation_timeout_s_;
  double state_timeout_s_;
  double status_timeout_s_;
  bool control_enabled_;
  bool auto_engage_;
  bool auto_disable_on_offboard_exit_;
  bool publish_pursuer_takeoff_ready_;
  bool takeoff_enabled_;
  double takeoff_height_m_;
  double takeoff_tolerance_m_;
  double takeoff_vertical_speed_tolerance_mps_;
  double altitude_position_gain_;
  double maximum_vertical_speed_mps_;
  double minimum_flight_height_m_;
  double minimum_target_height_above_m_;
  double maximum_tracking_range_m_;
  double maximum_vehicle_speed_mps_;
  std::size_t warmup_setpoint_count_;
  double preflight_stability_hold_s_;
  double control_period_s_{0.02};
  SimpleBelowTracker tracker_;
  Vector3 local_position_{};
  Vector3 local_position_at_global_{};
  Vector3 local_origin_{};
  Vector3 local_velocity_{};
  Vector3 relative_position_ned_{};
  Quaternion attitude_{};
  px4_msgs::msg::VehicleStatus vehicle_status_{};
  GeodeticPosition latest_global_position_{};
  GeodeticPosition common_geodetic_origin_{};
  bool local_valid_{false};
  bool attitude_valid_{false};
  bool vehicle_status_valid_{false};
  bool height_hold_set_{false};
  bool task_origin_locked_{false};
  bool global_valid_{false};
  bool local_at_global_valid_{false};
  bool common_origin_published_{false};
  bool local_reset_counters_initialized_{false};
  bool target_above_{false};
  bool track_id_valid_{false};
  bool observation_was_stale_{true};
  bool offboard_session_seen_{false};
  bool stopped_after_offboard_exit_{false};
  bool pursuer_takeoff_ready_published_{false};
  std::uint32_t active_track_id_{0};
  std::uint8_t local_xy_reset_counter_{};
  std::uint8_t local_z_reset_counter_{};
  double height_hold_z_{0.0};
  double yaw_hold_rad_{0.0};
  double current_heading_rad_{0.0};
  std::size_t warmup_count_{0};
  std::chrono::steady_clock::time_point local_update_{};
  std::chrono::steady_clock::time_point attitude_update_{};
  rclcpp::Time vehicle_status_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time depth_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time preflight_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::SimpleBelowTrackerStatus>::SharedPtr
    status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pursuer_takeoff_ready_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr task_origin_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr common_origin_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<anti_drone_interfaces::msg::DepthObservation>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::SimpleBelowTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
