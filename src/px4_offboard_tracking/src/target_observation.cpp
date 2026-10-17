#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "px4_offboard_tracking/camera_model.hpp"
#include "px4_offboard_tracking/msg/target_observation.hpp"
#include "px4_offboard_tracking/target_observation.hpp"

namespace px4_offboard_tracking
{

class TargetObservationNode : public rclcpp::Node
{
public:
  TargetObservationNode()
  : Node("target_observation"),
    relative_state_topic_(
      declare_parameter<std::string>("relative_state_topic", "/tracking/relative/state")),
    observation_topic_(
      declare_parameter<std::string>("observation_topic", "/tracking/camera/observation")),
    attitude_topic_(
      declare_parameter<std::string>(
        "attitude_topic", "/px4_1/fmu/out/vehicle_attitude")),
    common_frame_id_(declare_parameter<std::string>("common_frame_id", "map_ned")),
    camera_frame_id_(
      declare_parameter<std::string>("camera_frame_id", "pursuer/camera_optical")),
    state_timeout_s_(declare_parameter<double>("state_timeout_s", 0.5)),
    camera_rate_hz_(declare_parameter<double>("camera_rate_hz", 30.0)),
    observation_latency_s_(
      declare_parameter<bool>("enable_latency", false) ?
      declare_parameter<double>("latency_ms", 0.0) / 1000.0 :
      declare_parameter<double>("latency_ms", 0.0) * 0.0),
    enable_pixel_noise_(declare_parameter<bool>("enable_pixel_noise", false)),
    pixel_noise_stddev_px_(declare_parameter<double>("pixel_noise_stddev_px", 1.0)),
    enable_depth_noise_(declare_parameter<bool>("enable_depth_noise", false)),
    depth_noise_stddev_m_(declare_parameter<double>("depth_noise_stddev_m", 0.05)),
    enable_dropout_(declare_parameter<bool>("enable_dropout", false)),
    dropout_probability_(declare_parameter<double>("dropout_probability", 0.0)),
    enable_outliers_(declare_parameter<bool>("enable_outliers", false)),
    outlier_probability_(declare_parameter<double>("outlier_probability", 0.0)),
    outlier_min_depth_m_(declare_parameter<double>("outlier_min_depth_m", 1.0)),
    outlier_max_depth_m_(declare_parameter<double>("outlier_max_depth_m", 20.0)),
    random_seed_(declare_parameter<int>("random_seed", 1)),
    depth_camera_intrinsics_{
      declare_parameter<double>("camera_fx", 12847.019977569425),
      declare_parameter<double>("camera_fy", 6124.984365187126),
      declare_parameter<double>("camera_cx", 1919.5),
      declare_parameter<double>("camera_cy", 1079.5),
      static_cast<int>(declare_parameter<int>("camera_width", 3840)),
      static_cast<int>(declare_parameter<int>("camera_height", 2160)),
      declare_parameter<double>("camera_minimum_depth_m", 0.05)},
    search_camera_intrinsics_{
      declare_parameter<double>("search_camera_fx", 6089.462020537369),
      declare_parameter<double>("search_camera_fy", 1870.614872174388),
      declare_parameter<double>("search_camera_cx", 1919.5),
      declare_parameter<double>("search_camera_cy", 1079.5),
      depth_camera_intrinsics_.width,
      depth_camera_intrinsics_.height,
      depth_camera_intrinsics_.minimum_depth_m},
    depth_camera_model_(
      depth_camera_intrinsics_,
      CameraExtrinsics{
        Vector3d{
          declare_parameter<double>("camera_translation_body_x_m", 0.0),
          declare_parameter<double>("camera_translation_body_y_m", 0.0),
          declare_parameter<double>("camera_translation_body_z_m", 0.0)},
        Quaterniond{
          declare_parameter<double>("camera_q_body_to_optical_w", 0.0),
          declare_parameter<double>(
            "camera_q_body_to_optical_x", 0.70710678118654752440),
          declare_parameter<double>(
            "camera_q_body_to_optical_y", 0.70710678118654752440),
          declare_parameter<double>("camera_q_body_to_optical_z", 0.0)}},
      CameraFieldOfView{
        0.5 * declare_parameter<double>("depth_camera_horizontal_fov_rad", 0.296705973),
        0.5 * declare_parameter<double>("depth_camera_vertical_fov_rad", 0.349065850)}),
    search_camera_model_(
      search_camera_intrinsics_,
      CameraExtrinsics{
        Vector3d{
          declare_parameter<double>("search_camera_translation_body_x_m", 0.0),
          declare_parameter<double>("search_camera_translation_body_y_m", 0.0),
          declare_parameter<double>("search_camera_translation_body_z_m", 0.0)},
        Quaterniond{
          declare_parameter<double>(
            "search_camera_q_body_to_optical_w", -0.06162841671621935),
          declare_parameter<double>(
            "search_camera_q_body_to_optical_x", 0.7044160264027587),
          declare_parameter<double>(
            "search_camera_q_body_to_optical_y", 0.7044160264027587),
          declare_parameter<double>(
            "search_camera_q_body_to_optical_z", 0.06162841671621935)}},
      CameraFieldOfView{
        0.5 * declare_parameter<double>("search_camera_horizontal_fov_rad", 0.610865238),
        0.5 * declare_parameter<double>("search_camera_vertical_fov_rad", 1.047197551)}),
    observation_model_(
      depth_camera_intrinsics_,
      ObservationFaultConfig{
        enable_pixel_noise_ ? pixel_noise_stddev_px_ : 0.0,
        enable_depth_noise_ ? depth_noise_stddev_m_ : 0.0,
        enable_dropout_ ? dropout_probability_ : 0.0,
        enable_outliers_ ? outlier_probability_ : 0.0,
        outlier_min_depth_m_, outlier_max_depth_m_, static_cast<std::uint64_t>(random_seed_)})
  {
    if (state_timeout_s_ <= 0.0 || camera_rate_hz_ <= 0.0 || observation_latency_s_ < 0.0 ||
      random_seed_ < 0)
    {
      throw std::invalid_argument("camera rate/timeout must be positive and latency/seed non-negative");
    }

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      attitude_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
        q_body_to_ned_ = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        attitude_valid_ = std::isfinite(q_body_to_ned_.w) &&
          std::isfinite(q_body_to_ned_.x) && std::isfinite(q_body_to_ned_.y) &&
          std::isfinite(q_body_to_ned_.z);
        attitude_update_ = now();
      });
    relative_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      relative_state_topic_, 10,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        if (msg->header.frame_id != common_frame_id_) {
          relative_state_valid_ = false;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "rejected relative state outside common NED frame");
          return;
        }
        latest_relative_state_ = *msg;
        relative_state_valid_ = true;
      });
    observation_pub_ = create_publisher<px4_offboard_tracking::msg::TargetObservation>(
      observation_topic_, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / camera_rate_hz_));
    camera_timer_ = create_wall_timer(period, [this]() {on_camera_timer();});

    RCLCPP_INFO(
      get_logger(),
      "depth camera overlap FOV 17x20 deg and bearing-only search camera 35x60 deg at %.1f Hz, "
      "latency %.1f ms, depth observation=%s",
      camera_rate_hz_, observation_latency_s_ * 1000.0, observation_topic_.c_str());
  }

private:
  using ObservationMessage = px4_offboard_tracking::msg::TargetObservation;

  struct QueuedObservation
  {
    rclcpp::Time release_time;
    ObservationMessage message;
  };

  PinholeProjection ideal_projection() const
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    PinholeProjection invalid{{nan, nan, nan}, nan, nan, nan, nan, false};
    if (!relative_state_valid_ || !attitude_valid_) {return invalid;}

    const rclcpp::Time relative_time(latest_relative_state_.header.stamp);
    const double relative_age = (now() - relative_time).seconds();
    const double attitude_age = (now() - attitude_update_).seconds();
    if (relative_time.nanoseconds() <= 0 || relative_age < 0.0 ||
      relative_age > state_timeout_s_ || attitude_age < 0.0 || attitude_age > state_timeout_s_)
    {
      return invalid;
    }

    return depth_camera_model_.project(
      Vector3d{
        latest_relative_state_.pose.pose.position.x,
        latest_relative_state_.pose.pose.position.y,
        latest_relative_state_.pose.pose.position.z},
      q_body_to_ned_);
  }

  BearingObservation search_observation() const
  {
    if (!relative_state_valid_ || !attitude_valid_) {
      return {};
    }
    const rclcpp::Time relative_time(latest_relative_state_.header.stamp);
    const double relative_age = (now() - relative_time).seconds();
    const double attitude_age = (now() - attitude_update_).seconds();
    if (relative_time.nanoseconds() <= 0 || relative_age < 0.0 ||
      relative_age > state_timeout_s_ || attitude_age < 0.0 || attitude_age > state_timeout_s_)
    {
      return {};
    }
    return search_camera_model_.observe(
      Vector3d{
        latest_relative_state_.pose.pose.position.x,
        latest_relative_state_.pose.pose.position.y,
        latest_relative_state_.pose.pose.position.z},
      q_body_to_ned_);
  }

  void publish_ready_observations(const rclcpp::Time & current_time)
  {
    while (!delayed_observations_.empty() &&
      delayed_observations_.front().release_time <= current_time)
    {
      observation_pub_->publish(delayed_observations_.front().message);
      delayed_observations_.pop_front();
    }
  }

  void enqueue_or_publish(ObservationMessage && message, const rclcpp::Time & current_time)
  {
    if (observation_latency_s_ <= 0.0) {
      observation_pub_->publish(message);
      return;
    }
    delayed_observations_.push_back(
      {current_time + rclcpp::Duration::from_seconds(observation_latency_s_),
        std::move(message)});
  }

  void on_camera_timer()
  {
    const rclcpp::Time acquisition_time = now();
    publish_ready_observations(acquisition_time);

    const auto search = search_observation();
    if (search.visible != search_visible_) {
      search_visible_ = search.visible;
      RCLCPP_INFO(
        get_logger(), "bearing-only search camera target %s",
        search_visible_ ? "acquired" : "lost");
    }

    const auto sample = observation_model_.apply(ideal_projection());
    if (!sample.has_value()) {
      return;  // A dropped camera frame publishes no message.
    }

    ObservationMessage out;
    out.header.stamp = acquisition_time;
    out.header.frame_id = camera_frame_id_;
    out.position_camera.x = sample->position_camera.x;
    out.position_camera.y = sample->position_camera.y;
    out.position_camera.z = sample->position_camera.z;
    out.pixel_u = sample->pixel_u;
    out.pixel_v = sample->pixel_v;
    out.depth = sample->depth;
    out.visible = sample->visible;
    out.outlier_injected = sample->outlier_injected;
    enqueue_or_publish(std::move(out), acquisition_time);
  }

  std::string relative_state_topic_;
  std::string observation_topic_;
  std::string attitude_topic_;
  std::string common_frame_id_;
  std::string camera_frame_id_;
  double state_timeout_s_;
  double camera_rate_hz_;
  double observation_latency_s_;
  bool enable_pixel_noise_;
  double pixel_noise_stddev_px_;
  bool enable_depth_noise_;
  double depth_noise_stddev_m_;
  bool enable_dropout_;
  double dropout_probability_;
  bool enable_outliers_;
  double outlier_probability_;
  double outlier_min_depth_m_;
  double outlier_max_depth_m_;
  std::int64_t random_seed_;
  CameraIntrinsics depth_camera_intrinsics_;
  CameraIntrinsics search_camera_intrinsics_;
  CameraModel depth_camera_model_;
  BearingCameraModel search_camera_model_;
  TargetObservationModel observation_model_;
  Quaterniond q_body_to_ned_{};
  nav_msgs::msg::Odometry latest_relative_state_{};
  bool relative_state_valid_{false};
  bool attitude_valid_{false};
  bool search_visible_{false};
  rclcpp::Time attitude_update_{0, 0, RCL_ROS_TIME};
  std::deque<QueuedObservation> delayed_observations_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr relative_state_sub_;
  rclcpp::Publisher<ObservationMessage>::SharedPtr observation_pub_;
  rclcpp::TimerBase::SharedPtr camera_timer_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::TargetObservationNode>());
  rclcpp::shutdown();
  return 0;
}
