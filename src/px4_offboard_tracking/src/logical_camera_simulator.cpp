#include <anti_drone_interfaces/msg/depth_observation.hpp>
#include <anti_drone_interfaces/msg/search_observation.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "px4_offboard_tracking/camera_model.hpp"
#include "px4_offboard_tracking/sample_time_sync.hpp"
#include "px4_offboard_tracking/target_observation.hpp"

namespace px4_offboard_tracking
{
namespace
{

enum class LogicalCameraOutput
{
  SEARCH,
  DEPTH
};

LogicalCameraOutput parse_camera_output(const std::string & value)
{
  if (value == "search") {
    return LogicalCameraOutput::SEARCH;
  }
  if (value == "depth") {
    return LogicalCameraOutput::DEPTH;
  }
  throw std::invalid_argument("camera_output must be 'search' or 'depth'");
}

const char * camera_output_name(LogicalCameraOutput output)
{
  return output == LogicalCameraOutput::SEARCH ? "search" : "depth";
}

}  // namespace

class LogicalCameraSimulator : public rclcpp::Node
{
public:
  LogicalCameraSimulator()
  : Node("logical_camera_simulator"),
    camera_output_(parse_camera_output(
      declare_parameter<std::string>("camera_output", "depth"))),
    truth_topic_(declare_parameter<std::string>(
      "truth_topic", "/evaluation/relative_truth")),
    attitude_topic_(declare_parameter<std::string>(
      "attitude_topic", "/px4_1/fmu/out/vehicle_attitude")),
    search_topic_(declare_parameter<std::string>(
      "search_topic", "/perception/search/observation")),
    depth_topic_(declare_parameter<std::string>(
      "depth_topic", "/perception/depth/observation")),
    search_frame_id_(declare_parameter<std::string>(
      "search_frame_id", "pursuer/search_camera_optical")),
    depth_frame_id_(declare_parameter<std::string>(
      "depth_frame_id", "pursuer/depth_camera_optical")),
    maximum_state_age_s_(declare_parameter<double>("maximum_state_age_s", 0.25)),
    camera_rate_hz_(declare_parameter<double>("camera_rate_hz", 30.0)),
    pixel_stddev_px_(declare_parameter<double>("pixel_noise_stddev_px", 0.0)),
    depth_stddev_m_(declare_parameter<double>("depth_noise_stddev_m", 0.0)),
    depth_intrinsics_{
      declare_parameter<double>("depth_camera_fx", 12847.019977569425),
      declare_parameter<double>("depth_camera_fy", 6124.984365187126),
      declare_parameter<double>("depth_camera_cx", 1919.5),
      declare_parameter<double>("depth_camera_cy", 1079.5),
      static_cast<int>(declare_parameter<int>("depth_camera_width", 3840)),
      static_cast<int>(declare_parameter<int>("depth_camera_height", 2160)),
      declare_parameter<double>("depth_camera_minimum_depth_m", 0.05)},
    depth_camera_maximum_depth_m_(
      declare_parameter<double>("depth_camera_maximum_depth_m", 1e9)),
    depth_reliable_max_m_(
      declare_parameter<double>("depth_reliable_max_m", 15.0)),
    depth_valid_max_m_(
      declare_parameter<double>("depth_valid_max_m", -1.0)),
    search_intrinsics_{
      declare_parameter<double>("search_camera_fx", 6088.643),
      declare_parameter<double>("search_camera_fy", 1870.615),
      declare_parameter<double>("search_camera_cx", 1919.5),
      declare_parameter<double>("search_camera_cy", 1079.5),
      static_cast<int>(declare_parameter<int>("search_camera_width", 3840)),
      static_cast<int>(declare_parameter<int>("search_camera_height", 2160)),
      declare_parameter<double>("search_camera_minimum_depth_m", 0.05)},
    depth_camera_(
      depth_intrinsics_,
      CameraExtrinsics{{0.0, 0.0, 0.0}, CameraModel::default_upward_optical_quaternion()},
      CameraFieldOfView{
        declare_parameter<double>("depth_horizontal_half_fov_rad", 0.148352986),
        declare_parameter<double>("depth_vertical_half_fov_rad", 0.174532925)}),
    search_camera_(
      search_intrinsics_,
      CameraExtrinsics{{0.0, 0.0, 0.0}, CameraModel::default_search_optical_quaternion()},
      CameraFieldOfView{
        declare_parameter<double>("search_horizontal_half_fov_rad", 0.305432619),
        declare_parameter<double>("search_vertical_half_fov_rad", 0.523598776)}),
    attitude_buffer_(
      declare_parameter<double>("attitude_buffer_s", 2.0),
      declare_parameter<double>("attitude_sync_tolerance_s", 0.05)),
    depth_fault_model_(
      depth_intrinsics_,
      ObservationFaultConfig{
        pixel_stddev_px_, depth_stddev_m_,
        declare_parameter<double>("depth_dropout_probability", 0.0),
        declare_parameter<double>("depth_outlier_probability", 0.0),
        declare_parameter<double>("depth_outlier_min_m", 1.0),
        declare_parameter<double>("depth_outlier_max_m", 100.0),
        static_cast<std::uint64_t>(declare_parameter<int>("random_seed", 1))})
  {
    if (maximum_state_age_s_ <= 0.0 || camera_rate_hz_ <= 0.0 ||
      pixel_stddev_px_ < 0.0 || depth_stddev_m_ < 0.0)
    {
      throw std::invalid_argument("invalid logical camera simulator configuration");
    }
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_topic_, 10,
      [this](nav_msgs::msg::Odometry::UniquePtr message) {
        truth_ = *message;
        truth_update_ = now();
      });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      attitude_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr message) {
        const auto receipt_time = now();
        const Quaterniond attitude{
          message->q[0], message->q[1], message->q[2], message->q[3]};
        // simulation_truth_bridge stamps synthetic truth in the ROS clock
        // domain.  Gazebo GUI stalls can let PX4 sample time lag ROS receipt
        // time by more than a second, so mixing that PX4 timestamp with the
        // ROS-stamped truth creates long false camera dropouts.  Receipt time
        // keeps both simulator inputs in the same clock domain.  The real
        // perception/estimator path retains sensor-sample-time synchronization.
        attitude_buffer_.add(receipt_time.seconds(), attitude);
      });
    const auto camera_qos = rclcpp::SensorDataQoS();
    if (camera_output_ == LogicalCameraOutput::SEARCH) {
      search_pub_ = create_publisher<anti_drone_interfaces::msg::SearchObservation>(
        search_topic_, camera_qos);
    } else {
      depth_pub_ = create_publisher<anti_drone_interfaces::msg::DepthObservation>(
        depth_topic_, camera_qos);
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / camera_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {publish_frame();});
    RCLCPP_INFO(
      get_logger(), "logical %s camera ready: rate=%.1f Hz, truth=%s, attitude=%s",
      camera_output_name(camera_output_), camera_rate_hz_,
      truth_topic_.c_str(), attitude_topic_.c_str());
  }

private:
  bool fresh(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() > 0 && (now() - stamp).seconds() >= 0.0 &&
           (now() - stamp).seconds() <= maximum_state_age_s_;
  }

  Vector3d relative_position() const
  {
    return {
      truth_.pose.pose.position.x,
      truth_.pose.pose.position.y,
      truth_.pose.pose.position.z};
  }

  void publish_search(
    const BearingObservation & observation, const rclcpp::Time & acquisition_time)
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const bool measurement_valid =
      observation.visible &&
      std::isfinite(observation.pixel_u) &&
      std::isfinite(observation.pixel_v) &&
      std::isfinite(observation.horizontal_angle_rad) &&
      std::isfinite(observation.vertical_angle_rad) &&
      std::isfinite(observation.unit_line_of_sight_optical.x) &&
      std::isfinite(observation.unit_line_of_sight_optical.y) &&
      std::isfinite(observation.unit_line_of_sight_optical.z);
    anti_drone_interfaces::msg::SearchObservation output;
    output.interface_version =
      anti_drone_interfaces::msg::SearchObservation::INTERFACE_VERSION_V1;
    output.header.stamp = acquisition_time;
    output.header.frame_id = search_frame_id_;
    output.sequence = sequence_;
    output.track_id = measurement_valid ? 1U : 0U;
    output.visible = measurement_valid;
    output.confidence = measurement_valid ? 1.0F : 0.0F;
    output.image_width = static_cast<std::uint32_t>(search_intrinsics_.width);
    output.image_height = static_cast<std::uint32_t>(search_intrinsics_.height);
    output.pixel_u = measurement_valid ? observation.pixel_u : nan;
    output.pixel_v = measurement_valid ? observation.pixel_v : nan;
    output.bounding_box_valid = false;
    output.bounding_box_width_px = 0.0;
    output.bounding_box_height_px = 0.0;
    output.angles_valid = measurement_valid;
    output.horizontal_angle_rad =
      measurement_valid ? observation.horizontal_angle_rad : nan;
    output.vertical_angle_rad =
      measurement_valid ? observation.vertical_angle_rad : nan;
    const double search_angular_sigma = std::max(
      1e-6, std::max(0.5, pixel_stddev_px_) /
      std::min(search_intrinsics_.fx, search_intrinsics_.fy));
    output.angle_covariance = measurement_valid ?
      std::array<double, 4>{
      search_angular_sigma * search_angular_sigma, 0.0,
      0.0, search_angular_sigma * search_angular_sigma} :
      std::array<double, 4>{nan, nan, nan, nan};
    output.unit_los_valid = measurement_valid;
    output.unit_los_optical.x =
      measurement_valid ? observation.unit_line_of_sight_optical.x : nan;
    output.unit_los_optical.y =
      measurement_valid ? observation.unit_line_of_sight_optical.y : nan;
    output.unit_los_optical.z =
      measurement_valid ? observation.unit_line_of_sight_optical.z : nan;
    search_pub_->publish(output);
  }

  void publish_depth(
    const PinholeProjection & ideal, const rclcpp::Time & acquisition_time)
  {
    const auto observation = depth_fault_model_.apply(ideal);
    if (!observation.has_value()) {
      return;
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // 深度模型（2026-10-13 用户定案，替代三区/硬截断）：实机深度来自
    // YOLO 识别 + 自研估深（非 SDK）——可靠距离内准确，之外可测但精度
    // 随距离下降。无概率丢弃、无硬截断：噪声与协方差按 (d/reliable)²
    // 连续增长（双目视差误差∝距离²），IMM 依协方差自然降权。
    const bool within_fov =
      observation->visible &&
      std::isfinite(observation->pixel_u) &&
      std::isfinite(observation->pixel_v) &&
      std::isfinite(observation->position_camera.x) &&
      std::isfinite(observation->position_camera.y) &&
      std::isfinite(observation->position_camera.z) &&
      std::isfinite(observation->depth) &&
      observation->depth > depth_intrinsics_.minimum_depth_m;
    double depth_noise_mult = 1.0;
    double effective_depth = observation->depth;
    if (within_fov && observation->depth > depth_reliable_max_m_) {
      const double ratio = observation->depth / depth_reliable_max_m_;
      depth_noise_mult = ratio * ratio;
      std::normal_distribution<double> noise(
        0.0, depth_stddev_m_ * (depth_noise_mult - 1.0));
      // observation 为 const（fault model 返回只读样本）：噪声作用于
      // 深度副本，位置按比例缩放保持几何一致
      effective_depth = observation->depth + noise(rng_);
    }
    const double depth_scale =
      within_fov && observation->depth > 1e-6 ? effective_depth / observation->depth : 1.0;
    const bool measurement_valid = within_fov;
    anti_drone_interfaces::msg::DepthObservation output;
    output.interface_version =
      anti_drone_interfaces::msg::DepthObservation::INTERFACE_VERSION_V1;
    output.header.stamp = acquisition_time;
    output.header.frame_id = depth_frame_id_;
    output.sequence = sequence_;
    output.track_id = within_fov ? 1U : 0U;
    output.visible = within_fov;
    output.confidence = within_fov ? 1.0F : 0.0F;
    output.image_width = static_cast<std::uint32_t>(depth_intrinsics_.width);
    output.image_height = static_cast<std::uint32_t>(depth_intrinsics_.height);
    output.pixel_u = within_fov ? observation->pixel_u : nan;
    output.pixel_v = within_fov ? observation->pixel_v : nan;
    output.bounding_box_valid = false;
    output.bounding_box_width_px = 0.0;
    output.bounding_box_height_px = 0.0;
    output.angles_valid = within_fov;
    output.depth_z_m = measurement_valid ? effective_depth : nan;
    output.position_optical.x = measurement_valid ?
      observation->position_camera.x * depth_scale : nan;
    output.position_optical.y = measurement_valid ?
      observation->position_camera.y * depth_scale : nan;
    output.position_optical.z = measurement_valid ?
      observation->position_camera.z * depth_scale : nan;
    output.range_m = measurement_valid ? std::sqrt(
      observation->position_camera.x * observation->position_camera.x +
      observation->position_camera.y * observation->position_camera.y +
      observation->position_camera.z * observation->position_camera.z) : nan;
    output.horizontal_angle_rad = within_fov ? std::atan2(
      observation->position_camera.x, observation->position_camera.z) : nan;
    output.vertical_angle_rad = within_fov ? std::atan2(
      observation->position_camera.y, observation->position_camera.z) : nan;
    const double depth_angular_sigma = std::max(
      1e-6, std::max(0.5, pixel_stddev_px_) /
      std::min(depth_intrinsics_.fx, depth_intrinsics_.fy));
    output.angle_covariance = measurement_valid ?
      std::array<double, 4>{
      depth_angular_sigma * depth_angular_sigma, 0.0,
      0.0, depth_angular_sigma * depth_angular_sigma} :
      std::array<double, 4>{nan, nan, nan, nan};
    // 检测/深度分离区（2026-10-14）：>depth_valid_max_m 时 depth_valid=
    // false 而 visible/像素保留（复现实机 YOLO 检测在、估深无输出的
    // bearing-only 情形；默认 -1 = 关闭，维持 2026-10-13 定案行为）
    output.depth_valid = measurement_valid &&
      (depth_valid_max_m_ < 0.0 || observation->depth <= depth_valid_max_m_);
    const double sigma_x = std::max(
      1e-3, std::abs(observation->depth / depth_intrinsics_.fx) *
      std::max(0.5, pixel_stddev_px_));
    const double sigma_y = std::max(
      1e-3, std::abs(observation->depth / depth_intrinsics_.fy) *
      std::max(0.5, pixel_stddev_px_));
    const double sigma_z = std::max(1e-3, std::max(0.02, depth_stddev_m_) * depth_noise_mult);
    output.position_covariance = measurement_valid ?
      std::array<double, 9>{
      sigma_x * sigma_x, 0.0, 0.0,
      0.0, sigma_y * sigma_y, 0.0,
      0.0, 0.0, sigma_z * sigma_z} :
      std::array<double, 9>{nan, nan, nan, nan, nan, nan, nan, nan, nan};
    depth_pub_->publish(output);
  }

  void publish_frame()
  {
    if (!fresh(truth_update_)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "logical camera gated: truth not fresh (age=%.2fs)",
        (now() - truth_update_).seconds());
      return;
    }
    const rclcpp::Time acquisition_time(truth_.header.stamp);
    if (acquisition_time.nanoseconds() <= 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "logical camera gated: bad truth stamp");
      return;
    }
    Quaterniond attitude;
    if (!attitude_buffer_.sample(acquisition_time.seconds(), attitude)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "logical camera gated: attitude sample miss at t=%.3f", acquisition_time.seconds());
      return;
    }
    ++sequence_;
    const auto relative = relative_position();
    if (camera_output_ == LogicalCameraOutput::SEARCH) {
      publish_search(search_camera_.observe(relative, attitude), acquisition_time);
    } else {
      publish_depth(depth_camera_.project(relative, attitude), acquisition_time);
    }
  }

  LogicalCameraOutput camera_output_;
  std::string truth_topic_;
  std::string attitude_topic_;
  std::string search_topic_;
  std::string depth_topic_;
  std::string search_frame_id_;
  std::string depth_frame_id_;
  double maximum_state_age_s_;
  double camera_rate_hz_;
  double pixel_stddev_px_;
  double depth_stddev_m_;
  CameraIntrinsics depth_intrinsics_;
  double depth_camera_maximum_depth_m_{1e9};
  double depth_reliable_max_m_{15.0};
  double depth_valid_max_m_{-1.0};
  std::mt19937 rng_{std::random_device{}()};
  CameraIntrinsics search_intrinsics_;
  CameraModel depth_camera_;
  BearingCameraModel search_camera_;
  SampleTimeAttitudeBuffer attitude_buffer_;
  TargetObservationModel depth_fault_model_;
  nav_msgs::msg::Odometry truth_{};
  std::uint32_t sequence_{0};
  rclcpp::Time truth_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<anti_drone_interfaces::msg::SearchObservation>::SharedPtr search_pub_;
  rclcpp::Publisher<anti_drone_interfaces::msg::DepthObservation>::SharedPtr depth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::LogicalCameraSimulator>());
  rclcpp::shutdown();
  return 0;
}
