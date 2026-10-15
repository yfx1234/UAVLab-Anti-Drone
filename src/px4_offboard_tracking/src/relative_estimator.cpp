#include <anti_drone_interfaces/msg/depth_observation.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "px4_offboard_tracking/camera_model.hpp"
#include "px4_offboard_tracking/msg/relative_estimate.hpp"
#include "px4_offboard_tracking/relative_ekf.hpp"
#include "px4_offboard_tracking/sample_time_sync.hpp"

namespace px4_offboard_tracking
{

class RelativeEstimatorNode : public rclcpp::Node
{
public:
  RelativeEstimatorNode()
  : Node("relative_estimator"),
    observation_topic_(
      declare_parameter<std::string>(
        "observation_topic", "/perception/depth/observation")),
    estimate_topic_(
      declare_parameter<std::string>("estimate_topic", "/tracking/relative/estimate")),
    attitude_topic_(declare_parameter<std::string>(
      "attitude_topic", "/px4_1/fmu/out/vehicle_attitude")),
    common_frame_id_(declare_parameter<std::string>("common_frame_id", "map_ned")),
    estimator_rate_hz_(declare_parameter<double>("estimator_rate_hz", 50.0)),
    max_data_age_s_(declare_parameter<double>("max_data_age_s", 0.5)),
    max_position_variance_(declare_parameter<double>("max_position_variance", 25.0)),
    attitude_sync_tolerance_s_(
      declare_parameter<double>("attitude_sync_tolerance_s", 0.05)),
    attitude_buffer_s_(declare_parameter<double>("attitude_buffer_s", 2.0)),
    attitude_clock_domain_tolerance_s_(
      declare_parameter<double>("attitude_clock_domain_tolerance_s", 5.0)),
    pixel_stddev_px_(declare_parameter<double>("measurement_pixel_stddev_px", 1.5)),
    depth_stddev_m_(declare_parameter<double>("measurement_depth_stddev_m", 0.10)),
    camera_intrinsics_{
      declare_parameter<double>("camera_fx", 12847.019977569425),
      declare_parameter<double>("camera_fy", 6124.984365187126),
      declare_parameter<double>("camera_cx", 1919.5),
      declare_parameter<double>("camera_cy", 1079.5),
      static_cast<int>(declare_parameter<int>("camera_width", 3840)),
      static_cast<int>(declare_parameter<int>("camera_height", 2160)),
      declare_parameter<double>("camera_minimum_depth_m", 0.05)},
    camera_model_(
      camera_intrinsics_,
      CameraExtrinsics{
        Vector3d{
          declare_parameter<double>("camera_translation_body_x_m", 0.0),
          declare_parameter<double>("camera_translation_body_y_m", 0.0),
          declare_parameter<double>("camera_translation_body_z_m", 0.0)},
        Quaterniond{
          declare_parameter<double>("camera_q_body_to_optical_w", 0.0),
          declare_parameter<double>("camera_q_body_to_optical_x", 1.0),
          declare_parameter<double>("camera_q_body_to_optical_y", 0.0),
          declare_parameter<double>("camera_q_body_to_optical_z", 0.0)}}),
    attitude_buffer_(attitude_buffer_s_, attitude_sync_tolerance_s_),
    filter_(RelativeImmEkfConfig{RelativeEkfConfig{
      declare_parameter<double>("acceleration_noise_stddev_mps2", 2.0),
      declare_parameter<double>("initial_position_stddev_m", 1.0),
      declare_parameter<double>("initial_velocity_stddev_mps", 3.0),
      // 99.9% chi-square gate for a three-dimensional observation. This is
      // the same gate validated by the P3 IMM maneuver tests; the former 99%
      // gate rejected long bursts of otherwise consistent measurements when
      // a 5 m/s target entered a sustained turn.
      declare_parameter<double>("innovation_gate_mahalanobis", 16.266),
      declare_parameter<double>("initial_acceleration_stddev_mps2", 3.0),
      declare_parameter<double>("turn_rate_process_noise_stddev_rads", 0.5),
      declare_parameter<double>("initial_turn_rate_stddev_rads", 0.3)},
      declare_parameter<double>("imm_probability_cv_to_cv", 0.97),
      declare_parameter<double>("imm_probability_ca_to_ca", 0.95),
      declare_parameter<double>("imm_probability_ct_to_ct", 0.95),
      declare_parameter<double>("imm_initial_cv_probability", 0.6),
      declare_parameter<double>("imm_initial_ct_probability", 0.15)})
  {
    // 纯方位量测（2026-10-14）：实机 YOLO 检测到但自研估深无输出
    //（visible=true、depth_valid=false）时退化为 bearing-only 更新。
    bearing_only_enabled_ = declare_parameter<bool>("bearing_only_update_enabled", true);
    bearing_only_nominal_range_m_ =
      declare_parameter<double>("bearing_only_nominal_range_m", 12.0);
    bearing_only_range_sigma_m_ =
      declare_parameter<double>("bearing_only_range_sigma_m", 30.0);
    // 发散恢复阈值（2026-10-15 改秒单位）：距上次滤波器有效更新超过
    // 该秒数（观测持续到达但全部被拒——预门无效/姿态 miss/革新门任一
    // 层）则强制重置，下一条有效观测重新初始化。秒单位与相机帧率无关
    // （旧 45 帧@30Hz≈1.5s、15Hz 传 22 的帧率换算随之废除）；观测断流
    // 不触发——目标短暂出视场 coast 是合法状态（BEARING_HOLD 依赖）
    divergence_reset_period_s_ =
      declare_parameter<double>("divergence_reset_period_s", 1.5);
    if (estimator_rate_hz_ <= 0.0 || max_data_age_s_ <= 0.0 ||
      max_position_variance_ <= 0.0 || attitude_sync_tolerance_s_ <= 0.0 ||
      attitude_buffer_s_ <= attitude_sync_tolerance_s_ || pixel_stddev_px_ <= 0.0 ||
      depth_stddev_m_ <= 0.0 || attitude_clock_domain_tolerance_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid relative estimator timing/noise configuration");
    }

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_qos.best_effort().durability_volatile();
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      attitude_topic_, px4_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr msg) {on_attitude(*msg);});
    observation_sub_ = create_subscription<anti_drone_interfaces::msg::DepthObservation>(
      observation_topic_, rclcpp::SensorDataQoS(),
      [this](anti_drone_interfaces::msg::DepthObservation::UniquePtr msg) {
        on_observation(*msg);
      });
    estimate_pub_ = create_publisher<px4_offboard_tracking::msg::RelativeEstimate>(
      estimate_topic_, 10);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / estimator_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {publish_estimate();});
  }

private:
  void on_attitude(const px4_msgs::msg::VehicleAttitude & msg)
  {
    const Quaterniond quaternion{msg.q[0], msg.q[1], msg.q[2], msg.q[3]};
    if (!std::isfinite(quaternion.w) || !std::isfinite(quaternion.x) ||
      !std::isfinite(quaternion.y) || !std::isfinite(quaternion.z))
    {
      return;
    }
    const double receipt_time_s = now().seconds();
    const double sample_time_s = SampleTimeAttitudeBuffer::px4_sample_or_receipt_time(
      msg.timestamp, msg.timestamp_sample, receipt_time_s,
      attitude_clock_domain_tolerance_s_);
    attitude_buffer_.add(sample_time_s, quaternion);
  }

  bool attitude_at(const rclcpp::Time & timestamp, Quaterniond & output) const
  {
    return attitude_buffer_.sample(timestamp.seconds(), output);
  }

  std::array<double, 9> measurement_covariance_ned(
    const anti_drone_interfaces::msg::DepthObservation & msg,
    const Quaterniond & q_body_to_ned) const
  {
    Eigen::Matrix3d camera_covariance;
    bool supplied_covariance_valid = true;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        const double value = msg.position_covariance[3 * row + column];
        camera_covariance(row, column) = value;
        supplied_covariance_valid = supplied_covariance_valid && std::isfinite(value);
      }
    }
    supplied_covariance_valid = supplied_covariance_valid &&
      camera_covariance.diagonal().minCoeff() > 0.0;
    if (!supplied_covariance_valid) {
      const double normalized_x =
        (msg.pixel_u - camera_intrinsics_.cx) / camera_intrinsics_.fx;
      const double normalized_y =
        (msg.pixel_v - camera_intrinsics_.cy) / camera_intrinsics_.fy;
      Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
      jacobian(0, 0) = msg.depth_z_m / camera_intrinsics_.fx;
      jacobian(0, 2) = normalized_x;
      jacobian(1, 1) = msg.depth_z_m / camera_intrinsics_.fy;
      jacobian(1, 2) = normalized_y;
      jacobian(2, 2) = 1.0;
      Eigen::Matrix3d sensor_covariance = Eigen::Matrix3d::Zero();
      sensor_covariance(0, 0) = pixel_stddev_px_ * pixel_stddev_px_;
      sensor_covariance(1, 1) = pixel_stddev_px_ * pixel_stddev_px_;
      sensor_covariance(2, 2) = depth_stddev_m_ * depth_stddev_m_;
      camera_covariance = jacobian * sensor_covariance * jacobian.transpose();
    }

    const auto q_camera_to_ned = camera_model_.q_camera_to_ned(q_body_to_ned);
    const Eigen::Quaterniond eigen_quaternion(
      q_camera_to_ned.w, q_camera_to_ned.x, q_camera_to_ned.y, q_camera_to_ned.z);
    const Eigen::Matrix3d rotation = eigen_quaternion.normalized().toRotationMatrix();
    const Eigen::Matrix3d ned_covariance = rotation * camera_covariance * rotation.transpose();
    std::array<double, 9> output{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        output[3 * row + column] = ned_covariance(row, column);
      }
      output[3 * row + row] = std::max(output[3 * row + row], 1e-6);
    }
    return output;
  }

  void on_observation(const anti_drone_interfaces::msg::DepthObservation & msg)
  {
    ++diag_obs_received_;
    // bearing-only：检测在而深度不在。旧逻辑整条拒收（连方向都不用，
    // 2026-10-14 放宽）——深度路径条件仅在非 bearing-only 时施加；
    // bearing_only_update_enabled=false 时行为与旧版逐位一致。
    const bool bearing_only_msg = bearing_only_enabled_ && msg.visible &&
      !msg.depth_valid &&
      std::isfinite(msg.pixel_u) && std::isfinite(msg.pixel_v);
    if (msg.interface_version !=
      anti_drone_interfaces::msg::DepthObservation::INTERFACE_VERSION_V1 ||
      !msg.visible ||
      !std::isfinite(msg.pixel_u) || !std::isfinite(msg.pixel_v) ||
      (!bearing_only_msg &&
        (!msg.depth_valid ||
        !std::isfinite(msg.position_optical.x) ||
        !std::isfinite(msg.position_optical.y) ||
        !std::isfinite(msg.position_optical.z) ||
        !std::isfinite(msg.depth_z_m) ||
        msg.depth_z_m <= camera_intrinsics_.minimum_depth_m)))
    {
      ++diag_rejected_invalid_;
      consider_divergence_reset();
      return;
    }
    const rclcpp::Time timestamp(msg.header.stamp);
    Quaterniond q_body_to_ned;
    if (timestamp.nanoseconds() <= 0 || !attitude_at(timestamp, q_body_to_ned)) {
      ++diag_rejected_attitude_;
      consider_divergence_reset();
      return;
    }

    Vector3d relative_ned;
    std::array<double, 9> covariance_ned{};
    if (bearing_only_msg) {
      ++diag_bearing_only_;
      // 像素 → optical 单位视线 → NED。名义距离 = 滤波器预测在视线
      // 上的投影（未初始化用参数先验）；协方差在 optical 系取
      // 横向 sigma=角sigma*距离、距离 sigma=大参数值 → 旋转到 NED 后
      // 更新只修正横向、距离通道几乎不动（经典 bearing-only 伪量测）。
      const double nx =
        (msg.pixel_u - camera_intrinsics_.cx) / camera_intrinsics_.fx;
      const double ny =
        (msg.pixel_v - camera_intrinsics_.cy) / camera_intrinsics_.fy;
      Vector3d los_optical{nx, ny, 1.0};
      const double los_norm = std::sqrt(nx * nx + ny * ny + 1.0);
      los_optical.x /= los_norm;
      los_optical.y /= los_norm;
      los_optical.z /= los_norm;
      double nominal_range = bearing_only_nominal_range_m_;
      const auto q_camera_to_ned = camera_model_.q_camera_to_ned(q_body_to_ned);
      const Eigen::Quaterniond q_camera_eigen(
        q_camera_to_ned.w, q_camera_to_ned.x, q_camera_to_ned.y,
        q_camera_to_ned.z);
      const Eigen::Matrix3d rotation_camera_to_ned =
        q_camera_eigen.normalized().toRotationMatrix();
      const Eigen::Vector3d los_ned = rotation_camera_to_ned *
        Eigen::Vector3d(los_optical.x, los_optical.y, los_optical.z);
      const auto predicted = filter_.predicted_state(timestamp.seconds());
      const Eigen::Vector3d predicted_position(
        predicted.position[0], predicted.position[1], predicted.position[2]);
      const double projection = los_ned.dot(predicted_position);
      if (std::isfinite(projection) && projection > 1.0) {
        nominal_range = projection;
      }
      relative_ned = camera_model_.camera_position_to_relative_ned(
        Vector3d{
          los_optical.x * nominal_range, los_optical.y * nominal_range,
          los_optical.z * nominal_range},
        q_body_to_ned);
      const double angular_sigma = std::max(0.5, pixel_stddev_px_) /
        std::min(camera_intrinsics_.fx, camera_intrinsics_.fy);
      const double lateral_sigma = std::max(1e-3, angular_sigma * nominal_range);
      Eigen::Matrix3d camera_covariance = Eigen::Matrix3d::Zero();
      camera_covariance(0, 0) = lateral_sigma * lateral_sigma;
      camera_covariance(1, 1) = lateral_sigma * lateral_sigma;
      camera_covariance(2, 2) =
        bearing_only_range_sigma_m_ * bearing_only_range_sigma_m_;
      const Eigen::Matrix3d ned_covariance =
        rotation_camera_to_ned * camera_covariance *
        rotation_camera_to_ned.transpose();
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          covariance_ned[3 * row + column] = ned_covariance(row, column);
        }
        covariance_ned[3 * row + row] =
          std::max(covariance_ned[3 * row + row], 1e-6);
      }
    } else {
      relative_ned = camera_model_.camera_position_to_relative_ned(
        Vector3d{msg.position_optical.x, msg.position_optical.y, msg.position_optical.z},
        q_body_to_ned);
      covariance_ned = measurement_covariance_ned(msg, q_body_to_ned);
    }
    last_measurement_accepted_ = filter_.update(
      {relative_ned.x, relative_ned.y, relative_ned.z},
      covariance_ned, timestamp.seconds());
    if (last_measurement_accepted_) {
      ++diag_updates_accepted_;
      last_filter_update_s_ = now().seconds();
    } else {
      // 发散恢复：距上次有效更新超阈值（默认 1.5s）强制重置，下一条
      // 有效观测重新初始化（快速交叉下 IMM 速度爆掉后革新门死锁，
      // 2026-10-12 迎敌探针实测 coast 速度 ~100 m/s；2026-10-15 起
      // 预门无效/姿态 miss 也计入"无更新"）
      consider_divergence_reset();
    }
  }

  // 观测到达但滤波器未得到更新（任何一层拒绝）时检查距上次有效更新
  // 的墙钟间隔，超阈值强制重置。断流不触发（无观测到达则无人调用）。
  // 重置自身记为一次事件——至多每周期重置一次，防 WARN 刷屏。
  void consider_divergence_reset()
  {
    const double t = now().seconds();
    if (last_filter_update_s_ > 0.0 &&
      t - last_filter_update_s_ > divergence_reset_period_s_)
    {
      filter_.reset();
      ++diag_filter_resets_;
      last_filter_update_s_ = t;
      RCLCPP_WARN(
        get_logger(),
        "estimator diverged (no update for %.1fs) - filter reset",
        divergence_reset_period_s_);
    }
  }

  void publish_estimate()
  {
    const auto current_time = now();
    const auto state = filter_.predicted_state(current_time.seconds());
    px4_offboard_tracking::msg::RelativeEstimate out;
    out.header.stamp = current_time;
    out.header.frame_id = common_frame_id_;
    out.position.x = state.position[0];
    out.position.y = state.position[1];
    out.position.z = state.position[2];
    out.velocity.x = state.velocity[0];
    out.velocity.y = state.velocity[1];
    out.velocity.z = state.velocity[2];
    out.acceleration.x = state.acceleration[0];
    out.acceleration.y = state.acceleration[1];
    out.acceleration.z = state.acceleration[2];
    out.covariance = state.covariance;
    out.full_covariance = state.full_covariance;
    out.innovation = state.innovation;
    out.innovation_mahalanobis = state.innovation_mahalanobis;
    out.normalized_innovation_squared = state.innovation_mahalanobis;
    out.model_probabilities = state.model_probabilities;
    out.data_age = state.initialized ?
      current_time.seconds() - state.last_accepted_measurement_seconds :
      std::numeric_limits<double>::infinity();
    out.coast_time_s = out.data_age;
    double maximum_position_variance = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      maximum_position_variance = std::max(
        maximum_position_variance, state.full_covariance[9 * axis + axis]);
    }
    out.measurement_accepted = last_measurement_accepted_;
    out.valid = state.initialized && out.data_age >= 0.0 &&
      out.data_age <= max_data_age_s_ &&
      maximum_position_variance <= max_position_variance_;
    estimate_pub_->publish(out);
    // 节流诊断（数据链排障埋点）：观测流入/拒绝原因/接受数/滤波器状态。
    const double diag_time_s = current_time.seconds();
    if (diag_time_s - last_diag_time_ >= 2.0) {
      last_diag_time_ = diag_time_s;
      RCLCPP_INFO(
        get_logger(),
        "estimator diag: obs=%llu inv=%llu attmiss=%llu acc=%llu bo=%llu "
        "reset=%llu init=%d valid=%d age=%.2f",
        static_cast<unsigned long long>(diag_obs_received_),
        static_cast<unsigned long long>(diag_rejected_invalid_),
        static_cast<unsigned long long>(diag_rejected_attitude_),
        static_cast<unsigned long long>(diag_updates_accepted_),
        static_cast<unsigned long long>(diag_bearing_only_),
        static_cast<unsigned long long>(diag_filter_resets_),
        static_cast<int>(state.initialized), static_cast<int>(out.valid),
        std::isfinite(out.data_age) ? out.data_age : -1.0);
    }
  }

  std::string observation_topic_;
  std::string estimate_topic_;
  std::string attitude_topic_;
  std::string common_frame_id_;
  double estimator_rate_hz_;
  double max_data_age_s_;
  double max_position_variance_;
  double attitude_sync_tolerance_s_;
  double attitude_buffer_s_;
  double attitude_clock_domain_tolerance_s_;
  double pixel_stddev_px_;
  double depth_stddev_m_;
  CameraIntrinsics camera_intrinsics_;
  CameraModel camera_model_;
  SampleTimeAttitudeBuffer attitude_buffer_;
  RelativeImmEkf filter_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  bool last_measurement_accepted_{false};
  rclcpp::Subscription<anti_drone_interfaces::msg::DepthObservation>::SharedPtr observation_sub_;
  rclcpp::Publisher<px4_offboard_tracking::msg::RelativeEstimate>::SharedPtr estimate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t diag_obs_received_{0};
  std::uint64_t diag_rejected_invalid_{0};
  std::uint64_t diag_rejected_attitude_{0};
  std::uint64_t diag_updates_accepted_{0};
  std::uint64_t diag_bearing_only_{0};
  bool bearing_only_enabled_{true};
  double bearing_only_nominal_range_m_{12.0};
  double bearing_only_range_sigma_m_{30.0};
  double divergence_reset_period_s_{1.5};
  double last_filter_update_s_{-1.0};
  std::uint64_t diag_filter_resets_{0};
  double last_diag_time_{0.0};
};

}  // namespace px4_offboard_tracking

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard_tracking::RelativeEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
