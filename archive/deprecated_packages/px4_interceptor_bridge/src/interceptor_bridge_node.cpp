#include "px4_interceptor_bridge/bridge_geometry.hpp"
#include "px4_interceptor_bridge/timestamp_offset_estimator.hpp"

#include <anti_drone_interfaces/msg/target_camera_position.hpp>
#include <px4_msgs/msg/target_relative_measurement.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace px4_interceptor_bridge
{

class InterceptorBridgeNode : public rclcpp::Node
{
public:
  InterceptorBridgeNode()
  : Node("px4_interceptor_bridge"),
    input_topic_(declare_parameter<std::string>(
        "input_topic", "/perception/target/camera_position")),
    output_topic_(declare_parameter<std::string>(
        "output_topic", "/px4_1/fmu/in/target_relative_measurement")),
    attitude_topic_(declare_parameter<std::string>(
        "attitude_topic", "/px4_1/fmu/out/vehicle_attitude")),
    timesync_topic_(declare_parameter<std::string>(
        "timesync_topic", "/px4_1/fmu/out/timesync_status")),
    expected_frame_id_(declare_parameter<std::string>(
        "expected_frame_id", "pursuer_camera_optical")),
    minimum_confidence_(declare_parameter<double>("minimum_confidence", 0.5)),
    minimum_depth_m_(declare_parameter<double>("minimum_depth_m", 0.05)),
    maximum_measurement_age_s_(declare_parameter<double>("maximum_measurement_age_s", 0.25)),
    maximum_future_s_(declare_parameter<double>("maximum_future_s", 0.02)),
    measurement_queue_timeout_s_(
      declare_parameter<double>("measurement_queue_timeout_s", 0.10)),
    attitude_tolerance_s_(declare_parameter<double>("attitude_sync_tolerance_s", 0.06)),
    attitude_buffer_s_(declare_parameter<double>("attitude_buffer_s", 3.0)),
    timestamp_offset_jump_threshold_s_(
      declare_parameter<double>("timestamp_offset_jump_threshold_s", 0.10)),
    timestamp_offset_filter_alpha_(
      declare_parameter<double>("timestamp_offset_filter_alpha", 0.05)),
    timestamp_offset_window_size_(
      declare_parameter<int>("timestamp_offset_window_size", 100)),
    timestamp_jump_confirmation_samples_(
      declare_parameter<int>("timestamp_jump_confirmation_samples", 5)),
    diagnostics_period_s_(declare_parameter<double>("diagnostics_period_s", 5.0)),
    sequence_reset_timeout_s_(declare_parameter<double>("sequence_reset_timeout_s", 1.0)),
    covariance_symmetry_tolerance_(
      declare_parameter<double>("covariance_symmetry_tolerance", 1e-6)),
    maximum_position_variance_(declare_parameter<double>("maximum_position_variance", 100.0)),
    require_timesync_(declare_parameter<bool>("require_timesync", false)),
    maximum_timesync_age_s_(declare_parameter<double>("maximum_timesync_age_s", 5.0)),
    maximum_timesync_rtt_us_(declare_parameter<int>("maximum_timesync_rtt_us", 30000)),
    geometry_(CameraExtrinsics{
        Eigen::Vector3d{
          declare_parameter<double>("camera_translation_body_x_m", 0.0),
          declare_parameter<double>("camera_translation_body_y_m", 0.0),
          declare_parameter<double>("camera_translation_body_z_m", 0.0)},
        Eigen::Quaterniond{
          declare_parameter<double>("camera_q_body_to_optical_w", 0.0),
          declare_parameter<double>("camera_q_body_to_optical_x", 0.7071067811865476),
          declare_parameter<double>("camera_q_body_to_optical_y", 0.7071067811865476),
          declare_parameter<double>("camera_q_body_to_optical_z", 0.0)}}),
    timestamp_offset_estimator_(
      static_cast<std::int64_t>(timestamp_offset_jump_threshold_s_ * 1e9),
      timestamp_offset_filter_alpha_,
      static_cast<std::size_t>(timestamp_offset_window_size_),
      static_cast<unsigned>(timestamp_jump_confirmation_samples_))
  {
    validate_parameters();
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    sensor_qos.best_effort().durability_volatile();
    publisher_ = create_publisher<px4_msgs::msg::TargetRelativeMeasurement>(
      output_topic_, sensor_qos);
    attitude_subscription_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      attitude_topic_, sensor_qos,
      [this](px4_msgs::msg::VehicleAttitude::UniquePtr message) {on_attitude(*message);});
    timesync_subscription_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
      timesync_topic_, sensor_qos,
      [this](px4_msgs::msg::TimesyncStatus::UniquePtr message) {on_timesync(*message);});
    measurement_subscription_ =
      create_subscription<anti_drone_interfaces::msg::TargetCameraPosition>(
      input_topic_, sensor_qos,
      [this](anti_drone_interfaces::msg::TargetCameraPosition::UniquePtr message) {
        on_measurement(*message);
      });
    pending_timer_ = create_wall_timer(
      std::chrono::milliseconds(5), [this]() {process_pending_measurements();});
    RCLCPP_INFO(get_logger(), "bridge %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

private:
  struct AttitudeSample
  {
    std::int64_t timestamp_ns{};
    Eigen::Quaterniond q_body_to_ned;
  };

  struct PendingMeasurement
  {
    anti_drone_interfaces::msg::TargetCameraPosition message;
    rclcpp::Time receipt_time;
  };

  void validate_parameters() const
  {
    if (expected_frame_id_.empty() || minimum_confidence_ < 0.0 || minimum_confidence_ > 1.0 ||
      minimum_depth_m_ <= 0.0 || maximum_measurement_age_s_ <= 0.0 || maximum_future_s_ < 0.0 ||
      measurement_queue_timeout_s_ <= 0.0 ||
      attitude_tolerance_s_ <= 0.0 || attitude_buffer_s_ <= attitude_tolerance_s_ ||
      timestamp_offset_jump_threshold_s_ <= 0.0 ||
      timestamp_offset_filter_alpha_ <= 0.0 || timestamp_offset_filter_alpha_ > 1.0 ||
      timestamp_offset_window_size_ < 5 || timestamp_jump_confirmation_samples_ < 2 ||
      diagnostics_period_s_ <= 0.0 ||
      sequence_reset_timeout_s_ <= maximum_measurement_age_s_ ||
      covariance_symmetry_tolerance_ < 0.0 || maximum_position_variance_ <= 0.0 ||
      maximum_timesync_age_s_ <= 0.0 || maximum_timesync_rtt_us_ <= 0)
    {
      throw std::invalid_argument("invalid interceptor bridge parameters");
    }
  }

  void on_attitude(const px4_msgs::msg::VehicleAttitude & message)
  {
    Eigen::Quaterniond quaternion{message.q[0], message.q[1], message.q[2], message.q[3]};
    const std::uint64_t sample_timestamp_us =
      message.timestamp_sample != 0 ? message.timestamp_sample : message.timestamp;
    if (message.timestamp == 0 || sample_timestamp_us == 0 ||
      !quaternion.coeffs().allFinite() || quaternion.norm() < 1e-9)
    {
      return;
    }
    if (last_attitude_sample_us_ != 0 && sample_timestamp_us <= last_attitude_sample_us_) {
      ++duplicate_attitude_samples_;
      return;
    }
    last_attitude_sample_us_ = sample_timestamp_us;
    const std::int64_t receipt_timestamp_ns = now().nanoseconds();
    const std::int64_t publication_timestamp_ns =
      static_cast<std::int64_t>(message.timestamp) * 1000;
    const std::int64_t observed_offset_ns =
      receipt_timestamp_ns - publication_timestamp_ns;
    const auto offset_update = timestamp_offset_estimator_.update(observed_offset_ns);
    if (offset_update.confirmed_jump) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "confirmed DDS attitude clock offset change to %.3f ms; reset attitude buffer",
        static_cast<double>(offset_update.offset_ns) / 1e6);
      attitude_buffer_.clear();
    }
    if (offset_update.latency_outlier) {++attitude_latency_outliers_;}
    quaternion.normalize();
    const std::int64_t corrected_sample_timestamp_ns =
      static_cast<std::int64_t>(sample_timestamp_us) * 1000 + offset_update.offset_ns;
    attitude_buffer_.push_back(
      {corrected_sample_timestamp_ns, quaternion});
    const std::int64_t oldest_ns = attitude_buffer_.back().timestamp_ns -
      static_cast<std::int64_t>(attitude_buffer_s_ * 1e9);
    while (!attitude_buffer_.empty() && attitude_buffer_.front().timestamp_ns < oldest_ns) {
      attitude_buffer_.pop_front();
    }
    process_pending_measurements();
    publish_diagnostics_if_due();
  }

  void on_timesync(const px4_msgs::msg::TimesyncStatus & message)
  {
    if (message.source_protocol == px4_msgs::msg::TimesyncStatus::SOURCE_PROTOCOL_DDS &&
      message.round_trip_time <= static_cast<std::uint32_t>(maximum_timesync_rtt_us_))
    {
      have_healthy_timesync_ = true;
      last_healthy_timesync_receipt_ = now();
    }
    process_pending_measurements();
  }

  bool attitude_at(const std::int64_t timestamp_ns, Eigen::Quaterniond & output) const
  {
    if (attitude_buffer_.empty()) {return false;}
    const std::int64_t tolerance_ns = static_cast<std::int64_t>(attitude_tolerance_s_ * 1e9);
    const auto upper = std::lower_bound(
      attitude_buffer_.begin(), attitude_buffer_.end(), timestamp_ns,
      [](const AttitudeSample & sample, const std::int64_t time) {
        return sample.timestamp_ns < time;
      });
    if (upper != attitude_buffer_.end() && upper != attitude_buffer_.begin()) {
      const auto lower = std::prev(upper);
      if (timestamp_ns - lower->timestamp_ns <= tolerance_ns &&
        upper->timestamp_ns - timestamp_ns <= tolerance_ns)
      {
        const double alpha = static_cast<double>(timestamp_ns - lower->timestamp_ns) /
          static_cast<double>(upper->timestamp_ns - lower->timestamp_ns);
        output = lower->q_body_to_ned.slerp(alpha, upper->q_body_to_ned).normalized();
        return true;
      }
    }
    const AttitudeSample * nearest = nullptr;
    std::int64_t nearest_difference = tolerance_ns + 1;
    if (upper != attitude_buffer_.end()) {
      nearest = &*upper;
      nearest_difference = std::abs(upper->timestamp_ns - timestamp_ns);
    }
    if (upper != attitude_buffer_.begin()) {
      const auto lower = std::prev(upper);
      const auto difference = std::abs(lower->timestamp_ns - timestamp_ns);
      if (difference < nearest_difference) {
        nearest = &*lower;
        nearest_difference = difference;
      }
    }
    if (nearest == nullptr || nearest_difference > tolerance_ns) {return false;}
    output = nearest->q_body_to_ned;
    return true;
  }

  bool sequence_is_new(const std::uint32_t sequence) const
  {
    return !have_sequence_ || static_cast<std::int32_t>(sequence - last_sequence_) > 0;
  }

  bool timesync_is_healthy(const rclcpp::Time & current_time) const
  {
    return !require_timesync_ ||
      (have_healthy_timesync_ && last_healthy_timesync_receipt_.nanoseconds() > 0 &&
      (current_time - last_healthy_timesync_receipt_).seconds() <= maximum_timesync_age_s_);
  }

  void reject(const char * reason)
  {
    ++rejected_measurements_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "reject target measurement: %s", reason);
  }

  void publish_diagnostics_if_due()
  {
    const auto current = now();
    if (last_diagnostics_.nanoseconds() > 0 &&
      (current - last_diagnostics_).seconds() < diagnostics_period_s_)
    {
      return;
    }
    const std::uint64_t attempted = published_measurements_ + rejected_measurements_;
    const double success_ratio = attempted > 0 ?
      static_cast<double>(published_measurements_) / static_cast<double>(attempted) : 1.0;
    RCLCPP_INFO(
      get_logger(),
      "communication continuity: published=%lu rejected=%lu success=%.1f%% "
      "attitude_duplicates=%lu attitude_latency_outliers=%lu pending=%zu offset=%.3f ms",
      published_measurements_, rejected_measurements_, 100.0 * success_ratio,
      duplicate_attitude_samples_, attitude_latency_outliers_, pending_measurements_.size(),
      static_cast<double>(timestamp_offset_estimator_.offset_ns()) / 1e6);
    last_diagnostics_ = current;
  }

  void on_measurement(const anti_drone_interfaces::msg::TargetCameraPosition & message)
  {
    const rclcpp::Time current_time = now();
    const rclcpp::Time sample_time(message.header.stamp);
    const double age_s = (current_time - sample_time).seconds();
    if (!message.valid) {reject("invalid flag"); return;}
    if (message.header.frame_id != expected_frame_id_) {reject("unexpected frame_id"); return;}
    if (sample_time.nanoseconds() <= 0 || age_s > maximum_measurement_age_s_ ||
      age_s < -maximum_future_s_) {reject("timestamp age"); return;}
    const bool source_restart_allowed = have_seen_sequence_ &&
      last_seen_measurement_receipt_.nanoseconds() > 0 &&
      (current_time - last_seen_measurement_receipt_).seconds() > sequence_reset_timeout_s_;
    if ((!sequence_is_new(message.sequence) && !source_restart_allowed) ||
      (have_seen_sequence_ && !source_restart_allowed &&
      static_cast<std::int32_t>(message.sequence - last_seen_sequence_) <= 0))
    {
      reject("duplicate or regressed sequence"); return;
    }
    if (!std::isfinite(message.confidence) || message.confidence < minimum_confidence_) {
      reject("confidence"); return;
    }
    const Eigen::Vector3d position_camera{
      message.position_camera.x, message.position_camera.y, message.position_camera.z};
    if (!position_camera.allFinite() || position_camera.z() <= minimum_depth_m_) {
      reject("camera position"); return;
    }
    Eigen::Matrix3d covariance_camera;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance_camera(row, column) = message.position_covariance[3 * row + column];
      }
    }
    if (!BridgeGeometry::covariance_is_valid(
        covariance_camera, covariance_symmetry_tolerance_) ||
      covariance_camera.diagonal().maxCoeff() > maximum_position_variance_)
    {
      reject("position covariance"); return;
    }
    pending_measurements_.push_back({message, current_time});
    last_seen_sequence_ = message.sequence;
    have_seen_sequence_ = true;
    last_seen_measurement_receipt_ = current_time;
    process_pending_measurements();
  }

  void process_pending_measurements()
  {
    const rclcpp::Time current_time = now();
    while (!pending_measurements_.empty()) {
      const auto & pending = pending_measurements_.front();
      const rclcpp::Time sample_time(pending.message.header.stamp);
      const double queue_age_s = (current_time - pending.receipt_time).seconds();
      const double measurement_age_s = (current_time - sample_time).seconds();
      if (measurement_age_s > maximum_measurement_age_s_) {
        reject("measurement expired while queued");
        pending_measurements_.pop_front();
        continue;
      }
      if (!timesync_is_healthy(current_time)) {
        if (queue_age_s < measurement_queue_timeout_s_) {return;}
        reject("DDS timesync timeout");
        pending_measurements_.pop_front();
        continue;
      }
      Eigen::Quaterniond q_body_to_ned;
      if (!attitude_at(sample_time.nanoseconds(), q_body_to_ned)) {
        if (queue_age_s < measurement_queue_timeout_s_) {return;}
        reject("synchronized attitude timeout");
        pending_measurements_.pop_front();
        continue;
      }
      publish_measurement(pending.message, sample_time, current_time, q_body_to_ned);
      pending_measurements_.pop_front();
    }
  }

  void publish_measurement(
    const anti_drone_interfaces::msg::TargetCameraPosition & message,
    const rclcpp::Time & sample_time,
    const rclcpp::Time & current_time,
    const Eigen::Quaterniond & q_body_to_ned)
  {
    const Eigen::Vector3d position_camera{
      message.position_camera.x, message.position_camera.y, message.position_camera.z};
    Eigen::Matrix3d covariance_camera;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance_camera(row, column) = message.position_covariance[3 * row + column];
      }
    }
    const Eigen::Vector3d relative_ned =
      geometry_.camera_position_to_relative_ned(position_camera, q_body_to_ned);
    const Eigen::Matrix3d covariance_ned =
      geometry_.camera_covariance_to_ned(covariance_camera, q_body_to_ned);
    px4_msgs::msg::TargetRelativeMeasurement output;
    output.timestamp = static_cast<std::uint64_t>(current_time.nanoseconds() / 1000);
    output.timestamp_sample = static_cast<std::uint64_t>(sample_time.nanoseconds() / 1000);
    output.contract_version = px4_msgs::msg::TargetRelativeMeasurement::CONTRACT_VERSION_V1;
    output.sequence = message.sequence;
    output.frame = px4_msgs::msg::TargetRelativeMeasurement::FRAME_RELATIVE_NED;
    for (int index = 0; index < 3; ++index) {
      output.position[index] = static_cast<float>(relative_ned(index));
      output.velocity[index] = std::numeric_limits<float>::quiet_NaN();
    }
    output.covariance.fill(std::numeric_limits<float>::quiet_NaN());
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        output.covariance[6 * row + column] =
          static_cast<float>(covariance_ned(row, column));
      }
    }
    output.position_valid = true;
    output.velocity_valid = false;
    output.valid = true;
    publisher_->publish(output);
    ++published_measurements_;
    last_sequence_ = message.sequence;
    have_sequence_ = true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string attitude_topic_;
  std::string timesync_topic_;
  std::string expected_frame_id_;
  double minimum_confidence_;
  double minimum_depth_m_;
  double maximum_measurement_age_s_;
  double maximum_future_s_;
  double measurement_queue_timeout_s_;
  double attitude_tolerance_s_;
  double attitude_buffer_s_;
  double timestamp_offset_jump_threshold_s_;
  double timestamp_offset_filter_alpha_;
  int timestamp_offset_window_size_;
  int timestamp_jump_confirmation_samples_;
  double diagnostics_period_s_;
  double sequence_reset_timeout_s_;
  double covariance_symmetry_tolerance_;
  double maximum_position_variance_;
  bool require_timesync_;
  double maximum_timesync_age_s_;
  int maximum_timesync_rtt_us_;
  BridgeGeometry geometry_;
  TimestampOffsetEstimator timestamp_offset_estimator_;
  std::deque<AttitudeSample> attitude_buffer_;
  std::deque<PendingMeasurement> pending_measurements_;
  bool have_healthy_timesync_{false};
  rclcpp::Time last_healthy_timesync_receipt_{0, 0, RCL_ROS_TIME};
  bool have_sequence_{false};
  std::uint32_t last_sequence_{};
  bool have_seen_sequence_{false};
  std::uint32_t last_seen_sequence_{};
  rclcpp::Time last_seen_measurement_receipt_{0, 0, RCL_ROS_TIME};
  std::uint64_t published_measurements_{};
  std::uint64_t rejected_measurements_{};
  std::uint64_t attitude_latency_outliers_{};
  std::uint64_t duplicate_attitude_samples_{};
  std::uint64_t last_attitude_sample_us_{};
  rclcpp::Time last_diagnostics_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<px4_msgs::msg::TargetRelativeMeasurement>::SharedPtr publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_subscription_;
  rclcpp::Subscription<anti_drone_interfaces::msg::TargetCameraPosition>::SharedPtr
    measurement_subscription_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
};

}  // namespace px4_interceptor_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_interceptor_bridge::InterceptorBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
