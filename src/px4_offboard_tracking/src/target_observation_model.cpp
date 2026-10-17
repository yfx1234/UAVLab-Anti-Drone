#include "px4_offboard_tracking/target_observation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace px4_offboard_tracking
{

TargetObservationModel::TargetObservationModel(
  const CameraIntrinsics & intrinsics, const ObservationFaultConfig & config)
: intrinsics_(intrinsics), config_(config), random_engine_(config.random_seed)
{
  if (config_.pixel_noise_stddev_px < 0.0 || config_.depth_noise_stddev_m < 0.0) {
    throw std::invalid_argument("observation noise standard deviations must be non-negative");
  }
  validate_probability(config_.dropout_probability, "dropout_probability");
  validate_probability(config_.outlier_probability, "outlier_probability");
  if (config_.outlier_min_depth_m <= intrinsics_.minimum_depth_m ||
    config_.outlier_max_depth_m <= config_.outlier_min_depth_m)
  {
    throw std::invalid_argument("invalid outlier depth interval");
  }
}

void TargetObservationModel::validate_probability(double probability, const char * name)
{
  if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0) {
    throw std::invalid_argument(std::string(name) + " must be in [0, 1]");
  }
}

void TargetObservationModel::back_project(TargetObservationSample & sample) const
{
  sample.position_camera.x = (sample.pixel_u - intrinsics_.cx) / intrinsics_.fx * sample.depth;
  sample.position_camera.y = (sample.pixel_v - intrinsics_.cy) / intrinsics_.fy * sample.depth;
  sample.position_camera.z = sample.depth;
}

std::optional<TargetObservationSample> TargetObservationModel::apply(
  const PinholeProjection & ideal)
{
  if (unit_uniform_(random_engine_) < config_.dropout_probability) {
    return std::nullopt;
  }

  TargetObservationSample output{
    ideal.position_camera, ideal.pixel_u, ideal.pixel_v, ideal.depth, ideal.visible, false};

  if (unit_uniform_(random_engine_) < config_.outlier_probability) {
    std::uniform_real_distribution<double> pixel_u_distribution(
      0.0, static_cast<double>(intrinsics_.width - 1));
    std::uniform_real_distribution<double> pixel_v_distribution(
      0.0, static_cast<double>(intrinsics_.height - 1));
    std::uniform_real_distribution<double> depth_distribution(
      config_.outlier_min_depth_m, config_.outlier_max_depth_m);
    output.pixel_u = pixel_u_distribution(random_engine_);
    output.pixel_v = pixel_v_distribution(random_engine_);
    output.depth = depth_distribution(random_engine_);
    output.visible = true;
    output.outlier_injected = true;
    back_project(output);
    return output;
  }

  if (!ideal.visible) {
    return output;
  }

  if (config_.pixel_noise_stddev_px > 0.0) {
    std::normal_distribution<double> pixel_noise(0.0, config_.pixel_noise_stddev_px);
    output.pixel_u += pixel_noise(random_engine_);
    output.pixel_v += pixel_noise(random_engine_);
  }
  if (config_.depth_noise_stddev_m > 0.0) {
    std::normal_distribution<double> depth_noise(0.0, config_.depth_noise_stddev_m);
    output.depth = std::max(
      intrinsics_.minimum_depth_m, output.depth + depth_noise(random_engine_));
  }

  output.visible = output.pixel_u >= 0.0 && output.pixel_u < intrinsics_.width &&
    output.pixel_v >= 0.0 && output.pixel_v < intrinsics_.height &&
    output.depth > intrinsics_.minimum_depth_m;
  back_project(output);
  return output;
}

}  // namespace px4_offboard_tracking
