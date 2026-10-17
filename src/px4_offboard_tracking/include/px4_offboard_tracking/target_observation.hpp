#pragma once

#include <cstdint>
#include <optional>
#include <random>

#include "px4_offboard_tracking/camera_model.hpp"

namespace px4_offboard_tracking
{

struct ObservationFaultConfig
{
  double pixel_noise_stddev_px{};
  double depth_noise_stddev_m{};
  double dropout_probability{};
  double outlier_probability{};
  double outlier_min_depth_m{1.0};
  double outlier_max_depth_m{20.0};
  std::uint64_t random_seed{1};
};

struct TargetObservationSample
{
  Vector3d position_camera{};
  double pixel_u{};
  double pixel_v{};
  double depth{};
  bool visible{false};
  bool outlier_injected{false};
};

class TargetObservationModel
{
public:
  TargetObservationModel(
    const CameraIntrinsics & intrinsics, const ObservationFaultConfig & config);

  std::optional<TargetObservationSample> apply(const PinholeProjection & ideal);

private:
  static void validate_probability(double probability, const char * name);
  void back_project(TargetObservationSample & sample) const;

  CameraIntrinsics intrinsics_;
  ObservationFaultConfig config_;
  std::mt19937_64 random_engine_;
  std::uniform_real_distribution<double> unit_uniform_{0.0, 1.0};
};

}  // namespace px4_offboard_tracking
