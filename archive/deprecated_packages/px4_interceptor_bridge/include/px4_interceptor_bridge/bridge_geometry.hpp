#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace px4_interceptor_bridge
{

struct CameraExtrinsics
{
  Eigen::Vector3d translation_body_frd_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_body_to_optical{1.0, 0.0, 0.0, 0.0};
};

class BridgeGeometry
{
public:
  explicit BridgeGeometry(const CameraExtrinsics & extrinsics);

  Eigen::Vector3d camera_position_to_relative_ned(
    const Eigen::Vector3d & position_camera,
    const Eigen::Quaterniond & q_body_to_ned) const;

  Eigen::Matrix3d camera_covariance_to_ned(
    const Eigen::Matrix3d & covariance_camera,
    const Eigen::Quaterniond & q_body_to_ned) const;

  static bool covariance_is_valid(const Eigen::Matrix3d & covariance, double symmetry_tolerance);

private:
  CameraExtrinsics extrinsics_;
};

}  // namespace px4_interceptor_bridge
