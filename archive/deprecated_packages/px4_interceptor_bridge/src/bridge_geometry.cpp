#include "px4_interceptor_bridge/bridge_geometry.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <stdexcept>

namespace px4_interceptor_bridge
{

BridgeGeometry::BridgeGeometry(const CameraExtrinsics & extrinsics)
: extrinsics_(extrinsics)
{
  if (!extrinsics_.translation_body_frd_m.allFinite() ||
    !extrinsics_.q_body_to_optical.coeffs().allFinite() ||
    extrinsics_.q_body_to_optical.norm() < 1e-9)
  {
    throw std::invalid_argument("invalid camera extrinsics");
  }
  extrinsics_.q_body_to_optical.normalize();
}

Eigen::Vector3d BridgeGeometry::camera_position_to_relative_ned(
  const Eigen::Vector3d & position_camera,
  const Eigen::Quaterniond & q_body_to_ned_input) const
{
  Eigen::Quaterniond q_body_to_ned = q_body_to_ned_input;
  if (!position_camera.allFinite() || !q_body_to_ned.coeffs().allFinite() ||
    q_body_to_ned.norm() < 1e-9)
  {
    throw std::invalid_argument("invalid position or attitude");
  }
  q_body_to_ned.normalize();
  const Eigen::Vector3d position_body =
    extrinsics_.q_body_to_optical.conjugate() * position_camera;
  return q_body_to_ned * (extrinsics_.translation_body_frd_m + position_body);
}

Eigen::Matrix3d BridgeGeometry::camera_covariance_to_ned(
  const Eigen::Matrix3d & covariance_camera,
  const Eigen::Quaterniond & q_body_to_ned_input) const
{
  Eigen::Quaterniond q_body_to_ned = q_body_to_ned_input;
  if (!covariance_camera.allFinite() || !q_body_to_ned.coeffs().allFinite() ||
    q_body_to_ned.norm() < 1e-9)
  {
    throw std::invalid_argument("invalid covariance or attitude");
  }
  q_body_to_ned.normalize();
  const Eigen::Matrix3d rotation =
    q_body_to_ned.toRotationMatrix() * extrinsics_.q_body_to_optical.conjugate().toRotationMatrix();
  return rotation * covariance_camera * rotation.transpose();
}

bool BridgeGeometry::covariance_is_valid(
  const Eigen::Matrix3d & covariance, const double symmetry_tolerance)
{
  if (!covariance.allFinite() || symmetry_tolerance < 0.0 ||
    !covariance.isApprox(covariance.transpose(), symmetry_tolerance))
  {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -symmetry_tolerance;
}

}  // namespace px4_interceptor_bridge
