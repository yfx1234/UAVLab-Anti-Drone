#include <gtest/gtest.h>

#include <cmath>

#include "px4_interceptor_bridge/bridge_geometry.hpp"

namespace
{

constexpr double kTolerance = 1e-9;

px4_interceptor_bridge::BridgeGeometry upward_camera(
  const Eigen::Vector3d & translation = Eigen::Vector3d::Zero())
{
  return px4_interceptor_bridge::BridgeGeometry({
      translation,
      Eigen::Quaterniond{0.0, std::sqrt(0.5), std::sqrt(0.5), 0.0}});
}

TEST(BridgeGeometry, UpwardOpticalAxesMapToFrd)
{
  const auto geometry = upward_camera();
  const Eigen::Quaterniond level_body{1.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(geometry.camera_position_to_relative_ned({0.0, 0.0, 7.0}, level_body)
    .isApprox(Eigen::Vector3d{0.0, 0.0, -7.0}, kTolerance));
  EXPECT_TRUE(geometry.camera_position_to_relative_ned({1.0, 0.0, 0.0}, level_body)
    .isApprox(Eigen::Vector3d{0.0, 1.0, 0.0}, kTolerance));
  EXPECT_TRUE(geometry.camera_position_to_relative_ned({0.0, 1.0, 0.0}, level_body)
    .isApprox(Eigen::Vector3d{1.0, 0.0, 0.0}, kTolerance));
}

TEST(BridgeGeometry, IncludesCameraTranslation)
{
  const auto geometry = upward_camera({0.2, -0.1, 0.3});
  const Eigen::Vector3d result = geometry.camera_position_to_relative_ned(
    {0.0, 0.0, 2.0}, {1.0, 0.0, 0.0, 0.0});
  EXPECT_TRUE(result.isApprox(Eigen::Vector3d{0.2, -0.1, -1.7}, kTolerance));
}

TEST(BridgeGeometry, RotatesCovarianceWithAxes)
{
  const auto geometry = upward_camera();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance.diagonal() << 1.0, 4.0, 9.0;
  const Eigen::Matrix3d result = geometry.camera_covariance_to_ned(
    covariance, {1.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(result(0, 0), 4.0, kTolerance);
  EXPECT_NEAR(result(1, 1), 1.0, kTolerance);
  EXPECT_NEAR(result(2, 2), 9.0, kTolerance);
}

TEST(BridgeGeometry, RejectsInvalidCovariance)
{
  Eigen::Matrix3d valid = Eigen::Matrix3d::Identity();
  EXPECT_TRUE(px4_interceptor_bridge::BridgeGeometry::covariance_is_valid(valid, 1e-9));
  valid(0, 1) = 0.5;
  EXPECT_FALSE(px4_interceptor_bridge::BridgeGeometry::covariance_is_valid(valid, 1e-9));
  Eigen::Matrix3d negative = Eigen::Matrix3d::Identity();
  negative(2, 2) = -1.0;
  EXPECT_FALSE(px4_interceptor_bridge::BridgeGeometry::covariance_is_valid(negative, 1e-9));
}

}  // namespace
