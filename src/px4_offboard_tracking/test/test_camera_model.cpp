#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/camera_model.hpp"

using px4_offboard_tracking::CameraExtrinsics;
using px4_offboard_tracking::CameraFieldOfView;
using px4_offboard_tracking::CameraIntrinsics;
using px4_offboard_tracking::CameraModel;
using px4_offboard_tracking::BearingCameraModel;
using px4_offboard_tracking::Quaterniond;
using px4_offboard_tracking::Vector3d;

namespace
{
CameraModel upward_camera(const Vector3d & translation = {})
{
  const CameraIntrinsics intrinsics{320.0, 320.0, 319.5, 239.5, 640, 480, 0.05};
  const CameraExtrinsics extrinsics{translation, CameraModel::default_upward_optical_quaternion()};
  return CameraModel(intrinsics, extrinsics);
}
}  // namespace

TEST(CameraModel, TargetAboveLevelVehicleProjectsToImageCenter)
{
  const auto projection = upward_camera().project({0.0, 0.0, -7.0}, Quaterniond{});
  EXPECT_NEAR(projection.position_camera.x, 0.0, 1e-12);
  EXPECT_NEAR(projection.position_camera.y, 0.0, 1e-12);
  EXPECT_NEAR(projection.depth, 7.0, 1e-12);
  EXPECT_NEAR(projection.pixel_u, 319.5, 1e-12);
  EXPECT_NEAR(projection.pixel_v, 239.5, 1e-12);
  EXPECT_TRUE(projection.visible);
}

// H-4：三个用例的期望从旧安装四元数 (0,√2/2,√2/2,0) 更新到 2026-10-13
// 实物定案 (0,1,0,0)——optical +Z = body -Z（朝上）、+X(u) = body +X
//（机头/宽轴）、+Y(v) = body -Y（指向左舷）。

TEST(CameraModel, OpticalAxesFollowForwardOnUAndRightOnNegativeV)
{
  const auto right = upward_camera().project({0.0, 1.0, -5.0}, Quaterniond{});
  const auto body_forward = upward_camera().project({1.0, 0.0, -5.0}, Quaterniond{});
  EXPECT_NEAR(right.pixel_u, 319.5, 1e-12);
  EXPECT_LT(right.pixel_v, 239.5);   // body 右舷 → v 减小（图像上方）
  EXPECT_GT(body_forward.pixel_u, 319.5);  // 机头 → u 增大（宽轴）
  EXPECT_NEAR(body_forward.pixel_v, 239.5, 1e-12);
}

TEST(CameraModel, DepthOpticalCoordinatesMatchDeclaredBodyFrdMatrix)
{
  const auto projection = upward_camera().project({1.0, 2.0, -5.0}, Quaterniond{});
  // (x,y,z)_body → (x, -y, -z)_optical
  EXPECT_NEAR(projection.position_camera.x, 1.0, 1e-12);
  EXPECT_NEAR(projection.position_camera.y, -2.0, 1e-12);
  EXPECT_NEAR(projection.position_camera.z, 5.0, 1e-12);
}

TEST(CameraModel, TargetBehindUpwardCameraIsNotVisible)
{
  const auto projection = upward_camera().project({0.0, 0.0, 2.0}, Quaterniond{});
  EXPECT_LT(projection.depth, 0.0);
  EXPECT_FALSE(projection.visible);
  EXPECT_TRUE(std::isnan(projection.pixel_u));
}

TEST(CameraModel, TranslationExtrinsicChangesDepth)
{
  const auto projection = upward_camera({0.0, 0.0, -0.2}).project(
    {0.0, 0.0, -7.0}, Quaterniond{});
  EXPECT_NEAR(projection.depth, 6.8, 1e-12);
}

TEST(CameraModel, ProjectionAndInverseReturnCommonNedVector)
{
  const Vector3d relative_ned{1.0, 2.0, -7.0};
  const auto model = upward_camera({0.1, -0.2, -0.1});
  const auto projection = model.project(relative_ned, Quaterniond{});
  const auto recovered = model.camera_position_to_relative_ned(
    projection.position_camera, Quaterniond{});
  EXPECT_NEAR(recovered.x, relative_ned.x, 1e-12);
  EXPECT_NEAR(recovered.y, relative_ned.y, 1e-12);
  EXPECT_NEAR(recovered.z, relative_ned.z, 1e-12);
}

TEST(CameraModel, BodyOpticalAndRotatedNedRoundTripAtNumericalPrecision)
{
  const Vector3d relative_ned{-2.0, 1.0, -8.0};
  const double half_yaw = 0.25 * M_PI;
  const Quaterniond q_body_to_ned{
    std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw)};
  const auto model = upward_camera({0.1, -0.2, -0.1});
  const auto projection = model.project(relative_ned, q_body_to_ned);
  const auto recovered = model.camera_position_to_relative_ned(
    projection.position_camera, q_body_to_ned);
  EXPECT_NEAR(recovered.x, relative_ned.x, 1e-12);
  EXPECT_NEAR(recovered.y, relative_ned.y, 1e-12);
  EXPECT_NEAR(recovered.z, relative_ned.z, 1e-12);
}

TEST(CameraModel, RejectsInvalidIntrinsics)
{
  const CameraIntrinsics invalid{0.0, 320.0, 319.5, 239.5, 640, 480, 0.05};
  const CameraExtrinsics extrinsics{{}, CameraModel::default_upward_optical_quaternion()};
  EXPECT_THROW((void)CameraModel(invalid, extrinsics), std::invalid_argument);
}

TEST(CameraModel, DepthCameraUsesSpecifiedAngularFieldOfView)
{
  const CameraIntrinsics intrinsics{320.0, 320.0, 319.5, 239.5, 640, 480, 0.05};
  const CameraExtrinsics extrinsics{{}, CameraModel::default_upward_optical_quaternion()};
  const CameraModel model(
    intrinsics, extrinsics,
    CameraFieldOfView{8.5 * M_PI / 180.0, 10.0 * M_PI / 180.0});
  // 新安装下 u 轴 = 机头：水平半角 8.5° 管机头方向，垂直半角 10° 管
  // 右舷方向（v = -body_y）
  EXPECT_TRUE(model.project({0.0, 0.0, -10.0}, Quaterniond{}).visible);
  EXPECT_TRUE(model.project(
      {10.0 * std::tan(8.4 * M_PI / 180.0), 0.0, -10.0}, Quaterniond{}).visible);
  EXPECT_FALSE(model.project(
      {10.0 * std::tan(8.6 * M_PI / 180.0), 0.0, -10.0}, Quaterniond{}).visible);
  EXPECT_TRUE(model.project(
      {0.0, 10.0 * std::tan(9.9 * M_PI / 180.0), -10.0}, Quaterniond{}).visible);
  EXPECT_FALSE(model.project(
      {0.0, 10.0 * std::tan(10.1 * M_PI / 180.0), -10.0}, Quaterniond{}).visible);
}

// H-4：锁定实物安装四元数（2026-10-13 定案，宽轴沿机头）——库内函数、
// 估计器参数默认值与 tracking.yaml 三处必须一致
TEST(CameraModel, DefaultUpwardInstallationQuaternionMatchesHardwareDecision)
{
  const auto q = CameraModel::default_upward_optical_quaternion();
  EXPECT_NEAR(q.w, 0.0, 1e-12);
  EXPECT_NEAR(q.x, 1.0, 1e-12);
  EXPECT_NEAR(q.y, 0.0, 1e-12);
  EXPECT_NEAR(q.z, 0.0, 1e-12);
}

TEST(CameraModel, SearchCameraIsTiltedTenDegreesTowardBodyForward)
{
  const CameraIntrinsics intrinsics{320.0, 320.0, 319.5, 239.5, 640, 480, 0.05};
  const CameraExtrinsics extrinsics{{}, CameraModel::default_search_optical_quaternion()};
  const BearingCameraModel model(
    intrinsics, extrinsics,
    CameraFieldOfView{17.5 * M_PI / 180.0, 30.0 * M_PI / 180.0});
  const Vector3d boresight_body{
    std::sin(10.0 * M_PI / 180.0), 0.0, -std::cos(10.0 * M_PI / 180.0)};
  const auto observation = model.observe(boresight_body, Quaterniond{});
  EXPECT_TRUE(observation.visible);
  EXPECT_NEAR(observation.horizontal_angle_rad, 0.0, 1e-9);
  EXPECT_NEAR(observation.vertical_angle_rad, 0.0, 1e-9);
  EXPECT_NEAR(observation.unit_line_of_sight_optical.x, 0.0, 1e-9);
  EXPECT_NEAR(observation.unit_line_of_sight_optical.y, 0.0, 1e-9);
  EXPECT_NEAR(observation.unit_line_of_sight_optical.z, 1.0, 1e-9);
}

TEST(CameraModel, SearchCameraLongitudinalCoverageIsMinusTwentyToPlusFortyDegrees)
{
  const CameraIntrinsics intrinsics{320.0, 320.0, 319.5, 239.5, 640, 480, 0.05};
  const CameraExtrinsics extrinsics{{}, CameraModel::default_search_optical_quaternion()};
  const BearingCameraModel model(
    intrinsics, extrinsics,
    CameraFieldOfView{17.5 * M_PI / 180.0, 30.0 * M_PI / 180.0});
  const auto direction = [](double forward_angle_deg) {
      const double angle = forward_angle_deg * M_PI / 180.0;
      return Vector3d{std::sin(angle), 0.0, -std::cos(angle)};
    };
  EXPECT_TRUE(model.observe(direction(-19.9), Quaterniond{}).visible);
  EXPECT_TRUE(model.observe(direction(39.9), Quaterniond{}).visible);
  EXPECT_FALSE(model.observe(direction(-20.1), Quaterniond{}).visible);
  EXPECT_FALSE(model.observe(direction(40.1), Quaterniond{}).visible);
}
