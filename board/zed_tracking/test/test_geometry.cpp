// geometry.hpp 单元测试:坐标链、伪测距、letterbox 逆映射。

#include <gtest/gtest.h>

#include <cmath>

#include "zed_tracking/geometry.hpp"

using zed_tracking::Quat;
using zed_tracking::Vec3;

TEST(Geometry, IdentityQuaternionKeepsVector)
{
  zed_tracking::Mat3 rotation{};
  zed_tracking::quaternion_to_matrix(Quat{1.0, 0.0, 0.0, 0.0}, rotation);
  const Vec3 rotated = zed_tracking::rotate(rotation, Vec3{1.0, -2.0, 3.5});
  EXPECT_NEAR(rotated.x, 1.0, 1e-12);
  EXPECT_NEAR(rotated.y, -2.0, 1e-12);
  EXPECT_NEAR(rotated.z, 3.5, 1e-12);
}

TEST(Geometry, QuarterTurnAroundZMapsXToY)
{
  const double half = std::sqrt(0.5);
  zed_tracking::Mat3 rotation{};
  zed_tracking::quaternion_to_matrix(Quat{half, 0.0, 0.0, half}, rotation);
  const Vec3 rotated = zed_tracking::rotate(rotation, Vec3{1.0, 0.0, 0.0});
  EXPECT_NEAR(rotated.x, 0.0, 1e-12);
  EXPECT_NEAR(rotated.y, 1.0, 1e-12);
  EXPECT_NEAR(rotated.z, 0.0, 1e-12);
}

TEST(Geometry, CameraToBodySkyMountDefaultExtrinsics)
{
  // 默认外参:单位旋转 + 平移 (0.10, 0, 0.12)。
  // 目标在画面中心、深度 5 m -> 相机系 (0,0,5) -> 机体 (0.10, 0, 5.12)。
  const Vec3 camera_point = zed_tracking::pixel_to_camera(
    /*u=*/960.0, /*v=*/540.0, /*depth=*/5.0,
    /*fx=*/1000.0, /*fy=*/1000.0, /*cx=*/960.0, /*cy=*/540.0);
  ASSERT_NEAR(camera_point.x, 0.0, 1e-12);
  ASSERT_NEAR(camera_point.z, 5.0, 1e-12);
  const Vec3 body_point = zed_tracking::camera_to_body(
    Quat{1.0, 0.0, 0.0, 0.0}, Vec3{0.10, 0.0, 0.12}, camera_point);
  EXPECT_NEAR(body_point.x, 0.10, 1e-12);
  EXPECT_NEAR(body_point.y, 0.0, 1e-12);
  EXPECT_NEAR(body_point.z, 5.12, 1e-12);
}

TEST(Geometry, OffCenterPixelUsesPinholeModel)
{
  // u 在光轴右侧 0.5 倍焦距 * 深度 2 m -> 相机 x = 1.0 m。
  const Vec3 camera_point = zed_tracking::pixel_to_camera(
    1460.0, 540.0, 2.0, 1000.0, 1000.0, 960.0, 540.0);
  EXPECT_NEAR(camera_point.x, 1.0, 1e-12);
  EXPECT_NEAR(camera_point.y, 0.0, 1e-12);
  EXPECT_NEAR(camera_point.z, 2.0, 1e-12);
}

TEST(Geometry, PseudoRangePhysicalScalesInverse)
{
  // 30 cm 目标、f=1105 px:10 m 处约 33 px;距离减半框加倍。
  EXPECT_NEAR(zed_tracking::pseudo_range_physical(0.30, 1105.0, 33.15), 10.0, 0.01);
  EXPECT_NEAR(zed_tracking::pseudo_range_physical(0.30, 1105.0, 16.575), 20.0, 0.01);
  EXPECT_DOUBLE_EQ(zed_tracking::pseudo_range_physical(0.30, 1105.0, 0.0), 0.0);
}

TEST(Geometry, PseudoRangeReferenceMatchesOldContract)
{
  // 同分辨率下退化为 ref_range * ref_bbox / bbox。
  const double range = zed_tracking::pseudo_range_reference(
    16.5, 1920.0, 10.0, 33.0, 1920.0);
  EXPECT_NEAR(range, 20.0, 1e-9);
  // 标定分辨率不同时按占幅比换算。
  const double cross = zed_tracking::pseudo_range_reference(
    11.0, 1280.0, 10.0, 33.0, 1920.0);
  EXPECT_NEAR(cross, 20.0, 1e-9);
}

TEST(Geometry, LetterboxPortraitSourceMapsCenterBack)
{
  // 竖图 1080x1920 -> 模型输入 1280x736(宽高倒置是常态,勿假设方向)。
  const auto box = zed_tracking::make_letterbox(1080, 1920, 1280, 736);
  const double scale = std::min(1280.0 / 1080.0, 736.0 / 1920.0);
  EXPECT_NEAR(box.scale, scale, 1e-9);
  // 源图整幅映射到输入图 [pad_x, pad_x + w*scale] x [pad_y, pad_y + h*scale]。
  double x1 = box.pad_x, y1 = box.pad_y;
  double x2 = box.pad_x + 1080 * scale, y2 = box.pad_y + 1920 * scale;
  zed_tracking::map_box_back(box, &x1, &y1, &x2, &y2);
  EXPECT_NEAR(x1, 0.0, 1e-6);
  EXPECT_NEAR(y1, 0.0, 1e-6);
  EXPECT_NEAR(x2, 1080.0, 1e-6);
  EXPECT_NEAR(y2, 1920.0, 1e-6);
}

TEST(Geometry, LetterboxCenterRoundTrip)
{
  const auto box = zed_tracking::make_letterbox(1080, 1920, 1280, 736);
  // 原图中心 (540, 960) 映射到输入图再映射回来。
  const double mapped_x = 540.0 * box.scale + box.pad_x;
  const double mapped_y = 960.0 * box.scale + box.pad_y;
  double x = mapped_x, y = mapped_y, x2 = mapped_x, y2 = mapped_y;
  zed_tracking::map_box_back(box, &x, &y, &x2, &y2);
  EXPECT_NEAR(x, 540.0, 1e-6);
  EXPECT_NEAR(y, 960.0, 1e-6);
}
