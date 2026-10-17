// 几何纯函数库:像素/相机系/机体系换算与伪测距。
//
// 坐标约定(与外参配置文件一致,勿混用):
//   相机光学系(ZED 惯例): x 右, y 下, z 前(沿光轴)。
//   机体系:               x 前(机头), y 左, z 上。
//   四元数一律 wxyz,使用前归一化。
// 本头文件不依赖 ROS / ZED SDK / OpenCV,便于离线单元测试。

#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zed_tracking
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

using Mat3 = double[3][3];

struct Quat
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Quat normalized_quat(Quat q)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm < 1.0e-12 || !std::isfinite(norm)) {
    throw std::invalid_argument("quaternion must be finite and non-zero");
  }
  return Quat{q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

inline void quaternion_to_matrix(const Quat & q, Mat3 out)
{
  const Quat n = normalized_quat(q);
  const double w = n.w, x = n.x, y = n.y, z = n.z;
  out[0][0] = 1.0 - 2.0 * (y * y + z * z);
  out[0][1] = 2.0 * (x * y - z * w);
  out[0][2] = 2.0 * (x * z + y * w);
  out[1][0] = 2.0 * (x * y + z * w);
  out[1][1] = 1.0 - 2.0 * (x * x + z * z);
  out[1][2] = 2.0 * (y * z - x * w);
  out[2][0] = 2.0 * (x * z - y * w);
  out[2][1] = 2.0 * (y * z + x * w);
  out[2][2] = 1.0 - 2.0 * (x * x + y * y);
}

inline Vec3 rotate(const Mat3 rotation, const Vec3 & v)
{
  return Vec3{
    rotation[0][0] * v.x + rotation[0][1] * v.y + rotation[0][2] * v.z,
    rotation[1][0] * v.x + rotation[1][1] * v.y + rotation[1][2] * v.z,
    rotation[2][0] * v.x + rotation[2][1] * v.y + rotation[2][2] * v.z};
}

// 相机系 -> 机体系刚体变换:body = R_body_from_camera * camera + t。
// t 的单位是米,在机体系下表达(相机偏心杆臂)。
inline Vec3 camera_to_body(
  const Quat & q_body_from_camera, const Vec3 & camera_translation_body,
  const Vec3 & camera_point)
{
  Mat3 rotation{};
  quaternion_to_matrix(q_body_from_camera, rotation);
  const Vec3 rotated = rotate(rotation, camera_point);
  return Vec3{
    rotated.x + camera_translation_body.x,
    rotated.y + camera_translation_body.y,
    rotated.z + camera_translation_body.z};
}

// 像素 + 深度 -> 相机光学系点(米)。u/v 为全分辨率像素坐标(左上原点)。
inline Vec3 pixel_to_camera(
  double u, double v, double depth_m, double fx, double fy, double cx, double cy)
{
  return Vec3{(u - cx) / fx * depth_m, (v - cy) / fy * depth_m, depth_m};
}

// 参考式伪测距:沿用旧链路标定合同——参考框占幅比 / 当前框占幅比 * 参考距离。
// 占幅比在两次标定分辨率不同时不可约去,因此保留 image_width 项。
inline double pseudo_range_reference(
  double bbox_long_px, double image_width_px,
  double reference_range_m, double reference_bbox_px, double reference_image_width_px)
{
  if (bbox_long_px <= 0.0 || image_width_px <= 0.0 || reference_range_m <= 0.0 ||
    reference_bbox_px <= 0.0 || reference_image_width_px <= 0.0)
  {
    return 0.0;
  }
  const double fraction = bbox_long_px / image_width_px;
  const double reference_fraction = reference_bbox_px / reference_image_width_px;
  return reference_range_m * reference_fraction / fraction;
}

// 物理式伪测距:距离 = 目标实际尺寸 * 焦距像素 / 框长边像素。
// 假设目标长边在画面中与机身长边一致(朝天视角下手飞目标的近似)。
inline double pseudo_range_physical(
  double target_size_m, double focal_length_px, double bbox_long_px)
{
  if (target_size_m <= 0.0 || focal_length_px <= 0.0 || bbox_long_px <= 0.0) {
    return 0.0;
  }
  return target_size_m * focal_length_px / bbox_long_px;
}

// letterbox 逆映射:模型输入坐标 -> 原图坐标。
struct Letterbox
{
  double scale{1.0};
  int pad_x{0};
  int pad_y{0};
};

inline Letterbox make_letterbox(
  int src_w, int src_h, int dst_w, int dst_h)
{
  const double scale = std::min(
    static_cast<double>(dst_w) / src_w, static_cast<double>(dst_h) / src_h);
  const int scaled_w = std::max(1, static_cast<int>(std::round(src_w * scale)));
  const int scaled_h = std::max(1, static_cast<int>(std::round(src_h * scale)));
  return Letterbox{scale, (dst_w - scaled_w) / 2, (dst_h - scaled_h) / 2};
}

inline void map_box_back(
  const Letterbox & box, double * x1, double * y1, double * x2, double * y2)
{
  *x1 = (*x1 - box.pad_x) / box.scale;
  *y1 = (*y1 - box.pad_y) / box.scale;
  *x2 = (*x2 - box.pad_x) / box.scale;
  *y2 = (*y2 - box.pad_y) / box.scale;
}

}  // namespace zed_tracking
