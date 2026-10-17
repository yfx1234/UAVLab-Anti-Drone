#pragma once

#include <array>

namespace px4_offboard_tracking
{

struct Vector3d
{
  double x{};
  double y{};
  double z{};
};

struct Quaterniond
{
  double w{1.0};
  double x{};
  double y{};
  double z{};
};

struct CameraIntrinsics
{
  double fx{};
  double fy{};
  double cx{};
  double cy{};
  int width{};
  int height{};
  double minimum_depth_m{0.05};
};

struct CameraExtrinsics
{
  // Camera origin relative to the body origin, expressed in body FRD.
  Vector3d translation_body_frd_m{};
  // Hamilton quaternion rotating body FRD vectors into camera optical vectors.
  Quaterniond q_body_to_optical{};
};

struct CameraFieldOfView
{
  double horizontal_half_angle_rad{};
  double vertical_half_angle_rad{};
};

struct PinholeProjection
{
  Vector3d position_camera{};
  double pixel_u{};
  double pixel_v{};
  double depth{};
  double range_m{};
  bool visible{false};
};

struct BearingObservation
{
  double pixel_u{};
  double pixel_v{};
  double horizontal_angle_rad{};
  double vertical_angle_rad{};
  Vector3d unit_line_of_sight_optical{};
  bool visible{false};
};

class CameraModel
{
public:
  CameraModel(
    const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
    const CameraFieldOfView & field_of_view = {});

  PinholeProjection project(
    const Vector3d & target_relative_ned,
    const Quaterniond & q_body_to_ned) const;

  Vector3d camera_position_to_relative_ned(
    const Vector3d & position_camera,
    const Quaterniond & q_body_to_ned) const;

  Quaterniond q_camera_to_ned(const Quaterniond & q_body_to_ned) const;

  static Quaterniond default_upward_optical_quaternion();
  static Quaterniond default_search_optical_quaternion();

private:
  static Quaterniond normalize(const Quaterniond & quaternion);
  static Quaterniond conjugate(const Quaterniond & quaternion);
  static Quaterniond multiply(const Quaterniond & first, const Quaterniond & second);
  static Vector3d rotate(const Quaterniond & quaternion, const Vector3d & vector);

  CameraIntrinsics intrinsics_;
  CameraExtrinsics extrinsics_;
  CameraFieldOfView field_of_view_;
};

// Bearing-only logical camera. Its public observation deliberately contains no
// depth, range, or three-dimensional target position.
class BearingCameraModel
{
public:
  BearingCameraModel(
    const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
    const CameraFieldOfView & field_of_view);

  BearingObservation observe(
    const Vector3d & target_relative_ned,
    const Quaterniond & q_body_to_ned) const;

private:
  CameraModel projection_model_;
};

}  // namespace px4_offboard_tracking
