#include "px4_offboard_tracking/common_frame_transform.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84FirstEccentricitySquared = 6.69437999014e-3;
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

CommonFrameTransform::CommonFrameTransform(const GeodeticPosition & reference)
: reference_(reference)
{
  if (!is_valid(reference_)) {
    throw std::invalid_argument("invalid WGS84 common-frame reference");
  }

  reference_ecef_ = geodetic_to_ecef(reference_);
  const double latitude = reference_.latitude_deg * kDegToRad;
  const double longitude = reference_.longitude_deg * kDegToRad;
  sin_lat_ = std::sin(latitude);
  cos_lat_ = std::cos(latitude);
  sin_lon_ = std::sin(longitude);
  cos_lon_ = std::cos(longitude);
}

bool CommonFrameTransform::is_valid(const GeodeticPosition & position)
{
  return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg) &&
         std::isfinite(position.altitude_ellipsoid_m) &&
         position.latitude_deg >= -90.0 && position.latitude_deg <= 90.0 &&
         position.longitude_deg >= -180.0 && position.longitude_deg <= 180.0;
}

std::array<double, 3> CommonFrameTransform::geodetic_to_ecef(
  const GeodeticPosition & position)
{
  const double latitude = position.latitude_deg * kDegToRad;
  const double longitude = position.longitude_deg * kDegToRad;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double prime_vertical_radius = kWgs84SemiMajorAxisM /
    std::sqrt(1.0 - kWgs84FirstEccentricitySquared * sin_latitude * sin_latitude);

  return {
    (prime_vertical_radius + position.altitude_ellipsoid_m) * cos_latitude *
      std::cos(longitude),
    (prime_vertical_radius + position.altitude_ellipsoid_m) * cos_latitude *
      std::sin(longitude),
    (prime_vertical_radius * (1.0 - kWgs84FirstEccentricitySquared) +
      position.altitude_ellipsoid_m) * sin_latitude};
}

GeodeticPosition CommonFrameTransform::ecef_to_geodetic(
  const std::array<double, 3> & position)
{
  const double x = position[0];
  const double y = position[1];
  const double z = position[2];
  const double longitude = std::atan2(y, x);
  const double horizontal = std::hypot(x, y);
  double latitude = std::atan2(
    z, horizontal * (1.0 - kWgs84FirstEccentricitySquared));
  double altitude = 0.0;
  for (int iteration = 0; iteration < 8; ++iteration) {
    const double sin_latitude = std::sin(latitude);
    const double prime_vertical_radius = kWgs84SemiMajorAxisM /
      std::sqrt(
      1.0 - kWgs84FirstEccentricitySquared * sin_latitude * sin_latitude);
    altitude = horizontal / std::max(1.0e-12, std::cos(latitude)) -
      prime_vertical_radius;
    latitude = std::atan2(
      z,
      horizontal *
      (1.0 - kWgs84FirstEccentricitySquared *
      prime_vertical_radius / (prime_vertical_radius + altitude)));
  }
  const double sin_latitude = std::sin(latitude);
  const double prime_vertical_radius = kWgs84SemiMajorAxisM /
    std::sqrt(
    1.0 - kWgs84FirstEccentricitySquared * sin_latitude * sin_latitude);
  altitude = horizontal / std::max(1.0e-12, std::cos(latitude)) -
    prime_vertical_radius;
  return {
    latitude / kDegToRad,
    longitude / kDegToRad,
    altitude};
}

NedPosition CommonFrameTransform::to_ned(const GeodeticPosition & position) const
{
  if (!is_valid(position)) {
    throw std::invalid_argument("invalid WGS84 position");
  }

  const auto ecef = geodetic_to_ecef(position);
  const double dx = ecef[0] - reference_ecef_[0];
  const double dy = ecef[1] - reference_ecef_[1];
  const double dz = ecef[2] - reference_ecef_[2];

  return {
    -sin_lat_ * cos_lon_ * dx - sin_lat_ * sin_lon_ * dy + cos_lat_ * dz,
    -sin_lon_ * dx + cos_lon_ * dy,
    -cos_lat_ * cos_lon_ * dx - cos_lat_ * sin_lon_ * dy - sin_lat_ * dz};
}

GeodeticPosition CommonFrameTransform::from_ned(const NedPosition & position) const
{
  if (!std::isfinite(position.north_m) || !std::isfinite(position.east_m) ||
    !std::isfinite(position.down_m))
  {
    throw std::invalid_argument("invalid NED position");
  }
  // Inverse of the ECEF-to-NED rotation used by to_ned().
  const double dx =
    -sin_lat_ * cos_lon_ * position.north_m -
    sin_lon_ * position.east_m -
    cos_lat_ * cos_lon_ * position.down_m;
  const double dy =
    -sin_lat_ * sin_lon_ * position.north_m +
    cos_lon_ * position.east_m -
    cos_lat_ * sin_lon_ * position.down_m;
  const double dz =
    cos_lat_ * position.north_m - sin_lat_ * position.down_m;
  return ecef_to_geodetic({
    reference_ecef_[0] + dx,
    reference_ecef_[1] + dy,
    reference_ecef_[2] + dz});
}

}  // namespace px4_offboard_tracking
