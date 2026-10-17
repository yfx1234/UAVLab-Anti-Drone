#pragma once

#include <array>

namespace px4_offboard_tracking
{

struct GeodeticPosition
{
  double latitude_deg{};
  double longitude_deg{};
  double altitude_ellipsoid_m{};
};

struct NedPosition
{
  double north_m{};
  double east_m{};
  double down_m{};
};

class CommonFrameTransform
{
public:
  explicit CommonFrameTransform(const GeodeticPosition & reference);

  NedPosition to_ned(const GeodeticPosition & position) const;
  GeodeticPosition from_ned(const NedPosition & position) const;

  const GeodeticPosition & reference() const {return reference_;}

  static bool is_valid(const GeodeticPosition & position);

private:
  static std::array<double, 3> geodetic_to_ecef(const GeodeticPosition & position);
  static GeodeticPosition ecef_to_geodetic(const std::array<double, 3> & position);

  GeodeticPosition reference_;
  std::array<double, 3> reference_ecef_{};
  double sin_lat_{};
  double cos_lat_{};
  double sin_lon_{};
  double cos_lon_{};
};

}  // namespace px4_offboard_tracking
