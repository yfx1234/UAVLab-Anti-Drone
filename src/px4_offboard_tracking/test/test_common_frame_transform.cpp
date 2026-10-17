#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "px4_offboard_tracking/common_frame_transform.hpp"

using px4_offboard_tracking::CommonFrameTransform;
using px4_offboard_tracking::GeodeticPosition;
using px4_offboard_tracking::NedPosition;

TEST(CommonFrameTransform, ReferenceMapsToNedOrigin)
{
  const GeodeticPosition reference{47.397742, 8.545594, 488.0};
  const CommonFrameTransform transform(reference);
  const auto ned = transform.to_ned(reference);
  EXPECT_NEAR(ned.north_m, 0.0, 1e-9);
  EXPECT_NEAR(ned.east_m, 0.0, 1e-9);
  EXPECT_NEAR(ned.down_m, 0.0, 1e-9);
}

TEST(CommonFrameTransform, ConvertsSmallWgs84OffsetToNed)
{
  constexpr double deg_per_rad = 180.0 / M_PI;
  constexpr double earth_radius_m = 6378137.0;
  const GeodeticPosition reference{47.0, 8.0, 500.0};
  const double latitude_rad = reference.latitude_deg / deg_per_rad;
  const GeodeticPosition displaced{
    reference.latitude_deg + 1.0 / earth_radius_m * deg_per_rad,
    reference.longitude_deg + 2.0 / (earth_radius_m * std::cos(latitude_rad)) * deg_per_rad,
    reference.altitude_ellipsoid_m + 3.0};

  const auto ned = CommonFrameTransform(reference).to_ned(displaced);
  EXPECT_NEAR(ned.north_m, 1.0, 0.01);
  EXPECT_NEAR(ned.east_m, 2.0, 0.01);
  EXPECT_NEAR(ned.down_m, -3.0, 0.01);
}

TEST(CommonFrameTransform, RejectsInvalidGeodeticData)
{
  const GeodeticPosition invalid{
    std::numeric_limits<double>::quiet_NaN(), 8.0, 500.0};
  EXPECT_FALSE(CommonFrameTransform::is_valid(invalid));
  EXPECT_THROW((void)CommonFrameTransform{invalid}, std::invalid_argument);
}

TEST(CommonFrameTransform, RoundTripsTaskOriginOffset)
{
  const GeodeticPosition reference{31.2304, 121.4737, 42.0};
  const CommonFrameTransform transform(reference);
  const NedPosition expected{23.5, -17.25, -6.0};

  const auto geodetic = transform.from_ned(expected);
  const auto actual = transform.to_ned(geodetic);

  EXPECT_NEAR(actual.north_m, expected.north_m, 1.0e-4);
  EXPECT_NEAR(actual.east_m, expected.east_m, 1.0e-4);
  EXPECT_NEAR(actual.down_m, expected.down_m, 1.0e-4);
}
