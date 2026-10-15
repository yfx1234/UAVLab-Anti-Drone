#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "px4_offboard_tracking/attitude_setpoint_generator.hpp"
#include "px4_offboard_tracking/z_axis_alignment.hpp"

using px4_offboard_tracking::AttitudeSetpointGenerator;
using px4_offboard_tracking::AttitudeSetpointGeneratorConfig;
using px4_offboard_tracking::Quaternion;
using px4_offboard_tracking::ZAxisAlignment;

namespace
{
double quaternion_distance(const Quaternion & first, const Quaternion & second)
{
  const double dot = std::abs(
    first.w * second.w + first.x * second.x + first.y * second.y + first.z * second.z);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}
}  // namespace

TEST(AttitudeSetpointGenerator, VerticalAxisProducesLevelHover)
{
  const AttitudeSetpointGeneratorConfig config{};
  AttitudeSetpointGenerator generator(config);
  const auto output = generator.update({0.0, 0.0, -1.0}, 0.0, 0.0, Quaternion{}, 0.02);
  EXPECT_NEAR(output.roll, 0.0, 1e-12);
  EXPECT_NEAR(output.pitch, 0.0, 1e-12);
  EXPECT_NEAR(output.thrust, config.hover_thrust, 1e-12);
}

TEST(AttitudeSetpointGenerator, SlerpLimitsAngularStepAndMaintainsQuaternionSign)
{
  AttitudeSetpointGeneratorConfig config;
  config.smoothing_time_constant_s = 0.01;
  config.maximum_attitude_slew_rate_rad_s = 0.5;
  AttitudeSetpointGenerator generator(config);
  const Quaternion current{};
  const auto output = generator.update({0.5, 0.0, -0.866025403784}, 0.0, 0.0, current, 0.1);
  EXPECT_LE(quaternion_distance(current, output.attitude), 0.05 + 1e-12);
  EXPECT_GE(output.attitude.w, 0.0);
}

TEST(AttitudeSetpointGenerator, TiltCompensationIncreasesCollectiveThrustSmoothly)
{
  AttitudeSetpointGeneratorConfig config;
  config.smoothing_time_constant_s = 0.05;
  config.maximum_attitude_slew_rate_rad_s = 10.0;
  AttitudeSetpointGenerator generator(config);
  px4_offboard_tracking::AttitudeThrustCommand output;
  for (int index = 0; index < 30; ++index) {
    output = generator.update({0.342, 0.0, -0.9397}, 0.0, 0.0, Quaternion{}, 0.02);
  }
  EXPECT_GT(output.thrust, config.hover_thrust);
  EXPECT_LE(output.thrust, config.maximum_thrust);
  const auto axis = ZAxisAlignment::body_minus_z_axis_ned(output.attitude);
  EXPECT_GT(axis[0], 0.0);
  EXPECT_LT(axis[2], 0.0);
}
