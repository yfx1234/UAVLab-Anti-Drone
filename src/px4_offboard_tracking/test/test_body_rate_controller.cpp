#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/body_rate_controller.hpp"

using px4_offboard_tracking::AttitudeThrustCommand;
using px4_offboard_tracking::BodyRateController;
using px4_offboard_tracking::BodyRateControllerConfig;

TEST(BodyRateController, MatchingAttitudeCommandsZeroRates)
{
  BodyRateController controller(BodyRateControllerConfig{});
  AttitudeThrustCommand desired;
  desired.thrust = 0.7;
  const auto output = controller.update({}, desired);
  EXPECT_NEAR(output.roll_rate, 0.0, 1e-12);
  EXPECT_NEAR(output.pitch_rate, 0.0, 1e-12);
  EXPECT_NEAR(output.yaw_rate, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(output.thrust, 0.7);
}

TEST(BodyRateController, SeparatesYawAndRollPitchLimits)
{
  BodyRateControllerConfig config;
  config.maximum_roll_pitch_rate_rad_s = 1.0;
  config.maximum_yaw_rate_rad_s = 0.5;
  BodyRateController controller(config);
  AttitudeThrustCommand desired;
  desired.attitude = {0.0, std::sqrt(0.5), 0.0, std::sqrt(0.5)};
  const auto output = controller.update({}, desired);
  EXPECT_DOUBLE_EQ(output.roll_rate, 1.0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.5);
}

TEST(BodyRateController, QuaternionSignDoesNotChangeCommand)
{
  BodyRateController controller(BodyRateControllerConfig{});
  AttitudeThrustCommand first;
  first.attitude = {std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0};
  auto second = first;
  second.attitude = {-first.attitude.w, -first.attitude.x, 0.0, 0.0};
  EXPECT_NEAR(
    controller.update({}, first).roll_rate,
    controller.update({}, second).roll_rate, 1e-12);
}

TEST(BodyRateController, RejectsInvalidConfigurationAndQuaternion)
{
  auto config = BodyRateControllerConfig{};
  config.yaw_gain = 0.0;
  EXPECT_THROW({BodyRateController controller(config);}, std::invalid_argument);
  BodyRateController controller(BodyRateControllerConfig{});
  AttitudeThrustCommand desired;
  EXPECT_THROW(controller.update({0.0, 0.0, 0.0, 0.0}, desired), std::invalid_argument);
}
