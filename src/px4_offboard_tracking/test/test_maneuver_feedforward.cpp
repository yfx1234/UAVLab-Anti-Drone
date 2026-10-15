#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/maneuver_feedforward.hpp"

using px4_offboard_tracking::ManeuverFeedforward;
using px4_offboard_tracking::ManeuverFeedforwardConfig;

TEST(ManeuverFeedforward, SuppressesConstantVelocityAndUncertainEstimates)
{
  ManeuverFeedforward feedforward(ManeuverFeedforwardConfig{});
  const auto cv = feedforward.update({3.0, 0.0, 0.0}, 0.10, 1.0, true, 0.02);
  EXPECT_FALSE(cv.active);
  EXPECT_DOUBLE_EQ(cv.confidence, 0.0);

  const auto uncertain = feedforward.update({3.0, 0.0, 0.0}, 0.95, 8.0, true, 0.02);
  EXPECT_FALSE(uncertain.active);
  EXPECT_DOUBLE_EQ(uncertain.confidence, 0.0);
}

TEST(ManeuverFeedforward, EnablesBoundedConfidentManeuver)
{
  ManeuverFeedforward feedforward(ManeuverFeedforwardConfig{});
  px4_offboard_tracking::ManeuverFeedforwardResult output;
  for (int sample = 0; sample < 50; ++sample) {
    output = feedforward.update({8.0, 0.0, 0.0}, 0.90, 2.0, true, 0.02);
  }
  EXPECT_TRUE(output.active);
  EXPECT_GT(output.confidence, 0.70);
  EXPECT_LE(std::abs(output.filtered_target_acceleration_ned[0]), 4.0);
  EXPECT_NEAR(output.filtered_target_acceleration_ned[0], 4.0, 0.05);
}

TEST(ManeuverFeedforward, DecaysWhenEstimateIsLost)
{
  ManeuverFeedforward feedforward(ManeuverFeedforwardConfig{});
  for (int sample = 0; sample < 50; ++sample) {
    (void)feedforward.update({2.0, 0.0, 0.0}, 0.90, 1.0, true, 0.02);
  }
  const auto lost = feedforward.update({2.0, 0.0, 0.0}, 0.90, 1.0, false, 0.02);
  EXPECT_FALSE(lost.active);
  EXPECT_DOUBLE_EQ(lost.confidence, 0.0);
  EXPECT_LT(lost.filtered_target_acceleration_ned[0], 2.0);
  EXPECT_GT(lost.filtered_target_acceleration_ned[0], 0.0);
}

TEST(ManeuverFeedforward, RejectsInvalidConfigurationAndTimeStep)
{
  auto config = ManeuverFeedforwardConfig{};
  config.full_ca_probability = config.minimum_ca_probability;
  EXPECT_THROW(
    {
      ManeuverFeedforward invalid(config);
      (void)invalid;
    },
    std::invalid_argument);

  ManeuverFeedforward feedforward(ManeuverFeedforwardConfig{});
  EXPECT_THROW(
    feedforward.update({0.0, 0.0, 0.0}, 0.5, 1.0, true, 0.0),
    std::invalid_argument);
}
