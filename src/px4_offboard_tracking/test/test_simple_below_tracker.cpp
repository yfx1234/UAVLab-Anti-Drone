#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/simple_below_tracker.hpp"

namespace px4_offboard_tracking
{
namespace
{

SimpleBelowTrackerConfig unrestricted_config()
{
  SimpleBelowTrackerConfig config;
  config.horizontal_position_gain = 1.0;
  config.prediction_horizon_s = 0.2;
  config.relative_velocity_filter_time_constant_s = 0.0;
  config.maximum_relative_speed_mps = 100.0;
  config.maximum_prediction_distance_m = 100.0;
  config.maximum_horizontal_speed_mps = 100.0;
  config.maximum_horizontal_acceleration_mps2 = 1000.0;
  return config;
}

TEST(SimpleBelowTracker, ParsesOnlySupportedModes)
{
  EXPECT_EQ(
    parse_simple_below_tracking_mode("direct"), SimpleBelowTrackingMode::DIRECT);
  EXPECT_EQ(
    parse_simple_below_tracking_mode("predictive"),
    SimpleBelowTrackingMode::PREDICTIVE);
  EXPECT_THROW(parse_simple_below_tracking_mode("mpc"), std::invalid_argument);
}

TEST(SimpleBelowTracker, DirectModeUsesCurrentHorizontalObservation)
{
  SimpleBelowTracker tracker(unrestricted_config());
  ASSERT_TRUE(tracker.update_observation({2.0, -1.0}, 1.0));

  const auto result = tracker.step(SimpleBelowTrackingMode::DIRECT, true, 0.02, {});

  EXPECT_TRUE(result.observation_valid);
  EXPECT_FALSE(result.relative_velocity_valid);
  EXPECT_DOUBLE_EQ(result.predicted_relative_position_ned_m[0], 2.0);
  EXPECT_DOUBLE_EQ(result.predicted_relative_position_ned_m[1], -1.0);
  EXPECT_DOUBLE_EQ(result.velocity_setpoint_ned_mps[0], 2.0);
  EXPECT_DOUBLE_EQ(result.velocity_setpoint_ned_mps[1], -1.0);
}

TEST(SimpleBelowTracker, PredictiveModeLeadsByConfiguredHorizon)
{
  SimpleBelowTracker tracker(unrestricted_config());
  ASSERT_TRUE(tracker.update_observation({1.0, 0.0}, 1.0));
  ASSERT_TRUE(tracker.update_observation({1.1, -0.2}, 1.1));

  const auto result = tracker.step(
    SimpleBelowTrackingMode::PREDICTIVE, true, 0.02, {});

  ASSERT_TRUE(result.relative_velocity_valid);
  EXPECT_NEAR(result.relative_velocity_ned_mps[0], 1.0, 1e-12);
  EXPECT_NEAR(result.relative_velocity_ned_mps[1], -2.0, 1e-12);
  EXPECT_NEAR(result.predicted_relative_position_ned_m[0], 1.3, 1e-12);
  EXPECT_NEAR(result.predicted_relative_position_ned_m[1], -0.6, 1e-12);
  EXPECT_NEAR(result.velocity_setpoint_ned_mps[0], 1.3, 1e-12);
  EXPECT_NEAR(result.velocity_setpoint_ned_mps[1], -0.6, 1e-12);
}

TEST(SimpleBelowTracker, PredictsTargetMotionAfterRelativeVelocityConvergesToZero)
{
  SimpleBelowTracker tracker(unrestricted_config());
  ASSERT_TRUE(tracker.update_observation({2.0, 0.0}, 1.0));
  ASSERT_TRUE(tracker.update_observation({2.0, 0.0}, 1.1));

  const auto result = tracker.step(
    SimpleBelowTrackingMode::PREDICTIVE, true, 0.02, {1.2, 0.0});

  ASSERT_TRUE(result.relative_velocity_valid);
  EXPECT_NEAR(result.relative_velocity_ned_mps[0], 0.0, 1e-12);
  EXPECT_NEAR(result.estimated_target_velocity_ned_mps[0], 1.2, 1e-12);
  EXPECT_NEAR(result.predicted_relative_position_ned_m[0], 2.24, 1e-12);
}

TEST(SimpleBelowTracker, DisabledTrackingSmoothlyReturnsCommandToZero)
{
  auto config = unrestricted_config();
  config.maximum_horizontal_acceleration_mps2 = 1.0;
  SimpleBelowTracker tracker(config);
  ASSERT_TRUE(tracker.update_observation({10.0, 0.0}, 1.0));

  const auto active = tracker.step(SimpleBelowTrackingMode::DIRECT, true, 0.1, {});
  EXPECT_NEAR(active.velocity_setpoint_ned_mps[0], 0.1, 1e-12);
  const auto disabled = tracker.step(SimpleBelowTrackingMode::DIRECT, false, 0.05, {});
  EXPECT_NEAR(disabled.velocity_setpoint_ned_mps[0], 0.05, 1e-12);
  const auto stopped = tracker.step(SimpleBelowTrackingMode::DIRECT, false, 0.05, {});
  EXPECT_NEAR(stopped.velocity_setpoint_ned_mps[0], 0.0, 1e-12);
}

TEST(SimpleBelowTracker, LimitsPredictionSpeedAndAcceleration)
{
  auto config = unrestricted_config();
  config.maximum_prediction_distance_m = 0.5;
  config.maximum_horizontal_speed_mps = 2.0;
  config.maximum_horizontal_acceleration_mps2 = 1.0;
  SimpleBelowTracker tracker(config);
  ASSERT_TRUE(tracker.update_observation({0.0, 0.0}, 1.0));
  ASSERT_TRUE(tracker.update_observation({1.0, 0.0}, 1.1));

  const auto result = tracker.step(
    SimpleBelowTrackingMode::PREDICTIVE, true, 0.1, {});

  EXPECT_NEAR(result.predicted_relative_position_ned_m[0], 1.5, 1e-12);
  EXPECT_NEAR(std::hypot(
      result.velocity_setpoint_ned_mps[0], result.velocity_setpoint_ned_mps[1]),
    0.1, 1e-12);
}

TEST(SimpleBelowTracker, RejectsOutOfOrderObservation)
{
  SimpleBelowTracker tracker(unrestricted_config());
  ASSERT_TRUE(tracker.update_observation({1.0, 0.0}, 2.0));
  EXPECT_FALSE(tracker.update_observation({2.0, 0.0}, 1.9));

  const auto result = tracker.step(SimpleBelowTrackingMode::DIRECT, true, 0.02, {});
  EXPECT_DOUBLE_EQ(result.relative_position_ned_m[0], 1.0);
}

}  // namespace
}  // namespace px4_offboard_tracking
