#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "px4_offboard_tracking/capture_window.hpp"

using px4_offboard_tracking::CaptureWindow;
using px4_offboard_tracking::CaptureWindowConfig;
using px4_offboard_tracking::CaptureWindowInput;

namespace
{
CaptureWindowInput nominal(double time, double observation_time = -1.0)
{
  CaptureWindowInput input;
  input.timestamp_s = time;
  input.observation_valid = true;
  input.observation_timestamp_s = observation_time > 0.0 ? observation_time : time;
  input.quality_gate_passed = true;
  input.attack_geometry_valid = true;
  input.pixel_error_px = 20.0;
  input.alignment_error_rad = 0.05;
  input.los_rate_rad_s = 0.05;
  input.predicted_miss_distance_m = 6.5;
  input.time_to_closest_approach_s = 0.5;
  return input;
}
}  // namespace

TEST(CaptureWindow, RequiresContinuousHoldAndEightDistinctFrames)
{
  CaptureWindow window(CaptureWindowConfig{});
  for (int index = 0; index < 8; ++index) {
    const auto result = window.update(nominal(1.0 + 0.05 * index));
    EXPECT_FALSE(result.held);
  }
  const auto result = window.update(nominal(1.4));
  EXPECT_TRUE(result.instantaneous_pass);
  EXPECT_TRUE(result.held);
  EXPECT_TRUE(result.frame_count_ok);
  EXPECT_EQ(result.distinct_observation_frames, 9);
  EXPECT_NEAR(result.hold_elapsed_s, 0.4, 1e-9);
}

TEST(CaptureWindow, DoesNotCountRepeatedControlUseOfSameObservation)
{
  CaptureWindow window(CaptureWindowConfig{});
  for (int index = 0; index <= 50; ++index) {
    const auto result = window.update(nominal(1.0 + 0.01 * index, 1.0));
    EXPECT_FALSE(result.held);
    EXPECT_EQ(result.distinct_observation_frames, 1);
  }
}

TEST(CaptureWindow, RequiresBothDurationAndFrameCount)
{
  CaptureWindow window(CaptureWindowConfig{});
  for (int index = 0; index < 8; ++index) {
    EXPECT_FALSE(window.update(nominal(1.0 + 0.04 * index)).held);
  }
  const auto too_short = window.update(nominal(1.35));
  EXPECT_TRUE(too_short.frame_count_ok);
  EXPECT_FALSE(too_short.held);

  window.reset();
  for (int index = 0; index < 7; ++index) {
    EXPECT_FALSE(window.update(nominal(2.0 + 0.07 * index)).held);
  }
  const auto too_few_frames = window.update(nominal(2.5, 2.42));
  EXPECT_FALSE(too_few_frames.frame_count_ok);
  EXPECT_FALSE(too_few_frames.held);
}

TEST(CaptureWindow, AnyFailedConditionResetsHold)
{
  CaptureWindow window(CaptureWindowConfig{});
  window.update(nominal(1.0));
  auto failed = nominal(1.4);
  failed.pixel_error_px = 100.0;
  EXPECT_FALSE(window.update(failed).instantaneous_pass);
  EXPECT_FALSE(window.update(nominal(1.6)).held);
  EXPECT_EQ(window.update(nominal(1.65)).distinct_observation_frames, 2);
}

TEST(CaptureWindow, RejectsInvalidAttackGeometryAndPrediction)
{
  CaptureWindow window(CaptureWindowConfig{});
  auto input = nominal(1.0);
  input.attack_geometry_valid = false;
  input.predicted_miss_distance_m = 9.0;
  const auto result = window.update(input);
  EXPECT_FALSE(result.range_ok);
  EXPECT_FALSE(result.prediction_ok);
  EXPECT_FALSE(result.held);
}

TEST(CaptureWindow, RelativeSpeedIsNotACaptureGate)
{
  CaptureWindow window(CaptureWindowConfig{});
  auto input = nominal(1.0);
  const auto result = window.update(input);
  EXPECT_TRUE(result.relative_speed_ok);
  EXPECT_TRUE(result.instantaneous_pass);
}

TEST(CaptureWindow, NonMonotonicTimestampResetsWindow)
{
  CaptureWindow window(CaptureWindowConfig{});
  window.update(nominal(2.0));
  EXPECT_FALSE(window.update(nominal(1.0)).held);
  EXPECT_FALSE(window.update(nominal(2.1)).held);
}

TEST(CaptureWindow, ObservationTimestampRollbackResetsWindow)
{
  CaptureWindow window(CaptureWindowConfig{});
  window.update(nominal(2.0, 10.0));
  EXPECT_EQ(window.update(nominal(2.1, 10.1)).distinct_observation_frames, 2);
  const auto rollback = window.update(nominal(2.2, 9.0));
  EXPECT_FALSE(rollback.instantaneous_pass);
  EXPECT_EQ(rollback.distinct_observation_frames, 0);
  EXPECT_EQ(window.update(nominal(2.3, 9.1)).distinct_observation_frames, 1);
}

TEST(CaptureWindow, RejectsInvalidConfiguration)
{
  auto config = CaptureWindowConfig{};
  config.maximum_los_rate_rad_s = 0.0;
  EXPECT_THROW({CaptureWindow window(config);}, std::invalid_argument);
  config = CaptureWindowConfig{};
  config.minimum_distinct_observation_frames = 0;
  EXPECT_THROW({CaptureWindow window(config);}, std::invalid_argument);
}
