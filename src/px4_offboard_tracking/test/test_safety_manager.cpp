#include <gtest/gtest.h>

#include <stdexcept>

#include "px4_offboard_tracking/safety_manager.hpp"

using px4_offboard_tracking::SafetyManager;
using px4_offboard_tracking::SafetyManagerConfig;
using px4_offboard_tracking::SafetyManagerInput;

namespace
{
SafetyManagerInput nominal(double time_s)
{
  SafetyManagerInput input;
  input.timestamp_s = time_s;
  input.monitoring_active = true;
  input.local_state_valid = true;
  input.attitude_valid = true;
  input.range_valid = true;
  input.range_m = 7.0;
  input.tilt_rad = 0.05;
  input.speed_mps = 0.2;
  input.altitude_error_m = 0.1;
  return input;
}
}  // namespace

TEST(SafetyManager, AcceptsNominalFlight)
{
  SafetyManager manager(SafetyManagerConfig{});
  const auto result = manager.update(nominal(1.0));
  EXPECT_FALSE(result.abort_requested);
  EXPECT_TRUE(result.local_state_ok);
  EXPECT_TRUE(result.range_ok);
}

TEST(SafetyManager, ReportsIndependentAbortCauses)
{
  SafetyManager manager(SafetyManagerConfig{});
  auto input = nominal(1.0);
  input.range_m = 1.0;
  input.tilt_rad = 0.6;
  input.speed_mps = 5.0;
  input.altitude_error_m = 21.0;
  const auto result = manager.update(input);
  EXPECT_TRUE(result.abort_requested);
  EXPECT_FALSE(result.range_ok);
  EXPECT_FALSE(result.tilt_ok);
  EXPECT_FALSE(result.speed_ok);
  EXPECT_FALSE(result.altitude_ok);
}

TEST(SafetyManager, MissingRangeDoesNotInventCollision)
{
  SafetyManager manager(SafetyManagerConfig{});
  auto input = nominal(1.0);
  input.range_valid = false;
  EXPECT_FALSE(manager.update(input).abort_requested);
}

TEST(SafetyManager, FreshDepthBelowFiveMetersRequestsCollisionExit)
{
  SafetyManager manager(SafetyManagerConfig{});
  auto input = nominal(1.0);
  input.range_m = 4.99;
  const auto result = manager.update(input);
  EXPECT_TRUE(result.abort_requested);
  EXPECT_FALSE(result.range_ok);
  EXPECT_EQ(result.reason, "minimum range violated");
}

TEST(SafetyManager, InvalidOwnStateRequestsAbort)
{
  SafetyManager manager(SafetyManagerConfig{});
  auto input = nominal(1.0);
  input.local_state_valid = false;
  input.attitude_valid = false;
  const auto result = manager.update(input);
  EXPECT_TRUE(result.abort_requested);
  EXPECT_FALSE(result.local_state_ok);
  EXPECT_FALSE(result.attitude_ok);
}

TEST(SafetyManager, RecoveryRequiresContinuousHold)
{
  SafetyManager manager(SafetyManagerConfig{});
  auto input = nominal(1.0);
  input.recovery_active = true;
  EXPECT_FALSE(manager.update(input).recovery_complete);
  input.timestamp_s = 1.6;
  EXPECT_FALSE(manager.update(input).recovery_complete);
  input.timestamp_s = 2.0;
  EXPECT_TRUE(manager.update(input).recovery_complete);
  input.speed_mps = 1.0;
  input.timestamp_s = 2.1;
  EXPECT_FALSE(manager.update(input).recovery_complete);
}

TEST(SafetyManager, TimestampRollbackResetsRecovery)
{
  SafetyManager manager(SafetyManagerConfig{});
  auto input = nominal(2.0);
  input.recovery_active = true;
  manager.update(input);
  input.timestamp_s = 1.0;
  const auto result = manager.update(input);
  EXPECT_TRUE(result.abort_requested);
  EXPECT_FALSE(result.recovery_complete);
}

TEST(SafetyManager, RejectsInvalidConfiguration)
{
  auto config = SafetyManagerConfig{};
  config.recovery_maximum_tilt_rad = config.maximum_tilt_rad + 0.1;
  EXPECT_THROW({SafetyManager manager(config);}, std::invalid_argument);
}
