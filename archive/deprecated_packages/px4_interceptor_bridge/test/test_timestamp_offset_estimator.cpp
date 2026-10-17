#include <gtest/gtest.h>

#include "px4_interceptor_bridge/timestamp_offset_estimator.hpp"

using px4_interceptor_bridge::TimestampOffsetEstimator;

TEST(TimestampOffsetEstimator, IgnoresAlternatingQueueLatencySpikes)
{
  TimestampOffsetEstimator estimator(100'000'000, 0.1, 50, 5);
  for (int index = 0; index < 100; ++index) {
    const auto normal = estimator.update(5'000'000 + index * 1000);
    EXPECT_TRUE(normal.initialized);
    const auto delayed = estimator.update(350'000'000 + index * 1000);
    EXPECT_FALSE(delayed.confirmed_jump);
    EXPECT_TRUE(delayed.latency_outlier);
  }
  EXPECT_NEAR(estimator.offset_ns(), 5'000'000, 500'000);
}

TEST(TimestampOffsetEstimator, ConfirmsSustainedClockDomainChange)
{
  TimestampOffsetEstimator estimator(100'000'000, 0.1, 4, 3);
  for (int index = 0; index < 8; ++index) {
    estimator.update(5'000'000);
  }
  bool jumped = false;
  for (int index = 0; index < 10; ++index) {
    jumped = jumped || estimator.update(400'000'000).confirmed_jump;
  }
  EXPECT_TRUE(jumped);
  EXPECT_NEAR(estimator.offset_ns(), 400'000'000, 1);
}
