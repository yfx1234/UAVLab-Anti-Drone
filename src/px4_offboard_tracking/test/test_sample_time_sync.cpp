#include <gtest/gtest.h>

#include <cmath>

#include "px4_offboard_tracking/sample_time_sync.hpp"

using px4_offboard_tracking::Quaterniond;
using px4_offboard_tracking::SampleTimeAttitudeBuffer;

TEST(SampleTimeAttitudeBuffer, InterpolatesAtCameraAcquisitionTime)
{
  SampleTimeAttitudeBuffer buffer(2.0, 0.1);
  const double half_yaw = M_PI / 4.0;
  ASSERT_TRUE(buffer.add(10.0, Quaterniond{}));
  ASSERT_TRUE(buffer.add(
      10.1, Quaterniond{std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw)}));
  Quaterniond output;
  ASSERT_TRUE(buffer.sample(10.05, output));
  EXPECT_NEAR(output.w, std::cos(M_PI / 8.0), 1e-9);
  EXPECT_NEAR(output.z, std::sin(M_PI / 8.0), 1e-9);
}

TEST(SampleTimeAttitudeBuffer, RejectsUnsynchronizedOrOutOfOrderSamples)
{
  SampleTimeAttitudeBuffer buffer(2.0, 0.05);
  ASSERT_TRUE(buffer.add(5.0, Quaterniond{}));
  EXPECT_FALSE(buffer.add(4.9, Quaterniond{}));
  Quaterniond output;
  EXPECT_FALSE(buffer.sample(5.2, output));
}

TEST(SampleTimeAttitudeBuffer, SelectsPx4SampleTimeOnlyInRosClockDomain)
{
  EXPECT_DOUBLE_EQ(
    SampleTimeAttitudeBuffer::px4_sample_or_receipt_time(
      10001000ULL, 10.002, 0.01),
    10.001);
  EXPECT_DOUBLE_EQ(
    SampleTimeAttitudeBuffer::px4_sample_or_receipt_time(
      1000ULL, 1000.0, 0.1),
    1000.0);
}

TEST(SampleTimeAttitudeBuffer, PreservesSampleDelayForRawPx4HrtClock)
{
  const double receipt_time = 1720000000.0;
  const double mapped = SampleTimeAttitudeBuffer::px4_sample_or_receipt_time(
    10000000ULL, 9980000ULL, receipt_time, 5.0);
  EXPECT_NEAR(mapped, receipt_time - 0.02, 1e-9);
}
