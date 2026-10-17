#include <gtest/gtest.h>

#include "px4_offboard_tracking/target_observation.hpp"

using px4_offboard_tracking::CameraIntrinsics;
using px4_offboard_tracking::ObservationFaultConfig;
using px4_offboard_tracking::PinholeProjection;
using px4_offboard_tracking::TargetObservationModel;

namespace
{
const CameraIntrinsics kIntrinsics{320.0, 320.0, 319.5, 239.5, 640, 480, 0.05};
const PinholeProjection kIdeal{{0.0, 0.0, 7.0}, 319.5, 239.5, 7.0, 7.0, true};
}  // namespace

TEST(TargetObservationModel, DisabledFaultsPreserveIdealObservation)
{
  TargetObservationModel model(kIntrinsics, ObservationFaultConfig{});
  const auto sample = model.apply(kIdeal);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->pixel_u, kIdeal.pixel_u);
  EXPECT_DOUBLE_EQ(sample->pixel_v, kIdeal.pixel_v);
  EXPECT_DOUBLE_EQ(sample->depth, kIdeal.depth);
  EXPECT_TRUE(sample->visible);
  EXPECT_FALSE(sample->outlier_injected);
}

TEST(TargetObservationModel, FullDropoutReturnsNoFrame)
{
  ObservationFaultConfig config;
  config.dropout_probability = 1.0;
  TargetObservationModel model(kIntrinsics, config);
  EXPECT_FALSE(model.apply(kIdeal).has_value());
}

TEST(TargetObservationModel, FullOutlierProducesBoundedFalseObservation)
{
  ObservationFaultConfig config;
  config.outlier_probability = 1.0;
  config.outlier_min_depth_m = 2.0;
  config.outlier_max_depth_m = 3.0;
  config.random_seed = 42;
  TargetObservationModel model(kIntrinsics, config);
  const auto sample = model.apply(kIdeal);
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->outlier_injected);
  EXPECT_TRUE(sample->visible);
  EXPECT_GE(sample->pixel_u, 0.0);
  EXPECT_LT(sample->pixel_u, kIntrinsics.width);
  EXPECT_GE(sample->pixel_v, 0.0);
  EXPECT_LT(sample->pixel_v, kIntrinsics.height);
  EXPECT_GE(sample->depth, 2.0);
  EXPECT_LE(sample->depth, 3.0);
}

TEST(TargetObservationModel, SeededNoiseIsRepeatableAndChangesMeasurement)
{
  ObservationFaultConfig config;
  config.pixel_noise_stddev_px = 2.0;
  config.depth_noise_stddev_m = 0.1;
  config.random_seed = 7;
  TargetObservationModel first(kIntrinsics, config);
  TargetObservationModel second(kIntrinsics, config);
  const auto first_sample = first.apply(kIdeal);
  const auto second_sample = second.apply(kIdeal);
  ASSERT_TRUE(first_sample.has_value());
  ASSERT_TRUE(second_sample.has_value());
  EXPECT_DOUBLE_EQ(first_sample->pixel_u, second_sample->pixel_u);
  EXPECT_DOUBLE_EQ(first_sample->pixel_v, second_sample->pixel_v);
  EXPECT_DOUBLE_EQ(first_sample->depth, second_sample->depth);
  EXPECT_NE(first_sample->pixel_u, kIdeal.pixel_u);
  EXPECT_NE(first_sample->depth, kIdeal.depth);
}
