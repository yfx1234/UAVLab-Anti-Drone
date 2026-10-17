#include "falcon_intercept_control/real_flight_core.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace falcon_intercept_control;

TEST(Coordinates, StandardOpticalToFlu)
{
  const auto v = optical_to_body_flu({1.0, 2.0, 3.0}, CameraToFluTransform{});
  EXPECT_NEAR(v[0], 3.0, 1e-9); EXPECT_NEAR(v[1], -1.0, 1e-9); EXPECT_NEAR(v[2], -2.0, 1e-9);
  const auto frd = flu_to_frd(v);
  EXPECT_EQ(frd, (Vector3{3.0, 1.0, 2.0}));
}

TEST(Search, UsesActualYawAcrossPi)
{
  SearchController c(SearchConfig{0.2, 0.5, 120.0, 0.004, 0.2});
  auto a = c.update(SearchInput{0.0, 3.10, true, true, false, 0.0});
  auto b = c.update(SearchInput{0.1, -3.10, true, true, false, 0.0});
  EXPECT_TRUE(a.valid); EXPECT_TRUE(b.valid); EXPECT_LT(b.progress, 0.1);
}

TEST(Search, TimeoutAndCue)
{
  SearchController c(SearchConfig{0.2, 0.5, 1.0, 0.004, 0.2});
  c.update(SearchInput{0.0, 0.0, true, true, false, 0.0});
  auto cue = c.update(SearchInput{0.1, 0.0, true, true, true, 100.0});
  EXPECT_TRUE(cue.target_cued); EXPECT_NEAR(cue.yaw_rate_flu_rps, -0.2, 1e-9);
  EXPECT_TRUE(c.update(SearchInput{1.1, 0.0, true, true, false, 0.0}).timed_out);
}

TEST(Guidance, TrackHasZeroTranslationAndApproachIsLimited)
{
  GuidanceController c(GuidanceConfig{});
  auto track = c.update({5.0, 1.0, 0.5}, {0.0, 0.0, 0.0}, 0.1, false);
  EXPECT_EQ(track.velocity_body_flu_mps, (Vector3{})); EXPECT_LE(std::abs(track.yaw_rate_flu_rps), 0.2);
  auto approach = c.update({5.0, 1.0, 0.5}, {0.0, 0.0, 0.0}, 0.1, true);
  EXPECT_LE(std::abs(approach.velocity_body_flu_mps[0]), 0.03 + 1e-9);
  EXPECT_LE(std::abs(approach.velocity_body_flu_mps[1]), 0.03 + 1e-9);
}

TEST(Safety, PredictionDecay)
{
  EXPECT_DOUBLE_EQ(prediction_decay(0.1), 1.0); EXPECT_NEAR(prediction_decay(0.35), 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(prediction_decay(0.5), 0.0);
}

TEST(Safety, StationaryInWindowDoesNotInventPredictedMiss)
{
  const auto k = relative_kinematics({0.0, 0.0, 1.7}, {0.0, 0.0, 0.0});
  EXPECT_TRUE(k.valid); EXPECT_DOUBLE_EQ(k.predicted_miss_distance_m, 0.0);
}

TEST(Capture, RequiresDistinctFramesAndHold)
{
  CaptureConfig cfg; cfg.minimum_distinct_frames = 2; cfg.required_hold_time_s = 0.3;
  CaptureWindow w(cfg); RelativeKinematicsResult k; k.valid = true; k.range_m = 1.7;
  k.los_rate_norm_rad_s = 0.1; k.relative_speed_mps = 0.2; k.predicted_miss_distance_m = 0.1;
  k.time_to_closest_approach_s = 1.0;
  EXPECT_FALSE(w.update({0.0, 10.0, 1, true, true, 0.01, k}).held);
  EXPECT_FALSE(w.update({0.4, 10.0, 1, true, true, 0.01, k}).held);
  EXPECT_TRUE(w.update({0.4, 10.1, 2, true, true, 0.01, k}).held);
  EXPECT_FALSE(w.update({0.2, 10.2, 3, true, true, 0.01, k}).held);
}

TEST(RelativeKinematics, MirrorsUpstreamProductionCases)
{
  auto head_on = relative_kinematics({10.0, 0.0, 0.0}, {-2.0, 0.0, 0.0});
  EXPECT_TRUE(head_on.valid); EXPECT_DOUBLE_EQ(head_on.closing_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(head_on.time_to_closest_approach_s, 5.0);
  EXPECT_DOUBLE_EQ(head_on.predicted_miss_distance_m, 0.0);
  auto tangent = relative_kinematics({10.0, 0.0, 0.0}, {0.0, 2.0, 0.0});
  EXPECT_DOUBLE_EQ(tangent.los_rate_rad_s[2], 0.2);
  EXPECT_DOUBLE_EQ(tangent.predicted_miss_distance_m, 10.0);
  auto receding = relative_kinematics({10.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(receding.closing_speed_mps, -2.0);
  EXPECT_DOUBLE_EQ(receding.time_to_closest_approach_s, 0.0);
  EXPECT_FALSE(relative_kinematics({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}).valid);
}

TEST(QualityGate, ReportsIndependentProductionFailures)
{
  QualityConfig config;
  auto good = evaluate_quality(config, {true, true, 0.1, 1.0, 2.0, true});
  EXPECT_TRUE(good.passed);
  auto bad = evaluate_quality(config, {true, true, 0.3, 5.0, 20.0, false});
  EXPECT_FALSE(bad.passed); EXPECT_FALSE(bad.target_fresh); EXPECT_FALSE(bad.covariance_ok);
  EXPECT_FALSE(bad.innovation_ok); EXPECT_FALSE(bad.px4_state_ok);
  EXPECT_FALSE(evaluate_quality(config, {true, true, NAN, 1.0, 2.0, true}).passed);
}

TEST(Capture, RepeatedAndRolledBackObservationsCannotAccumulate)
{
  CaptureConfig cfg; cfg.minimum_distinct_frames = 3; cfg.required_hold_time_s = 0.2;
  CaptureWindow window(cfg); RelativeKinematicsResult k; k.valid = true; k.range_m = 1.7;
  k.los_rate_norm_rad_s = 0.05; k.relative_speed_mps = 0.2; k.predicted_miss_distance_m = 0.1;
  auto first = window.update({1.0, 10.0, 10, true, true, 0.01, k});
  EXPECT_EQ(first.distinct_frames, 1U);
  EXPECT_EQ(window.update({1.3, 10.0, 10, true, true, 0.01, k}).distinct_frames, 1U);
  EXPECT_EQ(window.update({1.4, 10.1, 11, true, true, 0.01, k}).distinct_frames, 2U);
  auto rollback = window.update({1.5, 9.0, 12, true, true, 0.01, k});
  EXPECT_FALSE(rollback.instantaneous_pass); EXPECT_EQ(rollback.distinct_frames, 0U);
  EXPECT_EQ(window.update({1.6, 9.1, 13, true, true, 0.01, k}).distinct_frames, 1U);
}

TEST(Capture, FailedConditionAndClockRollbackResetHold)
{
  CaptureConfig cfg; cfg.minimum_distinct_frames = 2; cfg.required_hold_time_s = 0.2;
  CaptureWindow window(cfg); RelativeKinematicsResult k; k.valid = true; k.range_m = 1.7;
  k.los_rate_norm_rad_s = 0.05; k.relative_speed_mps = 0.2; k.predicted_miss_distance_m = 0.1;
  window.update({2.0, 2.0, 1, true, true, 0.01, k});
  auto unsafe = k; unsafe.range_m = 1.0;
  EXPECT_FALSE(window.update({2.3, 2.3, 2, true, true, 0.01, unsafe}).instantaneous_pass);
  EXPECT_EQ(window.update({2.4, 2.4, 3, true, true, 0.01, k}).distinct_frames, 1U);
  EXPECT_FALSE(window.update({1.0, 2.5, 4, true, true, 0.01, k}).held);
  EXPECT_EQ(window.update({2.6, 2.6, 5, true, true, 0.01, k}).distinct_frames, 1U);
}

TEST(Capture, RejectsInvalidUpstreamConfigurations)
{
  CaptureConfig config; config.maximum_range_m = config.minimum_range_m;
  EXPECT_THROW(CaptureWindow invalid(config), std::invalid_argument);
  config = CaptureConfig{}; config.minimum_distinct_frames = 0;
  EXPECT_THROW(CaptureWindow invalid(config), std::invalid_argument);
}
