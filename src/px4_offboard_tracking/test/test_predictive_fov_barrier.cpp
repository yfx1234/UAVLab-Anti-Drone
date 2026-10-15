#include <gtest/gtest.h>

#include "px4_offboard_tracking/predictive_fov_barrier.hpp"

using px4_offboard_tracking::FovBarrierZone;
using px4_offboard_tracking::PredictiveFovBarrier;

TEST(PredictiveFovBarrier, ClassifiesFourZones)
{
  PredictiveFovBarrier barrier;
  EXPECT_EQ(barrier.update({0.0, true, true, 0.02, 0.03, 1}).zone,
    FovBarrierZone::NORMAL);
  barrier.reset();
  EXPECT_EQ(barrier.update({1.0, true, true, 0.11, 0.02, 2}).zone,
    FovBarrierZone::CENTERING);
  barrier.reset();
  EXPECT_EQ(barrier.update({2.0, true, true, 0.15, 0.02, 3}).zone,
    FovBarrierZone::RECOVERY);
  barrier.reset();
  EXPECT_EQ(barrier.update({3.0, false, true, 0.0, 0.0, 0}).zone,
    FovBarrierZone::IMM_COAST);
}

TEST(PredictiveFovBarrier, PredictsComponentBoundaryCrossing)
{
  PredictiveFovBarrier barrier;
  barrier.update({0.0, true, true, 0.05, 0.01, 1});
  const auto result = barrier.update({0.1, true, true, 0.08, 0.01, 2});
  EXPECT_EQ(result.zone, FovBarrierZone::RECOVERY);
  EXPECT_GT(result.predicted_horizontal_error_rad, 0.139);
  EXPECT_LT(std::abs(result.predicted_vertical_error_rad), 0.02);
}

TEST(PredictiveFovBarrier, RecoveryExitRequiresSixDegrees)
{
  PredictiveFovBarrier barrier;
  barrier.update({0.0, true, true, 0.15, 0.0, 1});
  EXPECT_EQ(barrier.update({0.5, true, true, 0.12, 0.0, 2}).zone,
    FovBarrierZone::RECOVERY);
  const auto recovered = barrier.update({1.5, true, true, 0.05, 0.0, 3});
  EXPECT_EQ(recovered.zone, FovBarrierZone::NORMAL);
  EXPECT_TRUE(recovered.recovery_exit_allowed);
}

TEST(PredictiveFovBarrier, LostWithoutUsableImm)
{
  PredictiveFovBarrier barrier;
  EXPECT_EQ(barrier.update({0.0, false, false, 0.0, 0.0, 0}).zone,
    FovBarrierZone::LOST);
}
