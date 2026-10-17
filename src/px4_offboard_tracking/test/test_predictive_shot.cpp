#include <gtest/gtest.h>

#include <cmath>

#include "px4_offboard_tracking/predictive_shot.hpp"

using px4_offboard_tracking::PredictiveShotConfig;
using px4_offboard_tracking::PredictiveShotInput;
using px4_offboard_tracking::PredictiveShotSolver;

namespace
{

PredictiveShotInput nominal_input()
{
  PredictiveShotInput input;
  input.relative_position_ned = {0.0, 0.0, -25.0};
  for (std::size_t axis = 0; axis < 9; ++axis) {
    input.state_covariance[9 * axis + axis] = axis < 3 ? 0.01 : 0.04;
  }
  return input;
}

}  // namespace

TEST(PredictiveShot, StationaryTargetProducesBallisticCompensation)
{
  PredictiveShotSolver solver(PredictiveShotConfig{});
  const auto result = solver.solve(nominal_input());
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.time_of_flight_s, 25.0 / 400.0, 1e-4);
  EXPECT_LT(result.shot_direction_ned[2], -0.999);
  EXPECT_GT(result.shot_direction_ned[2], -1.001);
}

TEST(PredictiveShot, LateralMotionProducesExpectedLeadAngle)
{
  PredictiveShotSolver solver(PredictiveShotConfig{});
  auto input = nominal_input();
  input.relative_velocity_ned = {0.0, 2.0, 0.0};
  const auto result = solver.solve(input);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.lead_angle_rad, std::atan(2.0 / 400.0), 2e-4);
  EXPECT_GT(result.shot_direction_ned[1], 0.0);
}

TEST(PredictiveShot, RejectsUnreachableRelativeSpeed)
{
  PredictiveShotSolver solver(PredictiveShotConfig{});
  auto input = nominal_input();
  input.relative_velocity_ned = {401.0, 0.0, 0.0};
  EXPECT_FALSE(solver.solve(input).valid);
}

TEST(PredictiveShot, PropagatesVelocityUncertaintyOverFlightTime)
{
  PredictiveShotSolver solver(PredictiveShotConfig{});
  auto first = nominal_input();
  auto second = first;
  second.state_covariance[9 * 3 + 3] = 4.0;
  second.state_covariance[9 * 4 + 4] = 4.0;
  const auto low_uncertainty = solver.solve(first);
  const auto high_uncertainty = solver.solve(second);
  ASSERT_TRUE(low_uncertainty.valid);
  ASSERT_TRUE(high_uncertainty.valid);
  EXPECT_GT(
    high_uncertainty.predicted_lateral_sigma_m,
    low_uncertainty.predicted_lateral_sigma_m);
}
