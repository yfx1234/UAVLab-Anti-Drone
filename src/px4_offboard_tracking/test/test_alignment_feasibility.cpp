#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "px4_offboard_tracking/alignment_feasibility.hpp"

using px4_offboard_tracking::AlignmentFeasibility;
using px4_offboard_tracking::AlignmentFeasibilityConfig;
using px4_offboard_tracking::AlignmentFeasibilityInput;

TEST(AlignmentFeasibility, AcceptsCommandWithMargins)
{
  AlignmentFeasibility checker(AlignmentFeasibilityConfig{});
  const auto result = checker.evaluate({0.2, 0.75, 0.9, 0.5, 0.2, 6.0, 2.0});
  EXPECT_TRUE(result.feasible);
  EXPECT_NEAR(result.thrust_margin, 0.15, 1e-9);
}

TEST(AlignmentFeasibility, ReportsEveryIndependentLimit)
{
  AlignmentFeasibility checker(AlignmentFeasibilityConfig{});
  const auto result = checker.evaluate({0.5, 0.85, 0.9, 2.5, 1.5, 1.0, 9.0});
  EXPECT_FALSE(result.feasible);
  EXPECT_FALSE(result.tilt_ok);
  EXPECT_FALSE(result.thrust_margin_ok);
  EXPECT_FALSE(result.body_rate_ok);
  EXPECT_FALSE(result.altitude_loss_ok);
  EXPECT_FALSE(result.minimum_range_ok);
  EXPECT_FALSE(result.alignment_time_ok);
}

TEST(AlignmentFeasibility, RejectsNonFiniteInput)
{
  AlignmentFeasibility checker(AlignmentFeasibilityConfig{});
  AlignmentFeasibilityInput input{0.2, 0.7, 0.9, 0.2, 0.1, 7.0, 1.0};
  input.range_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(checker.evaluate(input).feasible);
}

TEST(AlignmentFeasibility, RejectsInvalidConfiguration)
{
  auto config = AlignmentFeasibilityConfig{};
  config.minimum_range_m = 0.0;
  EXPECT_THROW({AlignmentFeasibility checker(config);}, std::invalid_argument);
}
