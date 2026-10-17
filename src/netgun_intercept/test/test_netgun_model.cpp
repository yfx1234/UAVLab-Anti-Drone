#include <gtest/gtest.h>

#include "netgun_intercept/netgun_model.hpp"

#include <cmath>

namespace
{

using netgun_intercept::CoverageInput;
using netgun_intercept::NetgunModel;
using netgun_intercept::NetgunModelConfig;

constexpr double kDeg = M_PI / 180.0;

TEST(NetgunModel, TimeOfFlightIsLinearWithoutDynamics)
{
  NetgunModel model;
  EXPECT_NEAR(model.time_of_flight_s(8.0), 8.0 / 55.0, 1e-12);
  EXPECT_NEAR(model.time_of_flight_s(0.0), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(model.trigger_lead_s(8.0), 8.0 / 55.0 + 0.15);
}

TEST(NetgunModel, CoverageSquareBoundary)
{
  NetgunModel model;
  EXPECT_TRUE(model.covered({0.0, 0.0, 8.0}));
  // 正方形边界：角点 (1.5, 1.5) 在界内（闭区间）。
  EXPECT_TRUE(model.covered({1.5, 1.5, 8.0}));
  EXPECT_TRUE(model.covered({-1.5, 1.5, 6.0}));
  EXPECT_FALSE(model.covered({1.51, 0.0, 8.0}));
  EXPECT_FALSE(model.covered({0.0, -1.51, 8.0}));
}

TEST(NetgunModel, CoverageRangeWindow)
{
  NetgunModel model;
  EXPECT_TRUE(model.covered({0.0, 0.0, 6.0}));
  EXPECT_TRUE(model.covered({0.0, 0.0, 10.0}));
  EXPECT_FALSE(model.covered({0.0, 0.0, 5.99}));
  EXPECT_FALSE(model.covered({0.0, 0.0, 10.01}));
  EXPECT_FALSE(model.covered({0.0, 0.0, std::nan("")}));
}

TEST(NetgunModel, EffectiveRangeHelper)
{
  NetgunModel model;
  EXPECT_TRUE(model.range_within_effective(8.0));
  EXPECT_FALSE(model.range_within_effective(12.0));
}

TEST(NetgunModel, ConfigOverrideMatchesPlanV5C)
{
  NetgunModelConfig config;
  EXPECT_DOUBLE_EQ(config.muzzle_speed_mps, 55.0);
  EXPECT_DOUBLE_EQ(config.coverage_half_extent_m, 1.5);
  EXPECT_DOUBLE_EQ(config.effective_range_min_m, 6.0);
  EXPECT_DOUBLE_EQ(config.effective_range_max_m, 10.0);
  EXPECT_EQ(config.max_shots, 1);
  // 8m 处覆盖半角 ≈ atan(1.5/8)（蒙射误差门的推导依据）。
  EXPECT_NEAR(std::atan(1.5 / 8.0) / kDeg, 10.6, 0.05);
}

}  // namespace
