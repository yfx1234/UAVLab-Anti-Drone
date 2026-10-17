#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "px4_offboard_tracking/target_trajectory.hpp"

using px4_offboard_tracking::TargetScenario;
using px4_offboard_tracking::TargetTrajectory;
using px4_offboard_tracking::target_scenario_from_string;

TEST(TargetTrajectory, ParsesSupportedScenarios)
{
  EXPECT_EQ(target_scenario_from_string("stationary"), TargetScenario::Stationary);
  EXPECT_EQ(target_scenario_from_string("straight"), TargetScenario::Straight);
  EXPECT_EQ(target_scenario_from_string("circle"), TargetScenario::Circle);
  EXPECT_EQ(target_scenario_from_string("s_curve"), TargetScenario::SCurve);
  EXPECT_EQ(target_scenario_from_string("figure_eight"), TargetScenario::FigureEight);
  EXPECT_EQ(target_scenario_from_string("rectangle"), TargetScenario::Rectangle);
  EXPECT_THROW(target_scenario_from_string("unknown"), std::invalid_argument);
}

TEST(TargetTrajectory, SCurveProgressesWithBoundedLateralMotion)
{
  TargetTrajectory trajectory(TargetScenario::SCurve, 12.0, 4.0, 5.0, 0.0, 40.0, 12.0);
  const auto start = trajectory.sample(0.0, 0.0, 0.0, 0.0);
  const auto next = trajectory.sample(0.0, 0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(start.x, 0.0);
  EXPECT_NEAR(start.y, 0.0, 1e-12);
  EXPECT_GT(start.vx, 0.0);
  EXPECT_GT(next.y, start.y);
  EXPECT_GT(next.x, start.x);
  EXPECT_LE(std::hypot(next.vx, next.vy), 5.0 + 1e-12);
}

TEST(TargetTrajectory, FigureEightIsBoundedAndStartsContinuously)
{
  TargetTrajectory trajectory(
    TargetScenario::FigureEight, 12.0, 4.0, 5.0, 0.0, 40.0, 20.0);
  const auto start = trajectory.sample(0.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(start.x, 0.0);
  EXPECT_NEAR(start.y, 0.0, 1e-12);
  EXPECT_NEAR(start.yaw, M_PI_2, 1e-12);
  double maximum_speed = 0.0;
  for (int sample_index = 0; sample_index < 10000; ++sample_index) {
    const auto point = trajectory.sample(
      0.0, 0.0, 0.0, 0.01 * static_cast<double>(sample_index));
    maximum_speed = std::max(maximum_speed, std::hypot(point.vx, point.vy));
  }
  EXPECT_LE(maximum_speed, 5.0 + 1e-9);
  EXPECT_GT(maximum_speed, 4.99);
}

TEST(TargetTrajectory, RectangleUsesRoundedCornersAndTangentialYaw)
{
  TargetTrajectory trajectory(TargetScenario::Rectangle, 12.0, 4.0, 1.0, 0.0, 8.0, 3.0);
  const auto start = trajectory.sample(0.0, 0.0, 0.0, 0.0);
  const auto before_turn = trajectory.sample(0.0, 0.0, 0.0, 6.49);
  const auto after_turn = trajectory.sample(0.0, 0.0, 0.0, 6.51);
  EXPECT_DOUBLE_EQ(start.x, 0.0);
  EXPECT_DOUBLE_EQ(start.y, 0.0);
  EXPECT_NEAR(start.yaw, M_PI, 1e-12);
  EXPECT_LT(std::abs(after_turn.yaw - before_turn.yaw), 0.05);
}

TEST(TargetTrajectory, EveryScenarioStartsAtTheSuppliedThreeDimensionalAnchor)
{
  for (const auto scenario : {
      TargetScenario::Stationary,
      TargetScenario::Straight,
      TargetScenario::Circle,
      TargetScenario::SCurve,
      TargetScenario::FigureEight,
      TargetScenario::Rectangle})
  {
    TargetTrajectory trajectory(
      scenario, 25.0, 20.0, 2.0, M_PI, 40.0, 12.0);
    const auto start = trajectory.sample_with_speed_ramp(
      7.0, -4.0, -3.0, 0.0, 3.0);
    EXPECT_NEAR(start.x, 7.0, 1e-12);
    EXPECT_NEAR(start.y, -4.0, 1e-12);
    EXPECT_NEAR(start.z, -28.0, 1e-12);
    EXPECT_NEAR(std::hypot(start.vx, start.vy), 0.0, 1e-12);
  }
}

TEST(TargetTrajectory, StationaryHoldsHorizontalPositionAtConfiguredHeight)
{
  TargetTrajectory trajectory(TargetScenario::Stationary, 12.0, 4.0, 1.0, 0.3);
  const auto point = trajectory.sample(2.0, -3.0, 0.5, 20.0);
  EXPECT_DOUBLE_EQ(point.x, 2.0);
  EXPECT_DOUBLE_EQ(point.y, -3.0);
  EXPECT_DOUBLE_EQ(point.z, -11.5);
  EXPECT_DOUBLE_EQ(point.yaw, 0.3);
}

TEST(TargetTrajectory, StraightUsesSpeedAndHeading)
{
  TargetTrajectory trajectory(TargetScenario::Straight, 12.0, 4.0, 20.0, M_PI_2);
  const auto point = trajectory.sample(1.0, 2.0, 0.0, 3.0);
  EXPECT_NEAR(point.x, 1.0, 1e-12);
  EXPECT_NEAR(point.y, 62.0, 1e-12);
  EXPECT_DOUBLE_EQ(point.z, -12.0);
  EXPECT_NEAR(point.vx, 0.0, 1e-12);
  EXPECT_NEAR(point.vy, 20.0, 1e-12);
  EXPECT_DOUBLE_EQ(point.ax, 0.0);
  EXPECT_DOUBLE_EQ(point.ay, 0.0);
  EXPECT_DOUBLE_EQ(point.yaw, M_PI_2);
}

TEST(TargetTrajectory, CircleUsesConfiguredRadiusAndTangentialYaw)
{
  TargetTrajectory trajectory(TargetScenario::Circle, 12.0, 4.0, 1.0, 0.0);
  const auto start = trajectory.sample(0.0, 0.0, 0.0, 0.0);
  const double quarter_period = M_PI * 4.0 / (2.0 * 1.0);
  const auto point = trajectory.sample(0.0, 0.0, 0.0, quarter_period);
  EXPECT_DOUBLE_EQ(start.x, 0.0);
  EXPECT_DOUBLE_EQ(start.y, 0.0);
  EXPECT_NEAR(point.x, -4.0, 1e-12);
  EXPECT_NEAR(point.y, 4.0, 1e-12);
  EXPECT_DOUBLE_EQ(point.z, -12.0);
  EXPECT_NEAR(point.vx, -1.0, 1e-12);
  EXPECT_NEAR(point.vy, 0.0, 1e-12);
  EXPECT_NEAR(point.ax, 0.0, 1e-12);
  EXPECT_NEAR(point.ay, -0.25, 1e-12);
  EXPECT_NEAR(point.yaw, M_PI, 1e-12);
}

TEST(TargetTrajectory, HighSpeedCircleReportsConsistentCentripetalAcceleration)
{
  TargetTrajectory trajectory(TargetScenario::Circle, 35.0, 100.0, 20.0, 0.0);
  const auto point = trajectory.sample(0.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(point.x, 0.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
  EXPECT_DOUBLE_EQ(point.vx, 0.0);
  EXPECT_DOUBLE_EQ(point.vy, 20.0);
  EXPECT_NEAR(point.ax, -4.0, 1e-12);
  EXPECT_DOUBLE_EQ(point.ay, 0.0);
}

TEST(TargetTrajectory, CircleRotatesWithConfiguredHeading)
{
  TargetTrajectory trajectory(TargetScenario::Circle, 25.0, 4.0, 2.0, M_PI);
  const double quarter_period = M_PI * 4.0 / (2.0 * 2.0);
  const auto start = trajectory.sample(10.0, 5.0, 0.0, 0.0);
  const auto point = trajectory.sample(10.0, 5.0, 0.0, quarter_period);
  EXPECT_NEAR(start.x, 10.0, 1e-12);
  EXPECT_NEAR(start.y, 5.0, 1e-12);
  EXPECT_NEAR(point.x, 14.0, 1e-12);
  EXPECT_NEAR(point.y, 1.0, 1e-12);
  EXPECT_NEAR(point.vx, 2.0, 1e-12);
  EXPECT_NEAR(point.vy, 0.0, 1e-12);
}

TEST(TargetTrajectory, HighSpeedFigureEightStartsContinuouslyAndRespectsSpeedLimit)
{
  TargetTrajectory trajectory(
    TargetScenario::FigureEight, 35.0, 4.0, 20.0, 0.0, 220.0, 120.0);
  const auto start = trajectory.sample(3.0, -2.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(start.x, 3.0);
  EXPECT_DOUBLE_EQ(start.y, -2.0);
  double maximum_speed = 0.0;
  for (int sample_index = 0; sample_index < 10000; ++sample_index) {
    const auto point = trajectory.sample(
      3.0, -2.0, 0.0, 0.01 * static_cast<double>(sample_index));
    maximum_speed = std::max(maximum_speed, std::hypot(point.vx, point.vy));
  }
  EXPECT_LE(maximum_speed, 20.0 + 1e-9);
  EXPECT_GT(maximum_speed, 19.99);
}

TEST(TargetTrajectory, SpeedRampStartsFromZeroVelocityAndPreservesAccelerationBound)
{
  TargetTrajectory trajectory(TargetScenario::Circle, 35.0, 16.0, 4.0, 0.0);
  const auto start = trajectory.sample_with_speed_ramp(0.0, 0.0, 0.0, 0.0, 3.0);
  const auto middle = trajectory.sample_with_speed_ramp(0.0, 0.0, 0.0, 1.5, 3.0);
  const auto complete = trajectory.sample_with_speed_ramp(0.0, 0.0, 0.0, 3.0, 3.0);
  EXPECT_DOUBLE_EQ(start.x, 0.0);
  EXPECT_DOUBLE_EQ(start.y, 0.0);
  EXPECT_DOUBLE_EQ(std::hypot(start.vx, start.vy), 0.0);
  EXPECT_NEAR(std::hypot(middle.vx, middle.vy), 2.0, 1e-12);
  EXPECT_NEAR(std::hypot(complete.vx, complete.vy), 4.0, 1e-12);
  EXPECT_LE(std::hypot(start.ax, start.ay), 4.0 / 3.0 + 1e-12);
}

TEST(TargetTrajectory, ZeroRampStartsAtAnchorWithFullConfiguredVelocity)
{
  TargetTrajectory trajectory(
    TargetScenario::Straight, 25.0, 20.0, 5.0, M_PI_2, 40.0, 12.0);
  const auto start = trajectory.sample_with_speed_ramp(
    7.0, -4.0, -3.0, 0.0, 0.0);
  EXPECT_NEAR(start.x, 7.0, 1e-12);
  EXPECT_NEAR(start.y, -4.0, 1e-12);
  EXPECT_NEAR(start.z, -28.0, 1e-12);
  EXPECT_NEAR(start.vx, 0.0, 1e-12);
  EXPECT_NEAR(start.vy, 5.0, 1e-12);
}

TEST(TargetTrajectory, RoundedRectangleAddsAccelerationOnlyInCorners)
{
  TargetTrajectory trajectory(TargetScenario::Rectangle, 35.0, 4.0, 20.0, 0.0, 200.0, 120.0);
  const auto straight = trajectory.sample(0.0, 0.0, 0.0, 1.0);
  const auto corner = trajectory.sample(0.0, 0.0, 0.0, 151.0 / 20.0);
  EXPECT_NEAR(std::hypot(straight.vx, straight.vy), 20.0, 1e-12);
  EXPECT_NEAR(std::hypot(straight.ax, straight.ay), 0.0, 1e-12);
  EXPECT_NEAR(std::hypot(corner.vx, corner.vy), 20.0, 1e-12);
  EXPECT_NEAR(std::hypot(corner.ax, corner.ay), 400.0 / 30.0, 1e-9);
}
