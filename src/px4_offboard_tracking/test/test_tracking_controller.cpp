#include <gtest/gtest.h>

#include "px4_offboard_tracking/acceleration_guidance.hpp"
#include "px4_offboard_tracking/geometric_control.hpp"
#include "px4_offboard_tracking/tracking_controller.hpp"

using px4_offboard_tracking::State3d;
using px4_offboard_tracking::TrackingController;

TEST(TrackingController, ClosesRangeAndFacesTarget)
{
  TrackingController controller(0.5, 0.0, 4.0, 2.0, 2.0);
  const auto command = controller.update(State3d{}, State3d{10.0, 0.0, -5.0});
  EXPECT_DOUBLE_EQ(command.vx, 4.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.vz, -2.0);
  EXPECT_DOUBLE_EQ(command.yaw, 0.0);
}

TEST(TrackingController, StopsInsideStandoffButMatchesTargetVelocity)
{
  TrackingController controller(1.0, 1.0, 5.0, 2.0, 3.0);
  State3d target{2.0, 0.0, 0.0, 1.0, -0.5, 0.25};
  const auto command = controller.update(State3d{}, target);
  EXPECT_DOUBLE_EQ(command.vx, 1.0);
  EXPECT_DOUBLE_EQ(command.vy, -0.5);
  EXPECT_DOUBLE_EQ(command.vz, 0.25);
}

TEST(TrackingController, LimitsHorizontalNorm)
{
  TrackingController controller(10.0, 0.0, 3.0, 1.0, 0.0);
  const auto command = controller.update(State3d{}, State3d{3.0, 4.0, 0.0});
  EXPECT_NEAR(std::hypot(command.vx, command.vy), 3.0, 1e-12);
}

TEST(AccelerationGuidance, ProducesBoundedAcceleration)
{
  px4_offboard_tracking::AccelerationGuidance guidance(1.0, 1.0, 3.0, 2.0, 2.0);
  State3d pursuer{};
  State3d target{10.0, 0.0, -5.0, 1.0, 0.0, 0.0};
  const auto command = guidance.update(pursuer, target);
  EXPECT_NEAR(std::hypot(command.ax, command.ay), 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(command.az, -2.0);
}

TEST(GeometricControl, HoverIsLevelWithNegativePx4ThrustConvention)
{
  px4_offboard_tracking::GeometricControl control(0.5, 0.8, 0.6, 3.0, 3.0);
  const auto command = control.attitude_from_acceleration({});
  EXPECT_NEAR(command.roll, 0.0, 1e-12);
  EXPECT_NEAR(command.pitch, 0.0, 1e-12);
  EXPECT_NEAR(command.thrust, 0.5, 1e-12);
}

TEST(GeometricControl, ForwardAccelerationCommandsNoseDown)
{
  px4_offboard_tracking::GeometricControl control(0.5, 0.8, 0.6, 3.0, 3.0);
  px4_offboard_tracking::AccelerationCommand acceleration;
  acceleration.ax = 2.0;
  const auto command = control.attitude_from_acceleration(acceleration);
  EXPECT_LT(command.pitch, 0.0);
  EXPECT_GT(command.thrust, 0.5);
}

TEST(GeometricControl, TransientCollectiveUsesCurrentRatherThanDesiredTilt)
{
  px4_offboard_tracking::GeometricControl control(0.5, 0.8, 0.6, 3.0, 3.0);
  // While the aircraft is still level, a newly requested tilt must not produce
  // an early collective increase and an unintended climb.
  EXPECT_NEAR(control.transient_compensated_thrust(0.0, 1.0), 0.5, 1e-12);
  EXPECT_NEAR(
    control.transient_compensated_thrust(0.0, std::cos(0.5)),
    0.5 / std::cos(0.5), 1e-12);
}

TEST(GeometricControl, ThrustSaturationReducesTiltBeforeVerticalAuthority)
{
  px4_offboard_tracking::GeometricControl control(0.5, 0.6, 0.7, 3.0, 3.0);
  px4_offboard_tracking::AccelerationCommand requested;
  requested.ax = 10.0;
  requested.ay = 0.0;
  requested.az = -1.0;
  const auto limited = control.prioritize_vertical_acceleration(requested);
  EXPECT_TRUE(limited.saturated);
  EXPECT_LT(std::abs(limited.acceleration.ax), std::abs(requested.ax));
  EXPECT_DOUBLE_EQ(limited.acceleration.az, requested.az);
  EXPECT_GT(limited.requested_normalized_thrust, 0.6);
}

TEST(GeometricControl, ThrustPriorityLeavesFeasibleAccelerationUnchanged)
{
  px4_offboard_tracking::GeometricControl control(0.5, 0.9, 0.7, 3.0, 3.0);
  px4_offboard_tracking::AccelerationCommand requested;
  requested.ax = 1.0;
  requested.ay = -0.5;
  requested.az = 0.0;
  const auto limited = control.prioritize_vertical_acceleration(requested);
  EXPECT_FALSE(limited.saturated);
  EXPECT_DOUBLE_EQ(limited.acceleration.ax, requested.ax);
  EXPECT_DOUBLE_EQ(limited.acceleration.ay, requested.ay);
  EXPECT_DOUBLE_EQ(limited.acceleration.az, requested.az);
}

TEST(GeometricControl, BodyRatesAreLimited)
{
  px4_offboard_tracking::GeometricControl control(0.5, 0.8, 0.6, 10.0, 1.0);
  px4_offboard_tracking::AttitudeThrustCommand desired;
  desired.attitude = {0.0, 1.0, 0.0, 0.0};
  const auto command = control.body_rate_from_attitude({}, desired);
  EXPECT_DOUBLE_EQ(command.roll_rate, 1.0);
}
