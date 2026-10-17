#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "px4_offboard_tracking/bearing_handover.hpp"
#include "px4_offboard_tracking/pure_mission.hpp"

using px4_offboard_tracking::BearingHandover;
using px4_offboard_tracking::BearingHandoverConfig;
using px4_offboard_tracking::BearingHandoverInput;
using px4_offboard_tracking::PureMission;
using px4_offboard_tracking::PureMissionConfig;
using px4_offboard_tracking::PureMissionInput;
using px4_offboard_tracking::PureMissionState;

namespace
{

double angle(
  const std::array<double, 3> & first, const std::array<double, 3> & second)
{
  double first_norm = 0.0;
  double second_norm = 0.0;
  double product = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    first_norm += first[axis] * first[axis];
    second_norm += second[axis] * second[axis];
    product += first[axis] * second[axis];
  }
  return std::acos(std::clamp(
    product / std::sqrt(first_norm * second_norm), -1.0, 1.0));
}

}  // namespace

TEST(BearingHandover, RangeFreeAxisConvergesWithoutAngularStep)
{
  BearingHandoverConfig config;
  config.maximum_axis_rate_rad_s = 0.8;
  config.los_rate_lead_s = 0.0;
  BearingHandover handover(config);
  const std::array<double, 3> attack{0.0, 0.0, -1.0};
  const double offset = 25.0 * 3.14159265358979323846 / 180.0;
  const std::array<double, 3> los{std::sin(offset), 0.0, -std::cos(offset)};

  auto result = handover.update({los, attack, 1.0, 1, true});
  EXPECT_LT(angle(result.commanded_axis_common, attack), 1e-12);
  double previous_error = angle(result.commanded_axis_common, los);
  for (std::uint32_t frame = 2; frame <= 30; ++frame) {
    const auto previous_axis = result.commanded_axis_common;
    result = handover.update({los, attack, 1.0 + 0.05 * (frame - 1), frame, true});
    const double step = angle(previous_axis, result.commanded_axis_common);
    EXPECT_LE(step, config.maximum_axis_rate_rad_s * 0.05 + 1e-9);
    const double error = angle(result.commanded_axis_common, los);
    EXPECT_LE(error, previous_error + 1e-10);
    previous_error = error;
  }
  EXPECT_LT(previous_error, 1e-6);
}

TEST(BearingHandover, RepeatedFrameDoesNotIntegrateOrInventRange)
{
  BearingHandover handover(BearingHandoverConfig{});
  const std::array<double, 3> attack{0.0, 0.0, -1.0};
  const std::array<double, 3> los{0.2, 0.0, -0.979795897};
  const auto first = handover.update({los, attack, 1.0, 1, true});
  const auto repeated = handover.update({los, attack, 1.05, 1, true});
  EXPECT_FALSE(repeated.new_measurement);
  EXPECT_EQ(repeated.commanded_axis_common, first.commanded_axis_common);
}

TEST(BearingHandover, ComputesFiniteLosRateForMovingBearing)
{
  BearingHandover handover(BearingHandoverConfig{});
  const std::array<double, 3> attack{0.0, 0.0, -1.0};
  handover.update({{0.0, 0.0, -1.0}, attack, 1.0, 1, true});
  const auto result = handover.update({{0.05, 0.0, -0.998749}, attack, 1.05, 2, true});
  EXPECT_TRUE(result.valid);
  EXPECT_GT(result.los_rate_common[0], 0.0);
  EXPECT_TRUE(std::isfinite(result.commanded_axis_rate_rad_s));
}

TEST(BearingHandover, CompletesRangeFreeSearchBearingDepthSequence)
{
  BearingHandoverConfig handover_config;
  handover_config.maximum_axis_rate_rad_s = 1.0;
  handover_config.los_rate_lead_s = 0.0;
  BearingHandover handover(handover_config);
  PureMissionConfig mission_config;
  mission_config.search_acquisition_hold_s = 0.0;
  mission_config.depth_acquisition_hold_s = 0.10;
  mission_config.camera_loss_hysteresis_s = 0.20;
  mission_config.minimum_depth_acquisition_frames = 4;
  PureMission mission(mission_config);
  PureMissionInput mission_input;
  mission_input.system_ready = true;
  mission.update(mission_input, 0.0);
  mission_input.takeoff_ready = true;
  mission.update(mission_input, 0.1);

  const double target_offset = 25.0 * 3.14159265358979323846 / 180.0;
  const std::array<double, 3> target_los{
    std::sin(target_offset), 0.0, -std::cos(target_offset)};
  std::array<double, 3> attack_axis{0.0, 0.0, -1.0};
  std::uint32_t depth_sequence = 0;
  for (std::uint32_t frame = 1; frame <= 30; ++frame) {
    const double time_s = 0.1 + 0.05 * frame;
    const double depth_error = angle(attack_axis, target_los);
    mission_input.search_visible = true;
    mission_input.search_sequence = frame;
    mission_input.depth_visible =
      depth_error <= 8.5 * 3.14159265358979323846 / 180.0;
    mission_input.depth_admission_ready = mission_input.depth_visible;
    if (mission_input.depth_visible) {
      mission_input.depth_sequence = ++depth_sequence;
    }
    mission.update(mission_input, time_s);
    const auto result = handover.update({
      target_los, attack_axis, time_s, frame, mission_input.search_visible});
    attack_axis = result.commanded_axis_common;
  }
  EXPECT_EQ(mission.state(), PureMissionState::DEPTH_ACQUIRE);
  EXPECT_GE(depth_sequence, mission_config.minimum_depth_acquisition_frames);
  EXPECT_LE(
    angle(attack_axis, target_los),
    8.5 * 3.14159265358979323846 / 180.0);
}
