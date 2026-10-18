#include <gtest/gtest.h>

#include "netgun_intercept/netgun_fire_control.hpp"

#include <array>
#include <cmath>

namespace
{

using netgun_intercept::FireControlInput;
using netgun_intercept::FireDecisionReason;
using netgun_intercept::FireControlConfig;
using netgun_intercept::NetgunFireControl;

constexpr double kDeg = M_PI / 180.0;

FireControlInput make_input(double now_s, double axis_error_deg, double range_m = 8.0)
{
  FireControlInput input;
  input.now_s = now_s;
  input.estimate_valid = true;
  input.authorization_passed = true;
  input.range_m = range_m;
  input.axis_error_rad = axis_error_deg * kDeg;
  input.los_rate_rad_s = 0.0;
  return input;
}

TEST(FireControl, PrimaryWindowFiresImmediatelyAtOneDegree)
{
  NetgunFireControl control;
  const auto decision = control.update(make_input(1.0, 1.0));
  EXPECT_TRUE(decision.fire);
  EXPECT_EQ(decision.reason, FireDecisionReason::PRIMARY_WINDOW);
  EXPECT_EQ(decision.shot_id, 1);
  EXPECT_TRUE(control.latched());
}

TEST(FireControl, PrimaryWindowFiresBelowOneDegree)
{
  NetgunFireControl control;
  EXPECT_TRUE(control.update(make_input(1.0, 0.4)).fire);
}

TEST(FireControl, WindowHoldFiresAfterPointTwoSeconds)
{
  NetgunFireControl control;
  EXPECT_FALSE(control.update(make_input(10.0, 1.5)).fire);
  EXPECT_FALSE(control.update(make_input(10.1, 1.5)).fire);
  EXPECT_FALSE(control.update(make_input(10.19, 1.5)).fire);
  const auto decision = control.update(make_input(10.2, 1.5));
  EXPECT_TRUE(decision.fire);
  EXPECT_EQ(decision.reason, FireDecisionReason::WINDOW_HOLD);
}

TEST(FireControl, WindowHoldResetsWhenLeavingWindow)
{
  NetgunFireControl control;
  EXPECT_FALSE(control.update(make_input(0.0, 1.5)).fire);
  EXPECT_FALSE(control.update(make_input(0.1, 1.5)).fire);
  // 离窗（>2°）重置计时。
  EXPECT_FALSE(control.update(make_input(0.15, 2.5)).fire);
  EXPECT_FALSE(control.update(make_input(0.16, 1.5)).fire);
  EXPECT_FALSE(control.update(make_input(0.3, 1.5)).fire);
  const auto decision = control.update(make_input(0.36, 1.5));
  EXPECT_TRUE(decision.fire);
  EXPECT_EQ(decision.reason, FireDecisionReason::WINDOW_HOLD);
}

TEST(FireControl, SingleShotLatchIsPermanent)
{
  NetgunFireControl control;
  control.update(make_input(1.0, 0.5));
  for (int i = 0; i < 5; ++i) {
    const auto decision = control.update(make_input(2.0 + i * 0.1, 0.5));
    EXPECT_FALSE(decision.fire);
    EXPECT_EQ(decision.reason, FireDecisionReason::LATCHED);
  }
  EXPECT_EQ(control.shot_count(), 1);
}

TEST(FireControl, AuthorizationAndValidityGatesBlockFire)
{
  NetgunFireControl control;
  auto unauthorized = make_input(1.0, 0.5);
  unauthorized.authorization_passed = false;
  EXPECT_FALSE(control.update(unauthorized).fire);

  auto invalid = make_input(1.0, 0.5);
  invalid.estimate_valid = false;
  EXPECT_FALSE(control.update(invalid).fire);

  EXPECT_FALSE(control.update(make_input(1.0, 0.5, 5.5)).fire);
  EXPECT_FALSE(control.update(make_input(1.0, 0.5, 10.5)).fire);

  auto fast_los = make_input(1.0, 0.5);
  fast_los.los_rate_rad_s = 0.6;
  EXPECT_FALSE(control.update(fast_los).fire);
  // 门恢复后仍可触发（未闭锁）。
  EXPECT_TRUE(control.update(make_input(1.1, 0.5)).fire);
}

TEST(FireControl, BlindShotDisabledByDefault)
{
  NetgunFireControl control;
  auto exiting = make_input(1.0, 5.0, 9.9);
  exiting.range_rate_mps = 2.0;  // 9.9 + 2*0.25 = 10.4 > 10 即将穿出
  exiting.engagement_timeout = true;
  EXPECT_FALSE(control.update(exiting).fire);
}

TEST(FireControl, BlindShotInterfaceWorksWhenEnabled)
{
  FireControlConfig config;
  config.blind_shot_enabled = true;
  NetgunFireControl control(config);

  auto exiting = make_input(1.0, 5.0, 9.9);
  exiting.range_rate_mps = 2.0;
  const auto exit_decision = control.update(exiting);
  EXPECT_TRUE(exit_decision.fire);
  EXPECT_EQ(exit_decision.reason, FireDecisionReason::BLIND_SHOT_EXIT);
}

TEST(FireControl, BlindShotRespectsErrorBound)
{
  FireControlConfig config;
  config.blind_shot_enabled = true;
  NetgunFireControl control(config);
  // 9° > 蒙射门 8°：即使窗口即将穿出也不打。
  auto too_off = make_input(1.0, 9.0, 9.9);
  too_off.range_rate_mps = 2.0;
  EXPECT_FALSE(control.update(too_off).fire);
  // 2° 在门内但属于判窗/蒙射重叠区——主支兜底已覆盖，此处验证不因蒙射误触。
  EXPECT_TRUE(control.update(make_input(1.05, 1.0)).fire);
}

TEST(FireControl, BlindShotTimeoutBranch)
{
  FireControlConfig config;
  config.blind_shot_enabled = true;
  NetgunFireControl control(config);
  auto timed_out = make_input(1.0, 5.0, 8.0);
  timed_out.engagement_timeout = true;
  const auto decision = control.update(timed_out);
  EXPECT_TRUE(decision.fire);
  EXPECT_EQ(decision.reason, FireDecisionReason::BLIND_SHOT_TIMEOUT);
}

TEST(FireControl, InvalidInputsAreSafe)
{
  NetgunFireControl control;
  auto nan_error = make_input(1.0, 0.5);
  nan_error.axis_error_rad = std::nan("");
  EXPECT_FALSE(control.update(nan_error).fire);
  EXPECT_FALSE(control.update(make_input(std::nan(""), 0.5)).fire);
}

// ---- F-2：sigma_range_lateral 必须读 9×9 行主序的位置块 ----

using netgun_intercept::RangeLateralSigma;
using netgun_intercept::sigma_range_lateral;

// 位置块对角 + 各阶互协方差标记值（应被忽略）。
std::array<double, 81> make_cov(double p00, double p11, double p22)
{
  std::array<double, 81> cov{};
  for (int i = 0; i < 81; ++i) {
    cov[i] = 99.0 + i;  // 标记值：任何位置块之外的下标读到都算错
  }
  cov[9 * 0 + 0] = p00;
  cov[9 * 1 + 1] = p11;
  cov[9 * 2 + 2] = p22;
  return cov;
}

// 位置块和第 0 行不同值：结果只取决于位置块（旧 3×3 下标读到的是第 0 行）。
TEST(SigmaRangeLateral, UsesPositionBlockNotFirstRow)
{
  auto cov = make_cov(0.04, 0.09, 0.25);
  const double u[3] = {0.0, 0.0, 1.0};
  const auto sigma = sigma_range_lateral(cov, u);
  EXPECT_NEAR(sigma.range_m, std::sqrt(0.25), 1e-12);
  EXPECT_NEAR(sigma.lateral_m, std::sqrt((0.04 + 0.09) / 2.0), 1e-12);
}

// 目标正上方：u=(0,0,-1)，σ_range = sqrt(P22)。
TEST(SigmaRangeLateral, OverheadMatchesDiagonal)
{
  auto cov = make_cov(0.04, 0.09, 0.25);
  const double u[3] = {0.0, 0.0, -1.0};
  const auto sigma = sigma_range_lateral(cov, u);
  EXPECT_NEAR(sigma.range_m, std::sqrt(0.25), 1e-12);
}

// 偏轴 45°：σ_range² = 0.5(P00+P11) + P01（对称）。
TEST(SigmaRangeLateral, OffAxisMatchesAnalytic)
{
  auto cov = make_cov(0.04, 0.09, 0.25);
  cov[9 * 0 + 1] = cov[9 * 1 + 0] = 0.02;
  const double s = 1.0 / std::sqrt(2.0);
  const double u[3] = {s, s, 0.0};
  const auto sigma = sigma_range_lateral(cov, u);
  const double expected2 = 0.5 * (0.04 + 0.09) + 0.02;
  EXPECT_NEAR(sigma.range_m, std::sqrt(expected2), 1e-12);
  EXPECT_NEAR(sigma.lateral_m, std::sqrt((0.04 + 0.09 + 0.25 - expected2) / 2.0),
    1e-12);
}

// ---- H-5：枪口偏置与枪管轴安装角 ----

using netgun_intercept::apply_muzzle_offset;
using netgun_intercept::gun_axis_body;

// 瞄准点平移量正好等于偏置（NED 各分量）。
TEST(MuzzleOffset, AimPointShiftsExactlyByOffset)
{
  const std::array<double, 3> aim{7.0, -2.5, -6.0};
  const std::array<double, 3> offset{0.10, -0.05, 0.20};
  const auto shifted = apply_muzzle_offset(aim, offset);
  EXPECT_NEAR(shifted[0], aim[0] - offset[0], 1e-12);
  EXPECT_NEAR(shifted[1], aim[1] - offset[1], 1e-12);
  EXPECT_NEAR(shifted[2], aim[2] - offset[2], 1e-12);
  // 零偏置 = 旧口径逐位一致
  const auto unshifted = apply_muzzle_offset(aim, {0.0, 0.0, 0.0});
  EXPECT_EQ(unshifted[0], aim[0]);
  EXPECT_EQ(unshifted[1], aim[1]);
  EXPECT_EQ(unshifted[2], aim[2]);
}

// 枪管轴：默认与 body -Z 同轴；俯仰/偏航按 Rz(yaw)·Ry(pitch) 旋转。
TEST(MuzzleOffset, GunAxisBodyDefaultAndTilted)
{
  const auto axis = gun_axis_body(0.0, 0.0);
  EXPECT_NEAR(axis[0], 0.0, 1e-12);
  EXPECT_NEAR(axis[1], 0.0, 1e-12);
  EXPECT_NEAR(axis[2], -1.0, 1e-12);

  const double pitch = 3.0 * M_PI / 180.0;
  const double yaw = -2.0 * M_PI / 180.0;
  const auto tilted = gun_axis_body(pitch, yaw);
  const double sin_p = std::sin(pitch);
  const double cos_p = std::cos(pitch);
  const double sin_y = std::sin(yaw);
  const double cos_y = std::cos(yaw);
  EXPECT_NEAR(tilted[0], -sin_p * cos_y, 1e-12);
  EXPECT_NEAR(tilted[1], -sin_p * sin_y, 1e-12);
  EXPECT_NEAR(tilted[2], -cos_p, 1e-12);
  // 单位向量
  EXPECT_NEAR(
    std::sqrt(tilted[0] * tilted[0] + tilted[1] * tilted[1] +
      tilted[2] * tilted[2]),
    1.0, 1e-12);
}

}  // namespace
