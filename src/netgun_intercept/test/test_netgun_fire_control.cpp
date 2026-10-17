#include <gtest/gtest.h>

#include "netgun_intercept/netgun_fire_control.hpp"

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

}  // namespace
