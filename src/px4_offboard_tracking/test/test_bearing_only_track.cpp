#include <gtest/gtest.h>

#include <cmath>
#include <array>

#include "px4_offboard_tracking/bearing_only_measurement.hpp"
#include "px4_offboard_tracking/relative_ekf.hpp"

// H-1 回归（审查单 T1-T5）：近距目标首次被看到时，感知要连续 3 帧
// 确认才输出深度，前 2 帧只有方位。估计器按 12 m 名义距离建轨且位置
// 协方差 1 m 过度自信；修复 = 首帧深度无条件重锚（H-1a，节点侧同款
// 判定在 EstimatorMirror 中镜像），本文件在无 ROS 环境下用真实滤波器
// 代码锁住该行为。

namespace
{

using px4_offboard_tracking::BearingOnlyMeasurement;
using px4_offboard_tracking::CameraExtrinsics;
using px4_offboard_tracking::CameraIntrinsics;
using px4_offboard_tracking::CameraModel;
using px4_offboard_tracking::Quaterniond;
using px4_offboard_tracking::RelativeImmEkf;
using px4_offboard_tracking::RelativeImmEkfConfig;
using px4_offboard_tracking::Vector3d;

constexpr double kDt = 1.0 / 15.0;  // 15 Hz

// 实机 launch（netgun_real.launch.py）参数
CameraIntrinsics real_intrinsics()
{
  return CameraIntrinsics{1266.5, 1266.5, 1011.7, 538.4, 1920, 1080, 0.3};
}

CameraModel real_camera()
{
  // 与 relative_estimator 默认一致：竖直向上、宽轴沿机头（H-4 锁定值）
  return CameraModel(
    real_intrinsics(),
    CameraExtrinsics{{}, CameraModel::default_upward_optical_quaternion()});
}

Quaterniond identity()
{
  return Quaterniond{};
}

Vector3d rotate_quat(const Quaterniond & q, const Vector3d & v)
{
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  return {
    (1.0 - 2.0 * (y * y + z * z)) * v.x + 2.0 * (x * y - z * w) * v.y +
      2.0 * (x * z + y * w) * v.z,
    2.0 * (x * y + z * w) * v.x + (1.0 - 2.0 * (x * x + z * z)) * v.y +
      2.0 * (y * z - x * w) * v.z,
    2.0 * (x * z - y * w) * v.x + 2.0 * (y * z + x * w) * v.y +
      (1.0 - 2.0 * (x * x + y * y)) * v.z};
}

// 桥常量协方差（横向 5 cm、深度 30 cm，光学系对角）旋到 NED。
std::array<double, 9> depth_covariance_ned(const CameraModel & camera)
{
  const Quaterniond q_camera_to_ned = camera.q_camera_to_ned(identity());
  const Vector3d axes[3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  const double variance[3] = {0.05 * 0.05, 0.05 * 0.05, 0.30 * 0.30};
  double rotation[9];
  for (int k = 0; k < 3; ++k) {
    const Vector3d column = rotate_quat(q_camera_to_ned, axes[k]);
    rotation[k] = column.x;
    rotation[3 + k] = column.y;
    rotation[6 + k] = column.z;
  }
  std::array<double, 9> out{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += rotation[3 * row + k] * variance[k] * rotation[3 * column + k];
      }
      out[3 * row + column] = sum;
    }
  }
  return out;
}

// relative_estimator 节点的无 ROS 镜像：bearing-only 伪量测 + 深度量测
// + H-1a 首帧深度重锚判定（last_depth_accept <= 0 语义）。
class EstimatorMirror
{
public:
  EstimatorMirror()
    : camera_(real_camera()),
      filter_(RelativeImmEkfConfig{[] {
        RelativeImmEkfConfig config;
        config.filter.acceleration_noise_stddev_mps2 = 4.0;
        config.filter.innovation_gate_mahalanobis = 25.0;
        return config;
      }()})
  {
  }

  bool feed_bearing_only(double timestamp, const Vector3d & target_ned)
  {
    const auto projection = camera_.project(target_ned, identity());
    EXPECT_TRUE(projection.visible);
    const auto predicted = filter_.predicted_state(timestamp);
    const auto measurement = make_bearing_only_measurement(
      camera_, real_intrinsics(), identity(),
      projection.pixel_u, projection.pixel_v, 1.5,
      12.0, 30.0,
      Vector3d{
        predicted.position[0], predicted.position[1], predicted.position[2]});
    std::array<double, 9> covariance{};
    for (int i = 0; i < 9; ++i) {
      covariance[i] = measurement.covariance_ned[i];
    }
    return filter_.update(
      {measurement.position_ned.x, measurement.position_ned.y,
        measurement.position_ned.z},
      covariance, timestamp);
  }

  bool feed_depth(double timestamp, const Vector3d & target_ned)
  {
    // H-1a：自初始化/重置以来没有接受过深度 → 首帧深度无条件重锚
    if (filter_.state().initialized && !depth_ever_accepted_) {
      filter_.reset();
      ++reanchor_count;
    }
    const auto projection = camera_.project(target_ned, identity());
    const auto relative = camera_.camera_position_to_relative_ned(
      projection.position_camera, identity());
    const bool accepted = filter_.update(
      {relative.x, relative.y, relative.z}, depth_covariance_ned(camera_),
      timestamp);
    if (accepted) {
      depth_ever_accepted_ = true;
    }
    return accepted;
  }

  const RelativeImmEkf & filter() const {return filter_;}
  int reanchor_count = 0;

private:
  CameraModel camera_;
  RelativeImmEkf filter_;
  bool depth_ever_accepted_{false};
};

Vector3d target_at(double range_m, double off_axis_deg)
{
  // 仰角朝北（+x 为北）：off_axis=0 → 正上方
  const double angle = off_axis_deg * M_PI / 180.0;
  return Vector3d{
    range_m * std::sin(angle), 0.0, -range_m * std::cos(angle)};
}

double position_error(const RelativeImmEkf & filter, const Vector3d & truth)
{
  const auto state = filter.state();
  return std::sqrt(
    (state.position[0] - truth.x) * (state.position[0] - truth.x) +
    (state.position[1] - truth.y) * (state.position[1] - truth.y) +
    (state.position[2] - truth.z) * (state.position[2] - truth.z));
}

double speed_error(const RelativeImmEkf & filter, const Vector3d & truth_velocity)
{
  const auto state = filter.state();
  return std::sqrt(
    (state.velocity[0] - truth_velocity.x) *
      (state.velocity[0] - truth_velocity.x) +
    (state.velocity[1] - truth_velocity.y) *
      (state.velocity[1] - truth_velocity.y) +
    (state.velocity[2] - truth_velocity.z) *
      (state.velocity[2] - truth_velocity.z));
}

}  // namespace

// 纯函数语义：预测位置为零（未初始化）时用先验名义距离；预测在视线上
// 且投影 >1 m 时用投影距离。
TEST(BearingOnlyMeasurement, NominalRangePriorAndProjection)
{
  const CameraModel camera = real_camera();
  const Vector3d target = target_at(7.5, 30.0);
  const auto projection = camera.project(target, identity());
  ASSERT_TRUE(projection.visible);

  const auto uninitialized = make_bearing_only_measurement(
    camera, real_intrinsics(), identity(),
    projection.pixel_u, projection.pixel_v, 1.5, 12.0, 30.0,
    Vector3d{0.0, 0.0, 0.0});
  EXPECT_NEAR(uninitialized.nominal_range_m, 12.0, 1e-12);

  const Vector3d predicted{1.0, 0.0, -5.0};
  const auto initialized = make_bearing_only_measurement(
    camera, real_intrinsics(), identity(),
    projection.pixel_u, projection.pixel_v, 1.5, 12.0, 30.0, predicted);
  // 视线方向 = 目标方向，预测在视线上 → 投影 ≈ |predicted|
  EXPECT_NEAR(initialized.nominal_range_m, std::hypot(1.0, 5.0), 1e-9);
  // 伪量测位置在目标视线上（名义距离缩放）
  EXPECT_NEAR(
    initialized.position_ned.x,
    std::hypot(1.0, 5.0) * std::sin(30.0 * M_PI / 180.0), 0.05);
  EXPECT_NEAR(
    initialized.position_ned.z,
    -std::hypot(1.0, 5.0) * std::cos(30.0 * M_PI / 180.0), 0.05);
}

// T1：斜视 30°、7.5 m 静止目标，[BO, BO, DEPTH×20]——首帧深度被接受
// （不拒收）、重锚一次，其后位置/速度误差贴零。
TEST(BearingOnlyTrack, T1ObliqueCaptureReanchorsOnFirstDepth)
{
  EstimatorMirror mirror;
  const Vector3d truth = target_at(7.5, 30.0);
  double t = 0.0;
  mirror.feed_bearing_only(t += kDt, truth);
  mirror.feed_bearing_only(t += kDt, truth);
  // 纯方位期位置=名义 12 m 点，与真值差约 4.5 m——这正是 H-1 要消除的
  // 错误站位来源（制导侧由 H-1b 的 est_pos_ok 挡住）
  EXPECT_GT(position_error(mirror.filter(), truth), 2.0);
  for (int frame = 0; frame < 20; ++frame) {
    EXPECT_TRUE(mirror.feed_depth(t += kDt, truth)) << "frame " << frame;
    if (frame == 0) {
      EXPECT_EQ(mirror.reanchor_count, 1);
    }
    EXPECT_LT(position_error(mirror.filter(), truth), 0.05);
    EXPECT_LT(speed_error(mirror.filter(), Vector3d{}), 0.1);
  }
}

// T2：正上方 4.5 m，[BO, BO, DEPTH×20]——现状（无重锚）首帧深度 NIS
// 21.9 起连续被拒 4 帧；修复后没有任何深度帧被拒收。
TEST(BearingOnlyTrack, T2OverheadCloseNoDepthRejection)
{
  EstimatorMirror mirror;
  const Vector3d truth = target_at(4.5, 0.0);
  double t = 0.0;
  mirror.feed_bearing_only(t += kDt, truth);
  mirror.feed_bearing_only(t += kDt, truth);
  for (int frame = 0; frame < 20; ++frame) {
    EXPECT_TRUE(mirror.feed_depth(t += kDt, truth)) << "frame " << frame;
    EXPECT_LT(position_error(mirror.filter(), truth), 0.05);
    EXPECT_LT(speed_error(mirror.filter(), Vector3d{}), 0.1);
  }
  EXPECT_EQ(mirror.reanchor_count, 1);
}

// T3：远距接近 16 m→10.5 m（逼近 2 m/s，纯方位约 2 s），进量程后
// [BO, BO, DEPTH×…]——首帧深度后 0.5 s 内速度误差 < 0.3 m/s。
TEST(BearingOnlyTrack, T3FarApproachVelocityConvergesAfterReanchor)
{
  EstimatorMirror mirror;
  double t = 0.0;
  double range = 16.0;
  // 2 m/s 沿视线朝本机逼近
  const Vector3d far = target_at(16.0, 30.0);
  const double far_norm =
    std::sqrt(far.x * far.x + far.y * far.y + far.z * far.z);
  const Vector3d closing_velocity{
    -far.x / far_norm * 2.0, -far.y / far_norm * 2.0,
    -far.z / far_norm * 2.0};
  int bearing_frames = 0;
  while (range > 10.5) {
    mirror.feed_bearing_only(t += kDt, target_at(range, 30.0));
    ++bearing_frames;
    range -= 2.0 * kDt;
  }
  EXPECT_GT(bearing_frames, 20);  // 纯方位约 2 s
  mirror.feed_bearing_only(t += kDt, target_at(range, 30.0));
  mirror.feed_bearing_only(t += kDt, target_at(range, 30.0));
  double first_depth_t = -1.0;
  for (int frame = 0; frame < 40 && range > 4.0; ++frame) {
    t += kDt;
    range -= 2.0 * kDt;
    const Vector3d truth = target_at(range, 30.0);
    mirror.feed_depth(t, truth);
    if (first_depth_t < 0.0) {
      first_depth_t = t;
      EXPECT_EQ(mirror.reanchor_count, 1);
      continue;
    }
    if (t - first_depth_t >= 0.5) {
      EXPECT_LT(speed_error(mirror.filter(), closing_velocity), 0.3);
    }
    EXPECT_LT(position_error(mirror.filter(), truth), 0.6);
  }
  EXPECT_GT(first_depth_t, 0.0);
}

// T3b：正上方 6.5 m，纯方位 1.5 s 后给深度——现状伪速度 4.8 m/s；
// 无条件重锚后首帧深度起 |v| < 0.1。
TEST(BearingOnlyTrack, T3bLongBearingOnlyStillReanchors)
{
  EstimatorMirror mirror;
  const Vector3d truth = target_at(6.5, 0.0);
  double t = 0.0;
  while (t < 1.5) {
    mirror.feed_bearing_only(t += kDt, truth);
  }
  for (int frame = 0; frame < 20; ++frame) {
    EXPECT_TRUE(mirror.feed_depth(t += kDt, truth));
    EXPECT_LT(speed_error(mirror.filter(), Vector3d{}), 0.1);
    EXPECT_LT(position_error(mirror.filter(), truth), 0.05);
  }
  EXPECT_EQ(mirror.reanchor_count, 1);
}

// T4：深度 1 s → 纯方位 6 帧（深度短暂丢失）→ 深度恢复——不重锚、
// 无重置，恢复时位置不跳变。
TEST(BearingOnlyTrack, T4DepthLossThenRecoveryKeepsTrack)
{
  EstimatorMirror mirror;
  const Vector3d truth = target_at(7.0, 15.0);
  double t = 0.0;
  for (int frame = 0; frame < 15; ++frame) {
    mirror.feed_depth(t += kDt, truth);
  }
  EXPECT_EQ(mirror.reanchor_count, 0);  // 首帧即深度：无需重锚
  const auto position_before = mirror.filter().state().position;
  for (int frame = 0; frame < 6; ++frame) {
    mirror.feed_bearing_only(t += kDt, truth);
  }
  // H-1a：已有过深度 → 短暂丢失后恢复走正常卡尔曼更新，不重锚
  EXPECT_EQ(mirror.reanchor_count, 0);
  mirror.feed_depth(t += kDt, truth);
  EXPECT_EQ(mirror.reanchor_count, 0);
  const auto position_after = mirror.filter().state().position;
  EXPECT_LT(
    std::sqrt(
      (position_after[0] - truth.x) * (position_after[0] - truth.x) +
      (position_after[1] - truth.y) * (position_after[1] - truth.y) +
      (position_after[2] - truth.z) * (position_after[2] - truth.z)),
    0.05);
}

// T5：目标 3 m/s，[BO, BO, DEPTH×20]——首帧深度后 0.5 s 内速度误差
// 收敛到 0.5 m/s 内。
TEST(BearingOnlyTrack, T5MovingTargetVelocityConvergence)
{
  EstimatorMirror mirror;
  double t = 0.0;
  const double speed = 3.0;
  mirror.feed_bearing_only(t += kDt, target_at(7.5, 30.0));
  mirror.feed_bearing_only(t += kDt, target_at(7.5, 30.0));
  const Vector3d truth_velocity{speed, 0.0, 0.0};
  double first_depth_t = -1.0;
  bool velocity_checked = false;
  for (int frame = 0; frame < 40; ++frame) {
    t += kDt;
    Vector3d truth = target_at(7.5, 30.0);
    truth.x += speed * t;
    mirror.feed_depth(t, truth);
    if (first_depth_t < 0.0) {
      first_depth_t = t;
      continue;
    }
    if (t - first_depth_t >= 0.5) {
      EXPECT_LT(speed_error(mirror.filter(), truth_velocity), 0.5);
      velocity_checked = true;
    }
  }
  EXPECT_TRUE(velocity_checked);
}
