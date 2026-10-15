#pragma once

#include <array>
#include <cstdint>

namespace virtual_interception_referee
{

std::array<double, 3> position_in_task_frame(
  const std::array<double, 3> & local_position_ned,
  const std::array<double, 3> & task_origin_ned);

std::array<double, 3> body_minus_z_in_ned(
  const std::array<double, 4> & quaternion_wxyz);

bool truth_interval_usable(
  double previous_sample_time_s, double current_sample_time_s,
  double maximum_truth_gap_s);

struct RefereeConfig
{
  double projectile_speed_mps{400.0};
  double gravity_mps2{9.80665};
  // Fixed-radius fallback for legacy tests or a zero-angle configuration.
  double hit_radius_m{0.4};
  double maximum_flight_time_s{0.25};
  double integration_step_s{0.0001};
  // Half-angle of the measured effective strike cone. A positive value makes
  // the hit radius grow with projectile travel instead of using hit_radius_m.
  double effect_cone_half_angle_rad{0.0};
  // 网枪 3×3 覆盖面即时判定模式（方案 v4 §5）：开启后不做弹丸运动学外推，
  // 触发时刻目标相对射击轴横向偏移 |x|,|y| ≤ coverage_half_extent_m 且
  // 距离 ∈ [effective_range_min_m, effective_range_max_m] 即命中。
  // 默认关闭：旧虚拟弹丸链行为不变；网枪链在其 launch/config 中显式开启。
  bool instant_coverage_mode{false};
  double coverage_half_extent_m{1.5};
  double effective_range_min_m{6.0};
  double effective_range_max_m{10.0};
};

struct RefereeShot
{
  std::array<double, 3> origin_ned{};
  std::array<double, 3> pursuer_velocity_ned{};
  std::array<double, 3> direction_ned{};
};

struct RefereeSegment
{
  double start_time_s{};
  double end_time_s{};
  std::array<double, 3> target_start_ned{};
  std::array<double, 3> target_end_ned{};
};

struct RefereeEvaluation
{
  double minimum_miss_distance_m{};
  double effective_hit_radius_m{};
  double minimum_clearance_m{};
  double closest_time_s{};
  std::array<double, 3> projectile_closest_ned{};
  std::array<double, 3> target_closest_ned{};
  bool hit{false};
  bool valid{false};
};

class RefereeModel
{
public:
  explicit RefereeModel(const RefereeConfig & config);

  std::array<double, 3> projectile_position(
    const RefereeShot & shot, double elapsed_s) const;
  RefereeEvaluation evaluate_segment(
    const RefereeShot & shot, const RefereeSegment & segment) const;
  const RefereeConfig & config() const;

private:
  RefereeConfig config_;
};

struct MissionHitCounterConfig
{
  double minimum_hit_interval_s{3.0};
  std::uint32_t required_hits{2};
};

class MissionHitCounter
{
public:
  explicit MissionHitCounter(const MissionHitCounterConfig & config);

  bool accept(bool hit, double shot_time_s);
  std::uint32_t successful_hit_count() const;
  bool mission_success() const;

private:
  MissionHitCounterConfig config_;
  double last_counted_hit_time_s_{-1.0};
  std::uint32_t successful_hit_count_{0};
};

}  // namespace virtual_interception_referee
