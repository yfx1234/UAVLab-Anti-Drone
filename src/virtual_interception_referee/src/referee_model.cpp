#include "virtual_interception_referee/referee_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace virtual_interception_referee
{
namespace
{

using Vector = std::array<double, 3>;

double norm(const Vector & value)
{
  return std::sqrt(
    value[0] * value[0] +
    value[1] * value[1] +
    value[2] * value[2]);
}

bool finite(const Vector & value)
{
  return
    std::isfinite(value[0]) &&
    std::isfinite(value[1]) &&
    std::isfinite(value[2]);
}

Vector interpolate(const Vector & start, const Vector & end, double fraction)
{
  return {
    start[0] + fraction * (end[0] - start[0]),
    start[1] + fraction * (end[1] - start[1]),
    start[2] + fraction * (end[2] - start[2])};
}

}  // namespace

std::array<double, 3> position_in_task_frame(
  const std::array<double, 3> & local_position_ned,
  const std::array<double, 3> & task_origin_ned)
{
  return {
    local_position_ned[0] - task_origin_ned[0],
    local_position_ned[1] - task_origin_ned[1],
    local_position_ned[2] - task_origin_ned[2]};
}

std::array<double, 3> body_minus_z_in_ned(
  const std::array<double, 4> & quaternion_wxyz)
{
  const double length = std::sqrt(
    quaternion_wxyz[0] * quaternion_wxyz[0] +
    quaternion_wxyz[1] * quaternion_wxyz[1] +
    quaternion_wxyz[2] * quaternion_wxyz[2] +
    quaternion_wxyz[3] * quaternion_wxyz[3]);
  if (!std::isfinite(length) || length < 1.0e-9) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan, nan};
  }
  const double w = quaternion_wxyz[0] / length;
  const double x = quaternion_wxyz[1] / length;
  const double y = quaternion_wxyz[2] / length;
  const double z = quaternion_wxyz[3] / length;
  // Third column of the body-to-NED rotation matrix, negated for Body -Z.
  return {
    -2.0 * (x * z + w * y),
    -2.0 * (y * z - w * x),
    -(1.0 - 2.0 * (x * x + y * y))};
}

bool truth_interval_usable(
  double previous_sample_time_s, double current_sample_time_s,
  double maximum_truth_gap_s)
{
  return std::isfinite(previous_sample_time_s) &&
         std::isfinite(current_sample_time_s) &&
         std::isfinite(maximum_truth_gap_s) &&
         maximum_truth_gap_s > 0.0 &&
         current_sample_time_s >= previous_sample_time_s &&
         current_sample_time_s - previous_sample_time_s <= maximum_truth_gap_s;
}

RefereeModel::RefereeModel(const RefereeConfig & config)
: config_(config)
{
  if (config_.projectile_speed_mps <= 0.0 ||
    config_.gravity_mps2 < 0.0 ||
    config_.hit_radius_m <= 0.0 ||
    config_.maximum_flight_time_s <= 0.0 ||
    config_.integration_step_s <= 0.0 ||
    config_.integration_step_s > config_.maximum_flight_time_s ||
    !std::isfinite(config_.effect_cone_half_angle_rad) ||
    config_.effect_cone_half_angle_rad < 0.0 ||
    config_.effect_cone_half_angle_rad >= 1.5707963267948966)
  {
    throw std::invalid_argument("invalid virtual interception referee configuration");
  }
}

std::array<double, 3> RefereeModel::projectile_position(
  const RefereeShot & shot, double elapsed_s) const
{
  if (!finite(shot.origin_ned) ||
    !finite(shot.pursuer_velocity_ned) ||
    !finite(shot.direction_ned) ||
    !std::isfinite(elapsed_s) ||
    elapsed_s < 0.0)
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan, nan};
  }
  return {
    shot.origin_ned[0] +
    (shot.pursuer_velocity_ned[0] +
    config_.projectile_speed_mps * shot.direction_ned[0]) * elapsed_s,
    shot.origin_ned[1] +
    (shot.pursuer_velocity_ned[1] +
    config_.projectile_speed_mps * shot.direction_ned[1]) * elapsed_s,
    shot.origin_ned[2] +
    (shot.pursuer_velocity_ned[2] +
    config_.projectile_speed_mps * shot.direction_ned[2]) * elapsed_s +
    0.5 * config_.gravity_mps2 * elapsed_s * elapsed_s};
}

RefereeEvaluation RefereeModel::evaluate_segment(
  const RefereeShot & shot, const RefereeSegment & segment) const
{
  RefereeEvaluation output;
  output.minimum_miss_distance_m = std::numeric_limits<double>::infinity();
  output.minimum_clearance_m = std::numeric_limits<double>::infinity();
  if (!finite(segment.target_start_ned) ||
    !finite(segment.target_end_ned) ||
    !std::isfinite(segment.start_time_s) ||
    !std::isfinite(segment.end_time_s) ||
    segment.start_time_s < 0.0 ||
    segment.end_time_s < segment.start_time_s ||
    segment.end_time_s > config_.maximum_flight_time_s)
  {
    return output;
  }

  if (config_.instant_coverage_mode) {
    // 网枪 3×3 覆盖面即时判定（方案 v4 §5）：触发时刻目标相对射击轴的
    // 横向偏移落在正方形覆盖内且距离在有效窗内即命中；无弹丸外推。
    const Vector to_target{
      segment.target_start_ned[0] - shot.origin_ned[0],
      segment.target_start_ned[1] - shot.origin_ned[1],
      segment.target_start_ned[2] - shot.origin_ned[2]};
    const double range = norm(to_target);
    const double along_axis =
      to_target[0] * shot.direction_ned[0] +
      to_target[1] * shot.direction_ned[1] +
      to_target[2] * shot.direction_ned[2];
    const Vector lateral{
      to_target[0] - along_axis * shot.direction_ned[0],
      to_target[1] - along_axis * shot.direction_ned[1],
      to_target[2] - along_axis * shot.direction_ned[2]};
    // 与射击轴正交的确定论基底：取世界系中与轴最不平行的轴叉积。
    const Vector reference = std::abs(shot.direction_ned[2]) < 0.9 ?
      Vector{0.0, 0.0, 1.0} : Vector{1.0, 0.0, 0.0};
    const Vector basis_u{
      shot.direction_ned[1] * reference[2] - shot.direction_ned[2] * reference[1],
      shot.direction_ned[2] * reference[0] - shot.direction_ned[0] * reference[2],
      shot.direction_ned[0] * reference[1] - shot.direction_ned[1] * reference[0]};
    const double basis_u_norm = norm(basis_u);
    if (basis_u_norm < 1e-9) {return output;}
    const Vector u_hat{
      basis_u[0] / basis_u_norm, basis_u[1] / basis_u_norm, basis_u[2] / basis_u_norm};
    const Vector v_hat{
      shot.direction_ned[1] * u_hat[2] - shot.direction_ned[2] * u_hat[1],
      shot.direction_ned[2] * u_hat[0] - shot.direction_ned[0] * u_hat[2],
      shot.direction_ned[0] * u_hat[1] - shot.direction_ned[1] * u_hat[0]};
    const double lateral_u = std::abs(
      lateral[0] * u_hat[0] + lateral[1] * u_hat[1] + lateral[2] * u_hat[2]);
    const double lateral_v = std::abs(
      lateral[0] * v_hat[0] + lateral[1] * v_hat[1] + lateral[2] * v_hat[2]);
    output.minimum_miss_distance_m = norm(lateral);
    output.effective_hit_radius_m = config_.coverage_half_extent_m;
    output.minimum_clearance_m =
      std::max(lateral_u, lateral_v) - config_.coverage_half_extent_m;
    output.closest_time_s = segment.start_time_s;
    output.projectile_closest_ned = {
      shot.origin_ned[0] + along_axis * shot.direction_ned[0],
      shot.origin_ned[1] + along_axis * shot.direction_ned[1],
      shot.origin_ned[2] + along_axis * shot.direction_ned[2]};
    output.target_closest_ned = segment.target_start_ned;
    const bool range_ok = range >= config_.effective_range_min_m &&
      range <= config_.effective_range_max_m;
    output.hit = range_ok && output.minimum_clearance_m <= 0.0;
    output.valid = std::isfinite(range);
    return output;
  }

  const double duration = segment.end_time_s - segment.start_time_s;
  const std::size_t steps = std::max<std::size_t>(
    1, static_cast<std::size_t>(
      std::ceil(duration / config_.integration_step_s)));
  for (std::size_t index = 0; index <= steps; ++index) {
    const double fraction = static_cast<double>(index) /
      static_cast<double>(steps);
    const double time = segment.start_time_s + fraction * duration;
    const auto projectile = projectile_position(shot, time);
    const auto target = interpolate(
      segment.target_start_ned, segment.target_end_ned, fraction);
    const Vector difference{
      projectile[0] - target[0],
      projectile[1] - target[1],
      projectile[2] - target[2]};
    const double miss = norm(difference);
    const Vector projectile_displacement{
      projectile[0] - shot.origin_ned[0],
      projectile[1] - shot.origin_ned[1],
      projectile[2] - shot.origin_ned[2]};
    const double effective_hit_radius =
      config_.effect_cone_half_angle_rad > 0.0 ?
      norm(projectile_displacement) *
      std::tan(config_.effect_cone_half_angle_rad) :
      config_.hit_radius_m;
    const double clearance = miss - effective_hit_radius;
    if (clearance < output.minimum_clearance_m) {
      output.minimum_miss_distance_m = miss;
      output.effective_hit_radius_m = effective_hit_radius;
      output.minimum_clearance_m = clearance;
      output.closest_time_s = time;
      output.projectile_closest_ned = projectile;
      output.target_closest_ned = target;
    }
  }
  output.hit = output.minimum_clearance_m <= 0.0;
  output.valid = std::isfinite(output.minimum_miss_distance_m) &&
    std::isfinite(output.effective_hit_radius_m) &&
    std::isfinite(output.minimum_clearance_m);
  return output;
}

const RefereeConfig & RefereeModel::config() const
{
  return config_;
}

MissionHitCounter::MissionHitCounter(const MissionHitCounterConfig & config)
: config_(config)
{
  if (config_.minimum_hit_interval_s < 0.0 || config_.required_hits == 0) {
    throw std::invalid_argument("invalid mission hit counter configuration");
  }
}

bool MissionHitCounter::accept(bool hit, double shot_time_s)
{
  if (!hit || !std::isfinite(shot_time_s)) {
    return false;
  }
  if (last_counted_hit_time_s_ >= 0.0 &&
    shot_time_s - last_counted_hit_time_s_ < config_.minimum_hit_interval_s)
  {
    return false;
  }
  ++successful_hit_count_;
  last_counted_hit_time_s_ = shot_time_s;
  return true;
}

std::uint32_t MissionHitCounter::successful_hit_count() const
{
  return successful_hit_count_;
}

bool MissionHitCounter::mission_success() const
{
  return successful_hit_count_ >= config_.required_hits;
}

}  // namespace virtual_interception_referee
