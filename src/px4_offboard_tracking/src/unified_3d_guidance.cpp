#include "px4_offboard_tracking/unified_3d_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

using Vector = std::array<double, 3>;

double dot(const Vector & a, const Vector & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector cross(const Vector & a, const Vector & b)
{
  return {
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]};
}

double norm(const Vector & value)
{
  return std::sqrt(dot(value, value));
}

Vector normalized(const Vector & value)
{
  const double length = norm(value);
  if (!std::isfinite(length) || length < 1e-9) {
    return {};
  }
  return {value[0] / length, value[1] / length, value[2] / length};
}

Vector scale(const Vector & value, double factor)
{
  return {factor * value[0], factor * value[1], factor * value[2]};
}

Vector add(const Vector & a, const Vector & b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vector subtract(const Vector & a, const Vector & b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vector limit_norm(const Vector & value, double maximum)
{
  const double length = norm(value);
  if (length <= maximum || length < 1e-9) {
    return value;
  }
  return scale(value, maximum / length);
}

double angle_between(const Vector & a, const Vector & b)
{
  return std::acos(std::clamp(dot(normalized(a), normalized(b)), -1.0, 1.0));
}

// Spherical interpolation for unit vectors. This preserves a continuous vector
// direction at the field-of-view boundary and avoids the old per-axis clipping.
Vector slerp_unit(const Vector & from, const Vector & to, double fraction)
{
  const Vector start = normalized(from);
  const Vector end = normalized(to);
  const double cosine = std::clamp(dot(start, end), -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (angle < 1e-8) {
    return start;
  }
  const double sine = std::sin(angle);
  if (std::abs(sine) < 1e-8) {
    return normalized(add(scale(start, 1.0 - fraction), scale(end, fraction)));
  }
  return normalized(add(
    scale(start, std::sin((1.0 - fraction) * angle) / sine),
    scale(end, std::sin(fraction * angle) / sine)));
}

bool finite_vector(const Vector & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

}  // namespace

Unified3dGuidance::Unified3dGuidance(const Unified3dGuidanceConfig & config)
: config_(config)
{
  if (config_.navigation_constant <= 0.0 || config_.radial_acceleration_mps2 < 0.0 ||
    config_.radial_velocity_gain < 0.0 || config_.desired_closing_speed_mps < 0.0 ||
    config_.target_acceleration_feedforward_gain < 0.0 ||
    config_.maximum_acceleration_mps2 <= 0.0 || config_.maximum_tilt_rad <= 0.0 ||
    config_.maximum_tilt_rad >= 1.4 || config_.depth_fov_constraint_rad <= 0.0 ||
    config_.terminal_alignment_start_range_m <= config_.capture_alignment_range_m ||
    config_.capture_alignment_range_m <= 0.0 || config_.capture_alignment_rad <= 0.0 ||
    config_.capture_alignment_rad > config_.depth_fov_constraint_rad ||
    config_.transverse_velocity_gain < 0.0)
  {
    throw std::invalid_argument("invalid unified 3D guidance configuration");
  }
}

Unified3dGuidanceResult Unified3dGuidance::update(
  const Unified3dGuidanceInput & input) const
{
  constexpr double gravity = 9.80665;
  constexpr Vector gravity_ned{0.0, 0.0, gravity};
  Unified3dGuidanceResult output;
  if (!finite_vector(input.relative_position_ned) ||
    !finite_vector(input.relative_velocity_ned) ||
    !finite_vector(input.target_acceleration_ned) ||
    !std::isfinite(input.target_acceleration_confidence) ||
    input.target_acceleration_confidence < 0.0 ||
    input.target_acceleration_confidence > 1.0 ||
    !std::isfinite(input.terminal_alignment_strength) ||
    input.terminal_alignment_strength < 0.0 ||
    input.terminal_alignment_strength > 1.0 ||
    !std::isfinite(input.launch_planner_blend) ||
    input.launch_planner_blend < 0.0 || input.launch_planner_blend > 1.0 ||
    (input.launch_planner_blend > 0.0 &&
    !finite_vector(input.planned_acceleration_ned)))
  {
    return output;
  }

  output.range_m = norm(input.relative_position_ned);
  if (output.range_m < 1e-3) {
    return output;
  }
  output.line_of_sight_ned = scale(input.relative_position_ned, 1.0 / output.range_m);
  output.closing_speed_mps = -dot(input.relative_velocity_ned, output.line_of_sight_ned);

  const Vector los_rate_vector = scale(
    cross(input.relative_position_ned, input.relative_velocity_ned),
    1.0 / (output.range_m * output.range_m));
  output.line_of_sight_rate_rad_s = norm(los_rate_vector);
  const Vector los_derivative = cross(los_rate_vector, output.line_of_sight_ned);
  output.proportional_navigation_acceleration_ned = scale(
    los_derivative,
    config_.navigation_constant * std::max(0.0, output.closing_speed_mps));
  const Vector transverse_relative_velocity = subtract(
    input.relative_velocity_ned,
    scale(
      output.line_of_sight_ned,
      dot(input.relative_velocity_ned, output.line_of_sight_ned)));
  output.transverse_velocity_acceleration_ned = scale(
    transverse_relative_velocity, config_.transverse_velocity_gain);

  // A signed radial speed loop establishes closure and also brakes an
  // overshoot. The previous positive bias could only accelerate toward the
  // target, so closing speed kept growing until a safety abort.
  const double desired_closing_speed =
    std::isfinite(input.desired_closing_speed_mps) ?
    input.desired_closing_speed_mps : config_.desired_closing_speed_mps;
  const double closing_error = desired_closing_speed - output.closing_speed_mps;
  const double radial_magnitude = std::clamp(
    config_.radial_velocity_gain * closing_error,
    -config_.radial_acceleration_mps2,
    config_.radial_acceleration_mps2);
  Vector radial = scale(output.line_of_sight_ned, radial_magnitude);
  output.launch_planner_blend = input.launch_planner_blend;
  if (input.launch_planner_blend > 0.0) {
    const double planned_radial_acceleration =
      dot(input.planned_acceleration_ned, output.line_of_sight_ned);
    radial = scale(output.line_of_sight_ned, planned_radial_acceleration);
    output.selected_radial_acceleration_mps2 = planned_radial_acceleration;
    output.planned_radial_control_active = true;
  } else {
    output.selected_radial_acceleration_mps2 = radial_magnitude;
  }
  const Vector target_acceleration_transverse = subtract(
    input.target_acceleration_ned,
    scale(output.line_of_sight_ned,
    dot(input.target_acceleration_ned, output.line_of_sight_ned)));
  output.target_acceleration_feedforward_ned = scale(
    target_acceleration_transverse,
    config_.target_acceleration_feedforward_gain *
    input.target_acceleration_confidence);
  Vector requested_acceleration = add(
    add(
      add(radial, output.proportional_navigation_acceleration_ned),
      output.transverse_velocity_acceleration_ned),
    output.target_acceleration_feedforward_ned);
  requested_acceleration = limit_norm(
    requested_acceleration, config_.maximum_acceleration_mps2);

  // For a multicopter in NED:
  //   acceleration = gravity_D + specific_thrust * body_minus_Z
  // Therefore the attack/camera/thrust axis is normalize(acceleration - gravity_D).
  const Vector requested_specific_force = subtract(requested_acceleration, gravity_ned);
  const double specific_force_magnitude = norm(requested_specific_force);
  if (specific_force_magnitude < 1e-3) {
    return output;
  }
  output.free_attack_axis_ned = scale(
    requested_specific_force, 1.0 / specific_force_magnitude);

  output.allowed_axis_error_rad = config_.depth_fov_constraint_rad;
  if (std::isfinite(input.maximum_axis_error_rad)) {
    output.allowed_axis_error_rad = std::clamp(
      input.maximum_axis_error_rad,
      config_.capture_alignment_rad,
      config_.depth_fov_constraint_rad);
  }
  if (input.terminal_alignment_required) {
    const double range_fraction = std::clamp(
      (output.range_m - config_.capture_alignment_range_m) /
      (config_.terminal_alignment_start_range_m - config_.capture_alignment_range_m),
      0.0, 1.0);
    const double terminal_axis_error_rad =
      config_.capture_alignment_rad +
      range_fraction * (output.allowed_axis_error_rad - config_.capture_alignment_rad);
    output.allowed_axis_error_rad += input.terminal_alignment_strength *
      (terminal_axis_error_rad - output.allowed_axis_error_rad);
  }

  const Vector vertical_up{0.0, 0.0, -1.0};
  const double los_tilt = angle_between(vertical_up, output.line_of_sight_ned);
  if (!std::isfinite(los_tilt) ||
    los_tilt > config_.maximum_tilt_rad + output.allowed_axis_error_rad)
  {
    // The target LOS and the allowed vehicle tilt cones do not intersect.
    return output;
  }

  const double free_axis_error = angle_between(
    output.line_of_sight_ned, output.free_attack_axis_ned);
  output.constrained_attack_axis_ned = output.free_attack_axis_ned;
  if (free_axis_error > output.allowed_axis_error_rad) {
    output.constrained_attack_axis_ned = slerp_unit(
      output.line_of_sight_ned, output.free_attack_axis_ned,
      output.allowed_axis_error_rad / free_axis_error);
    output.field_of_view_constrained = true;
  }

  const double constrained_tilt = angle_between(
    vertical_up, output.constrained_attack_axis_ned);
  if (constrained_tilt > config_.maximum_tilt_rad) {
    output.constrained_attack_axis_ned = slerp_unit(
      vertical_up, output.constrained_attack_axis_ned,
      config_.maximum_tilt_rad / constrained_tilt);
    output.field_of_view_constrained = true;
  }

  output.attack_axis_error_rad = angle_between(
    output.line_of_sight_ned, output.constrained_attack_axis_ned);
  const Vector constrained_acceleration = add(
    gravity_ned, scale(output.constrained_attack_axis_ned, specific_force_magnitude));
  output.acceleration.ax = constrained_acceleration[0];
  output.acceleration.ay = constrained_acceleration[1];
  output.acceleration.az = constrained_acceleration[2];
  const double horizontal_los = std::hypot(
    output.line_of_sight_ned[0], output.line_of_sight_ned[1]);
  output.acceleration.yaw = horizontal_los > 1e-6 ?
    std::atan2(output.line_of_sight_ned[1], output.line_of_sight_ned[0]) : 0.0;
  output.valid = finite_vector(constrained_acceleration) &&
    std::isfinite(output.attack_axis_error_rad);
  return output;
}

}  // namespace px4_offboard_tracking
