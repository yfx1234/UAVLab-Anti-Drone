#include "px4_offboard_tracking/attitude_setpoint_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

double vector_norm(const std::array<double, 3> & vector)
{
  return std::sqrt(
    vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

std::array<double, 3> normalize_vector(const std::array<double, 3> & vector)
{
  const double length = vector_norm(vector);
  if (length < 1e-9) {return {};}
  return {vector[0] / length, vector[1] / length, vector[2] / length};
}

std::array<double, 3> cross(
  const std::array<double, 3> & first, const std::array<double, 3> & second)
{
  return {first[1] * second[2] - first[2] * second[1],
    first[2] * second[0] - first[0] * second[2],
    first[0] * second[1] - first[1] * second[0]};
}

Quaternion normalize_quaternion(const Quaternion & quaternion)
{
  const double length = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(length) || length < 1e-9) {return {};}
  return {quaternion.w / length, quaternion.x / length,
    quaternion.y / length, quaternion.z / length};
}

double dot(const Quaternion & first, const Quaternion & second)
{
  return first.w * second.w + first.x * second.x +
         first.y * second.y + first.z * second.z;
}

Quaternion negate(const Quaternion & quaternion)
{
  return {-quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

Quaternion quaternion_from_rotation(
  const std::array<double, 3> & b1, const std::array<double, 3> & b2,
  const std::array<double, 3> & b3)
{
  const double r00 = b1[0], r01 = b2[0], r02 = b3[0];
  const double r10 = b1[1], r11 = b2[1], r12 = b3[1];
  const double r20 = b1[2], r21 = b2[2], r22 = b3[2];
  Quaternion quaternion;
  const double trace = r00 + r11 + r22;
  if (trace > 0.0) {
    const double s = 2.0 * std::sqrt(trace + 1.0);
    quaternion = {0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s};
  } else if (r00 > r11 && r00 > r22) {
    const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
    quaternion = {(r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s};
  } else if (r11 > r22) {
    const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
    quaternion = {(r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s};
  } else {
    const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
    quaternion = {(r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s};
  }
  return normalize_quaternion(quaternion);
}

Quaternion slerp(const Quaternion & start_input, const Quaternion & end_input, double fraction)
{
  const Quaternion start = normalize_quaternion(start_input);
  Quaternion end = normalize_quaternion(end_input);
  double cosine = dot(start, end);
  if (cosine < 0.0) {
    end = negate(end);
    cosine = -cosine;
  }
  cosine = std::clamp(cosine, -1.0, 1.0);
  if (cosine > 0.9995) {
    return normalize_quaternion({
      start.w + fraction * (end.w - start.w),
      start.x + fraction * (end.x - start.x),
      start.y + fraction * (end.y - start.y),
      start.z + fraction * (end.z - start.z)});
  }
  const double angle = std::acos(cosine);
  const double sine = std::sin(angle);
  const double start_weight = std::sin((1.0 - fraction) * angle) / sine;
  const double end_weight = std::sin(fraction * angle) / sine;
  return normalize_quaternion({
    start_weight * start.w + end_weight * end.w,
    start_weight * start.x + end_weight * end.x,
    start_weight * start.y + end_weight * end.y,
    start_weight * start.z + end_weight * end.z});
}

void euler_from_quaternion(
  const Quaternion & quaternion, double & roll, double & pitch, double & yaw)
{
  roll = std::atan2(
    2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
    1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y));
  pitch = std::asin(std::clamp(
    2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x), -1.0, 1.0));
  yaw = std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace

AttitudeSetpointGenerator::AttitudeSetpointGenerator(
  const AttitudeSetpointGeneratorConfig & config)
: config_(config), previous_thrust_(config.hover_thrust)
{
  if (config_.hover_thrust <= 0.0 || config_.minimum_thrust < 0.0 ||
    config_.maximum_thrust <= config_.minimum_thrust ||
    config_.smoothing_time_constant_s <= 0.0 ||
    config_.maximum_attitude_slew_rate_rad_s <= 0.0)
  {
    throw std::invalid_argument("invalid attitude setpoint generator configuration");
  }
}

void AttitudeSetpointGenerator::reset()
{
  initialized_ = false;
  previous_attitude_ = {};
  previous_thrust_ = config_.hover_thrust;
}

AttitudeThrustCommand AttitudeSetpointGenerator::update(
  const std::array<double, 3> & desired_camera_axis_ned_input,
  double desired_yaw_rad, double desired_vertical_acceleration_mps2,
  const Quaternion & current_body_to_ned, double delta_time_s)
{
  constexpr double gravity = 9.80665;
  if (!std::isfinite(delta_time_s) || delta_time_s <= 0.0 ||
    !std::isfinite(desired_yaw_rad) || !std::isfinite(desired_vertical_acceleration_mps2))
  {
    throw std::invalid_argument("invalid attitude setpoint input timing/value");
  }
  const auto camera_axis = normalize_vector(desired_camera_axis_ned_input);
  if (camera_axis[2] >= -1e-3) {
    throw std::invalid_argument("desired camera axis must point upward in NED");
  }

  const std::array<double, 3> b3{-camera_axis[0], -camera_axis[1], -camera_axis[2]};
  const std::array<double, 3> heading{
    std::cos(desired_yaw_rad), std::sin(desired_yaw_rad), 0.0};
  auto b2 = normalize_vector(cross(b3, heading));
  if (vector_norm(b2) < 1e-6) {b2 = {0.0, 1.0, 0.0};}
  const auto b1 = normalize_vector(cross(b2, b3));
  Quaternion raw_attitude = quaternion_from_rotation(b1, b2, b3);

  if (!initialized_) {
    previous_attitude_ = normalize_quaternion(current_body_to_ned);
    previous_thrust_ = config_.hover_thrust;
    initialized_ = true;
  }
  if (dot(previous_attitude_, raw_attitude) < 0.0) {raw_attitude = negate(raw_attitude);}
  const double quaternion_dot = std::clamp(
    std::abs(dot(normalize_quaternion(previous_attitude_), raw_attitude)), 0.0, 1.0);
  const double angular_distance = 2.0 * std::acos(quaternion_dot);
  double fraction = 1.0 - std::exp(-delta_time_s / config_.smoothing_time_constant_s);
  if (angular_distance > 1e-9) {
    fraction = std::min(
      fraction, config_.maximum_attitude_slew_rate_rad_s * delta_time_s / angular_distance);
  }
  fraction = std::clamp(fraction, 0.0, 1.0);
  previous_attitude_ = slerp(previous_attitude_, raw_attitude, fraction);

  const double required_specific_thrust =
    std::max(0.0, (gravity - desired_vertical_acceleration_mps2) / (-camera_axis[2]));
  const double raw_thrust = std::clamp(
    config_.hover_thrust * required_specific_thrust / gravity,
    config_.minimum_thrust, config_.maximum_thrust);
  previous_thrust_ += fraction * (raw_thrust - previous_thrust_);

  AttitudeThrustCommand output;
  output.attitude = previous_attitude_;
  output.thrust = previous_thrust_;
  euler_from_quaternion(output.attitude, output.roll, output.pitch, output.yaw);
  return output;
}

}  // namespace px4_offboard_tracking

