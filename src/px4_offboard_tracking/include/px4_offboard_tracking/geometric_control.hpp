#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "px4_offboard_tracking/acceleration_guidance.hpp"

namespace px4_offboard_tracking
{

struct Quaternion
{
  double w{1.0};
  double x{};
  double y{};
  double z{};
};

struct AttitudeThrustCommand
{
  Quaternion attitude{};
  double roll{};
  double pitch{};
  double yaw{};
  double thrust{};  // Positive collective magnitude; PX4 FRD z command is -thrust.
};

struct BodyRateThrustCommand
{
  double roll_rate{};
  double pitch_rate{};
  double yaw_rate{};
  double thrust{};
};

struct ThrustPriorityResult
{
  AccelerationCommand acceleration{};
  double requested_normalized_thrust{};
  bool saturated{false};
};

class GeometricControl
{
public:
  GeometricControl(
    double hover_thrust, double max_thrust, double max_tilt_rad, double attitude_gain,
    double max_body_rate)
  : hover_thrust_(hover_thrust), max_thrust_(max_thrust), max_tilt_(max_tilt_rad),
    attitude_gain_(attitude_gain), max_body_rate_(max_body_rate) {}

  // When collective authority is insufficient, preserve the requested
  // vertical specific force first and reduce horizontal acceleration/tilt.
  // This keeps the minimum-height recovery effective instead of spending the
  // remaining thrust margin on lateral interception.
  ThrustPriorityResult prioritize_vertical_acceleration(
    const AccelerationCommand & input) const
  {
    constexpr double gravity = 9.80665;
    ThrustPriorityResult output;
    output.acceleration = input;
    const double vertical_specific_force = std::max(0.1, gravity - input.az);
    const double horizontal_specific_force = std::hypot(input.ax, input.ay);
    const double total_specific_force = std::hypot(
      horizontal_specific_force, vertical_specific_force);
    output.requested_normalized_thrust =
      hover_thrust_ * total_specific_force / gravity;
    if (output.requested_normalized_thrust <= max_thrust_) {
      return output;
    }

    output.saturated = true;
    const double maximum_specific_force =
      max_thrust_ * gravity / std::max(1.0e-6, hover_thrust_);
    const double maximum_horizontal = std::sqrt(std::max(
      0.0,
      maximum_specific_force * maximum_specific_force -
      vertical_specific_force * vertical_specific_force));
    if (horizontal_specific_force > maximum_horizontal &&
      horizontal_specific_force > 1.0e-9)
    {
      const double scale = maximum_horizontal / horizontal_specific_force;
      output.acceleration.ax *= scale;
      output.acceleration.ay *= scale;
    }
    return output;
  }

  AttitudeThrustCommand attitude_from_acceleration(const AccelerationCommand & input) const
  {
    constexpr double gravity = 9.80665;
    double ax = input.ax;
    double ay = input.ay;
    const double horizontal = std::hypot(ax, ay);
    const double vertical_force = std::max(0.1, gravity - input.az);
    const double max_horizontal = vertical_force * std::tan(max_tilt_);
    if (horizontal > max_horizontal) {
      ax *= max_horizontal / horizontal;
      ay *= max_horizontal / horizontal;
    }

    // Desired body-z axis in NED. Body thrust acts along negative FRD z.
    auto b3 = normalize({-ax, -ay, vertical_force});
    const std::array<double, 3> heading{std::cos(input.yaw), std::sin(input.yaw), 0.0};
    auto b2 = normalize(cross(b3, heading));
    if (norm(b2) < 1e-6) {b2 = {0.0, 1.0, 0.0};}
    const auto b1 = cross(b2, b3);

    AttitudeThrustCommand output;
    output.attitude = quaternion_from_rotation(b1, b2, b3);
    euler_from_quaternion(output.attitude, output.roll, output.pitch, output.yaw);
    output.thrust = std::clamp(
      hover_thrust_ * std::sqrt(ax * ax + ay * ay + vertical_force * vertical_force) / gravity,
      0.0, max_thrust_);
    return output;
  }

  // Preserve the requested vertical acceleration while the real attitude is
  // still converging to the commanded attitude. Using the desired tilt for
  // collective feed-forward creates a lift pulse during tilt-in because the
  // vehicle is temporarily flatter than the command.
  double transient_compensated_thrust(
    double acceleration_down_mps2, double current_attack_axis_up_component) const
  {
    return std::clamp(
      transient_compensated_thrust_request(
        acceleration_down_mps2, current_attack_axis_up_component),
      0.0, max_thrust_);
  }

  double transient_compensated_thrust_request(
    double acceleration_down_mps2, double current_attack_axis_up_component) const
  {
    constexpr double gravity = 9.80665;
    const double usable_up_component =
      std::max(0.35, current_attack_axis_up_component);
    const double vertical_specific_force =
      std::max(0.1, gravity - acceleration_down_mps2);
    return hover_thrust_ * vertical_specific_force /
           (gravity * usable_up_component);
  }

  double hover_thrust() const {return hover_thrust_;}
  double maximum_thrust() const {return max_thrust_;}

  BodyRateThrustCommand body_rate_from_attitude(
    const Quaternion & current, const AttitudeThrustCommand & desired) const
  {
    const Quaternion current_normalized = normalize_quaternion(current);
    const Quaternion desired_normalized = normalize_quaternion(desired.attitude);
    const Quaternion error = multiply(conjugate(current_normalized), desired_normalized);
    const double sign = error.w >= 0.0 ? 1.0 : -1.0;
    return {
      std::clamp(2.0 * attitude_gain_ * sign * error.x, -max_body_rate_, max_body_rate_),
      std::clamp(2.0 * attitude_gain_ * sign * error.y, -max_body_rate_, max_body_rate_),
      std::clamp(2.0 * attitude_gain_ * sign * error.z, -max_body_rate_, max_body_rate_),
      desired.thrust};
  }

private:
  static double norm(const std::array<double, 3> & v)
  {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  static std::array<double, 3> normalize(const std::array<double, 3> & v)
  {
    const double length = norm(v);
    if (length < 1e-9) {return {0.0, 0.0, 0.0};}
    return {v[0] / length, v[1] / length, v[2] / length};
  }

  static std::array<double, 3> cross(
    const std::array<double, 3> & a, const std::array<double, 3> & b)
  {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]};
  }

  static Quaternion normalize_quaternion(const Quaternion & q)
  {
    const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (length < 1e-9) {return {};}
    return {q.w / length, q.x / length, q.y / length, q.z / length};
  }

  static Quaternion conjugate(const Quaternion & q) {return {q.w, -q.x, -q.y, -q.z};}

  static Quaternion multiply(const Quaternion & a, const Quaternion & b)
  {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
  }

  static Quaternion quaternion_from_rotation(
    const std::array<double, 3> & b1, const std::array<double, 3> & b2,
    const std::array<double, 3> & b3)
  {
    const double r00 = b1[0], r01 = b2[0], r02 = b3[0];
    const double r10 = b1[1], r11 = b2[1], r12 = b3[1];
    const double r20 = b1[2], r21 = b2[2], r22 = b3[2];
    Quaternion q;
    const double trace = r00 + r11 + r22;
    if (trace > 0.0) {
      const double s = 2.0 * std::sqrt(trace + 1.0);
      q = {0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s};
    } else if (r00 > r11 && r00 > r22) {
      const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
      q = {(r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s};
    } else if (r11 > r22) {
      const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
      q = {(r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s};
    } else {
      const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
      q = {(r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s};
    }
    return normalize_quaternion(q);
  }

  static void euler_from_quaternion(
    const Quaternion & q, double & roll, double & pitch, double & yaw)
  {
    roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  double hover_thrust_;
  double max_thrust_;
  double max_tilt_;
  double attitude_gain_;
  double max_body_rate_;
};

}  // namespace px4_offboard_tracking
