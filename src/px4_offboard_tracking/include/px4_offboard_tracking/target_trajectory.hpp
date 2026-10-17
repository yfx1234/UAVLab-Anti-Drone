#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace px4_offboard_tracking
{

enum class TargetScenario
{
  Stationary,
  Straight,
  Circle,
  SCurve,
  FigureEight,
  Rectangle
};

inline TargetScenario target_scenario_from_string(const std::string & value)
{
  if (value == "stationary") {
    return TargetScenario::Stationary;
  }
  if (value == "straight") {
    return TargetScenario::Straight;
  }
  if (value == "circle") {
    return TargetScenario::Circle;
  }
  if (value == "s_curve") {
    return TargetScenario::SCurve;
  }
  if (value == "figure_eight") {
    return TargetScenario::FigureEight;
  }
  if (value == "rectangle") {
    return TargetScenario::Rectangle;
  }
  throw std::invalid_argument(
          "target_scenario must be stationary, straight, circle, s_curve, "
          "figure_eight, or rectangle");
}

struct TargetTrajectorySetpoint
{
  double x{};
  double y{};
  double z{};
  double vx{};
  double vy{};
  double vz{};
  double ax{};
  double ay{};
  double az{};
  double yaw{};
};

class TargetTrajectory
{
public:
  TargetTrajectory(
    TargetScenario scenario, double height, double radius, double speed, double heading,
    double path_length = 8.0, double path_width = 3.0)
  : scenario_(scenario), height_(height), radius_(radius), speed_(speed), heading_(heading)
    , path_length_(path_length), path_width_(path_width)
  {
    if (height_ < 0.0) {
      throw std::invalid_argument("target height must be non-negative");
    }
    if (radius_ <= 0.0) {
      throw std::invalid_argument("target radius must be positive");
    }
    if (speed_ < 0.0) {
      throw std::invalid_argument("target speed must be non-negative");
    }
    if (path_length_ <= 0.0 || path_width_ <= 0.0) {
      throw std::invalid_argument("target path length and width must be positive");
    }
    if (scenario_ == TargetScenario::SCurve && speed_ > 0.0) {
      const double amplitude = 0.5 * path_width_;
      const double wave_number = 2.0 * M_PI / path_length_;
      const double maximum_tangent_norm =
        std::sqrt(1.0 + std::pow(amplitude * wave_number, 2));
      s_curve_forward_speed_ = speed_ / maximum_tangent_norm;
    }
    if (scenario_ == TargetScenario::FigureEight && speed_ > 0.0) {
      const double amplitude_x = 0.5 * path_length_;
      const double amplitude_y = 0.5 * path_width_;
      double maximum_derivative_norm = 0.0;
      for (int sample_index = 0; sample_index < 4096; ++sample_index) {
        const double phase =
          2.0 * M_PI * static_cast<double>(sample_index) / 4096.0;
        maximum_derivative_norm = std::max(
          maximum_derivative_norm,
          std::hypot(
            amplitude_x * std::sin(phase),
            2.0 * amplitude_y * std::cos(2.0 * phase)));
      }
      s_curve_phase_rate_ = speed_ / std::max(1e-6, maximum_derivative_norm);
    }
  }

  TargetTrajectorySetpoint sample(
    double origin_x, double origin_y, double origin_z, double elapsed_seconds) const
  {
    const double elapsed = std::max(0.0, elapsed_seconds);
    TargetTrajectorySetpoint output;
    output.z = origin_z - height_;  // NED: a greater altitude has a more negative z value.

    if (scenario_ == TargetScenario::Stationary) {
      output.x = origin_x;
      output.y = origin_y;
      output.yaw = heading_;
    } else if (scenario_ == TargetScenario::Straight) {
      output.x = origin_x + speed_ * elapsed * std::cos(heading_);
      output.y = origin_y + speed_ * elapsed * std::sin(heading_);
      output.vx = speed_ * std::cos(heading_);
      output.vy = speed_ * std::sin(heading_);
      output.yaw = heading_;
    } else if (scenario_ == TargetScenario::Circle) {
      const double omega = speed_ / radius_;
      const double phase = omega * elapsed;
      // Start at the current hold position instead of commanding an instantaneous
      // radius-sized position jump when target motion becomes enabled.
      const double local_x = radius_ * (std::cos(phase) - 1.0);
      const double local_y = radius_ * std::sin(phase);
      const double local_vx = -speed_ * std::sin(phase);
      const double local_vy = speed_ * std::cos(phase);
      const double local_ax = -speed_ * omega * std::cos(phase);
      const double local_ay = -speed_ * omega * std::sin(phase);
      const double cosine = std::cos(heading_);
      const double sine = std::sin(heading_);
      output.x = origin_x + cosine * local_x - sine * local_y;
      output.y = origin_y + sine * local_x + cosine * local_y;
      output.vx = cosine * local_vx - sine * local_vy;
      output.vy = sine * local_vx + cosine * local_vy;
      output.ax = cosine * local_ax - sine * local_ay;
      output.ay = sine * local_ax + cosine * local_ay;
      output.yaw = std::atan2(output.vy, output.vx);
    } else if (scenario_ == TargetScenario::SCurve) {
      // A forward-progressing sinusoid is a true S path rather than the old
      // closed figure-eight. Scale the along-track speed so total speed never
      // exceeds the configured value.
      const double amplitude = 0.5 * path_width_;
      const double wave_number = 2.0 * M_PI / path_length_;
      const double local_x = s_curve_forward_speed_ * elapsed;
      const double phase = wave_number * local_x;
      const double local_y = amplitude * std::sin(phase);
      const double local_vx = s_curve_forward_speed_;
      const double local_vy =
        amplitude * wave_number * s_curve_forward_speed_ * std::cos(phase);
      const double local_ay =
        -amplitude * wave_number * wave_number *
        s_curve_forward_speed_ * s_curve_forward_speed_ * std::sin(phase);
      const double cosine = std::cos(heading_);
      const double sine = std::sin(heading_);
      output.x = origin_x + cosine * local_x - sine * local_y;
      output.y = origin_y + sine * local_x + cosine * local_y;
      output.vx = cosine * local_vx - sine * local_vy;
      output.vy = sine * local_vx + cosine * local_vy;
      output.ax = -sine * local_ay;
      output.ay = cosine * local_ay;
      output.yaw = std::atan2(output.vy, output.vx);
    } else if (scenario_ == TargetScenario::FigureEight) {
      // Closed figure-eight retained as a separate maneuvering scenario.
      const double amplitude_x = 0.5 * path_length_;
      const double amplitude = 0.5 * path_width_;
      const double phase_rate = s_curve_phase_rate_;
      const double phase = phase_rate * elapsed;
      // The offset makes elapsed=0 coincide with the preceding hold setpoint.
      const double local_x = amplitude_x * (std::cos(phase) - 1.0);
      const double local_y = amplitude * std::sin(2.0 * phase);
      const double dx_dphase = -amplitude_x * std::sin(phase);
      const double dy_dphase = 2.0 * amplitude * std::cos(2.0 * phase);
      const double d2x_dphase2 = -amplitude_x * std::cos(phase);
      const double d2y_dphase2 = -4.0 * amplitude * std::sin(2.0 * phase);
      const double cosine = std::cos(heading_);
      const double sine = std::sin(heading_);
      output.x = origin_x + cosine * local_x - sine * local_y;
      output.y = origin_y + sine * local_x + cosine * local_y;
      const double velocity_x = cosine * dx_dphase - sine * dy_dphase;
      const double velocity_y = sine * dx_dphase + cosine * dy_dphase;
      output.vx = phase_rate * velocity_x;
      output.vy = phase_rate * velocity_y;
      output.ax = phase_rate * phase_rate *
        (cosine * d2x_dphase2 - sine * d2y_dphase2);
      output.ay = phase_rate * phase_rate *
        (sine * d2x_dphase2 + cosine * d2y_dphase2);
      output.yaw = std::atan2(velocity_y, velocity_x);
    } else {
      // Rounded corners keep position and velocity continuous. Speed is constant along the path.
      const double corner_radius = 0.25 * std::min(path_length_, path_width_);
      const double straight_x = path_length_ - 2.0 * corner_radius;
      const double straight_y = path_width_ - 2.0 * corner_radius;
      const double corner_length = 0.5 * M_PI * corner_radius;
      const double perimeter = 2.0 * (straight_x + straight_y) + 4.0 * corner_length;
      const double distance = speed_ > 0.0 ? std::fmod(speed_ * elapsed, perimeter) : 0.0;
      double local_x{};
      double local_y{};
      double tangent_x{};
      double tangent_y{};
      double acceleration_x{};
      double acceleration_y{};
      double remaining = distance;
      if (remaining < straight_x) {
        local_x = 0.5 * path_length_ - corner_radius - remaining;
        local_y = -0.5 * path_width_;
        tangent_x = -1.0;
      } else if ((remaining -= straight_x) < corner_length) {
        const double angle = -M_PI_2 - remaining / corner_radius;
        local_x = -0.5 * path_length_ + corner_radius + corner_radius * std::cos(angle);
        local_y = -0.5 * path_width_ + corner_radius + corner_radius * std::sin(angle);
        tangent_x = std::sin(angle);
        tangent_y = -std::cos(angle);
        acceleration_x = -speed_ * speed_ / corner_radius * std::cos(angle);
        acceleration_y = -speed_ * speed_ / corner_radius * std::sin(angle);
      } else if ((remaining -= corner_length) < straight_y) {
        local_x = -0.5 * path_length_;
        local_y = -0.5 * path_width_ + corner_radius + remaining;
        tangent_y = 1.0;
      } else if ((remaining -= straight_y) < corner_length) {
        const double angle = -M_PI - remaining / corner_radius;
        local_x = -0.5 * path_length_ + corner_radius + corner_radius * std::cos(angle);
        local_y = 0.5 * path_width_ - corner_radius + corner_radius * std::sin(angle);
        tangent_x = std::sin(angle);
        tangent_y = -std::cos(angle);
        acceleration_x = -speed_ * speed_ / corner_radius * std::cos(angle);
        acceleration_y = -speed_ * speed_ / corner_radius * std::sin(angle);
      } else if ((remaining -= corner_length) < straight_x) {
        local_x = -0.5 * path_length_ + corner_radius + remaining;
        local_y = 0.5 * path_width_;
        tangent_x = 1.0;
      } else if ((remaining -= straight_x) < corner_length) {
        const double angle = -1.5 * M_PI - remaining / corner_radius;
        local_x = 0.5 * path_length_ - corner_radius + corner_radius * std::cos(angle);
        local_y = 0.5 * path_width_ - corner_radius + corner_radius * std::sin(angle);
        tangent_x = std::sin(angle);
        tangent_y = -std::cos(angle);
        acceleration_x = -speed_ * speed_ / corner_radius * std::cos(angle);
        acceleration_y = -speed_ * speed_ / corner_radius * std::sin(angle);
      } else if ((remaining -= corner_length) < straight_y) {
        local_x = 0.5 * path_length_;
        local_y = 0.5 * path_width_ - corner_radius - remaining;
        tangent_y = -1.0;
      } else {
        remaining -= straight_y;
        const double angle = -2.0 * M_PI - remaining / corner_radius;
        local_x = 0.5 * path_length_ - corner_radius + corner_radius * std::cos(angle);
        local_y = -0.5 * path_width_ + corner_radius + corner_radius * std::sin(angle);
        tangent_x = std::sin(angle);
        tangent_y = -std::cos(angle);
        acceleration_x = -speed_ * speed_ / corner_radius * std::cos(angle);
        acceleration_y = -speed_ * speed_ / corner_radius * std::sin(angle);
      }
      // Like every other scenario, make the supplied origin the actual point
      // occupied at elapsed=0 rather than the geometric rectangle centre.
      local_x -= 0.5 * path_length_ - corner_radius;
      local_y += 0.5 * path_width_;
      const double cosine = std::cos(heading_);
      const double sine = std::sin(heading_);
      output.x = origin_x + cosine * local_x - sine * local_y;
      output.y = origin_y + sine * local_x + cosine * local_y;
      output.vx = speed_ * (cosine * tangent_x - sine * tangent_y);
      output.vy = speed_ * (sine * tangent_x + cosine * tangent_y);
      output.ax = cosine * acceleration_x - sine * acceleration_y;
      output.ay = sine * acceleration_x + cosine * acceleration_y;
      output.yaw = std::atan2(
        sine * tangent_x + cosine * tangent_y,
        cosine * tangent_x - sine * tangent_y);
    }
    return output;
  }

  TargetTrajectorySetpoint sample_with_speed_ramp(
    double origin_x, double origin_y, double origin_z, double elapsed_seconds,
    double ramp_duration_s) const
  {
    const double elapsed = std::max(0.0, elapsed_seconds);
    if (ramp_duration_s <= 0.0 || elapsed >= ramp_duration_s) {
      const double trajectory_elapsed =
        ramp_duration_s > 0.0 ? elapsed - 0.5 * ramp_duration_s : elapsed;
      return sample(origin_x, origin_y, origin_z, trajectory_elapsed);
    }

    const double time_scale = elapsed / ramp_duration_s;
    const double time_scale_rate = 1.0 / ramp_duration_s;
    const double trajectory_elapsed =
      0.5 * elapsed * elapsed / ramp_duration_s;
    auto output = sample(origin_x, origin_y, origin_z, trajectory_elapsed);
    const double base_vx = output.vx;
    const double base_vy = output.vy;
    const double base_vz = output.vz;
    output.vx *= time_scale;
    output.vy *= time_scale;
    output.vz *= time_scale;
    output.ax = output.ax * time_scale * time_scale + base_vx * time_scale_rate;
    output.ay = output.ay * time_scale * time_scale + base_vy * time_scale_rate;
    output.az = output.az * time_scale * time_scale + base_vz * time_scale_rate;
    return output;
  }

private:
  TargetScenario scenario_;
  double height_;
  double radius_;
  double speed_;
  double heading_;
  double path_length_;
  double path_width_;
  double s_curve_forward_speed_{};
  double s_curve_phase_rate_{};
};

}  // namespace px4_offboard_tracking
