#pragma once

#include <algorithm>
#include <cmath>

#include "px4_offboard_tracking/tracking_controller.hpp"

namespace px4_offboard_tracking
{

struct AccelerationCommand
{
  double ax{};
  double ay{};
  double az{};
  double yaw{};
};

class AccelerationGuidance
{
public:
  AccelerationGuidance(
    double position_gain, double velocity_gain, double max_horizontal_acceleration,
    double max_vertical_acceleration, double standoff_distance)
  : kp_(position_gain), kv_(velocity_gain), max_xy_(max_horizontal_acceleration),
    max_z_(max_vertical_acceleration), standoff_(standoff_distance) {}

  AccelerationCommand update(const State3d & pursuer, const State3d & target) const
  {
    const double dx = target.x - pursuer.x;
    const double dy = target.y - pursuer.y;
    const double range = std::hypot(dx, dy);
    const double ux = range > 1e-6 ? dx / range : 0.0;
    const double uy = range > 1e-6 ? dy / range : 0.0;
    const double closing_error = std::max(0.0, range - standoff_);

    AccelerationCommand command;
    command.ax = kp_ * closing_error * ux + kv_ * (target.vx - pursuer.vx);
    command.ay = kp_ * closing_error * uy + kv_ * (target.vy - pursuer.vy);
    const double horizontal = std::hypot(command.ax, command.ay);
    if (horizontal > max_xy_) {
      command.ax *= max_xy_ / horizontal;
      command.ay *= max_xy_ / horizontal;
    }
    command.az = std::clamp(
      kp_ * (target.z - pursuer.z) + kv_ * (target.vz - pursuer.vz), -max_z_, max_z_);
    command.yaw = range > 1e-6 ? std::atan2(dy, dx) : 0.0;
    return command;
  }

private:
  double kp_;
  double kv_;
  double max_xy_;
  double max_z_;
  double standoff_;
};

}  // namespace px4_offboard_tracking
