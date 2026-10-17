#pragma once

#include <algorithm>
#include <cmath>

namespace px4_offboard_tracking
{

struct State3d
{
  double x{};
  double y{};
  double z{};
  double vx{};
  double vy{};
  double vz{};
};

struct VelocityCommand
{
  double vx{};
  double vy{};
  double vz{};
  double yaw{};
};

class TrackingController
{
public:
  TrackingController(double position_gain, double velocity_feedforward,
    double max_horizontal_speed, double max_vertical_speed, double standoff_distance)
  : kp_(position_gain), kv_(velocity_feedforward), max_xy_(max_horizontal_speed),
    max_z_(max_vertical_speed), standoff_(standoff_distance) {}

  VelocityCommand update(const State3d & pursuer, const State3d & target) const
  {
    const double dx = target.x - pursuer.x;
    const double dy = target.y - pursuer.y;
    const double range = std::hypot(dx, dy);
    const double closing_error = std::max(0.0, range - standoff_);
    const double ux = range > 1e-6 ? dx / range : 0.0;
    const double uy = range > 1e-6 ? dy / range : 0.0;

    VelocityCommand command;
    command.vx = kp_ * closing_error * ux + kv_ * target.vx;
    command.vy = kp_ * closing_error * uy + kv_ * target.vy;
    const double speed = std::hypot(command.vx, command.vy);
    if (speed > max_xy_) {
      command.vx *= max_xy_ / speed;
      command.vy *= max_xy_ / speed;
    }
    command.vz = std::clamp(kp_ * (target.z - pursuer.z) + kv_ * target.vz, -max_z_, max_z_);
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
