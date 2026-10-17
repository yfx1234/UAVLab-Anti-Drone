// Copyright 2026 danyu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinPublishRateHz = 2.1;  // PX4 Offboard requires > 2 Hz setpoints.
constexpr double kSmall = 1e-6;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Vec3 operator+(const Vec3 & lhs, const Vec3 & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3 & lhs, const Vec3 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(const Vec3 & lhs, double scale)
{
  return {lhs.x * scale, lhs.y * scale, lhs.z * scale};
}

double dot3(const Vec3 & lhs, const Vec3 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double norm2(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

double norm3(const Vec3 & value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

bool finite3(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

Vec3 normalized_xy(const Vec3 & value, const Vec3 & fallback = {1.0, 0.0, 0.0})
{
  const double length = norm2(value.x, value.y);
  if (!std::isfinite(length) || length <= kSmall) {
    return fallback;
  }
  return {value.x / length, value.y / length, 0.0};
}

double deg_to_rad(double deg)
{
  return deg * kPi / 180.0;
}

double rad_to_deg(double rad)
{
  return rad * 180.0 / kPi;
}

double wrap_pi(double yaw)
{
  while (yaw > kPi) {
    yaw -= 2.0 * kPi;
  }
  while (yaw < -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

double wrap_360(double deg)
{
  double wrapped = std::fmod(deg, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped;
}

double smoothstep(double u)
{
  u = clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

double smoothstep_integral(double u)
{
  u = clamp(u, 0.0, 1.0);
  return u * u * u - 0.5 * u * u * u * u;
}

double smootherstep(double u)
{
  u = clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double smootherstep_derivative(double u)
{
  u = clamp(u, 0.0, 1.0);
  return 30.0 * u * u * (1.0 - u) * (1.0 - u);
}

struct PathSample
{
  Vec3 position{};
  Vec3 tangent{1.0, 0.0, 0.0};
  double curvature{0.0};
};

class PathGeometry
{
public:
  static double project_line(
    const Vec3 & position, const Vec3 & origin, const Vec3 & unit, double length)
  {
    return clamp(dot3(position - origin, unit), 0.0, length);
  }

  static double project_line_unclamped(
    const Vec3 & position, const Vec3 & origin, const Vec3 & unit)
  {
    return dot3(position - origin, unit);
  }

  static PathSample sample_line(
    const Vec3 & origin, const Vec3 & unit, double length, double progress)
  {
    return {origin + unit * clamp(progress, 0.0, length), unit, 0.0};
  }
};

struct SpeedStep
{
  double speed{0.0};
  double acceleration{0.0};
  double distance{0.0};
};

class SpeedPlanner
{
public:
  void reset(double speed = 0.0)
  {
    speed_ = std::max(0.0, speed);
    acceleration_ = 0.0;
  }

  SpeedStep update(
    double target_speed, double dt, double max_acceleration, double max_deceleration,
    double max_jerk)
  {
    target_speed = std::max(0.0, target_speed);
    if (!std::isfinite(dt) || dt <= kSmall) {
      return {speed_, acceleration_, 0.0};
    }

    const double previous_speed = speed_;
    const double speed_error = target_speed - speed_;
    double desired_acceleration = 0.0;
    if (speed_error > kSmall) {
      // Start removing positive acceleration before reaching the speed target. Without this
      // velocity-domain braking term, a jerk-limited planner necessarily overshoots the target
      // while its acceleration ramps back to zero.
      const double acceleration_recovery_speed = acceleration_ > 0.0 ?
        acceleration_ * acceleration_ / (2.0 * max_jerk) : 0.0;
      desired_acceleration = speed_error <= acceleration_recovery_speed ?
        0.0 : max_acceleration;
    } else if (speed_error < -kSmall) {
      const double deceleration_recovery_speed = acceleration_ < 0.0 ?
        acceleration_ * acceleration_ / (2.0 * max_jerk) : 0.0;
      desired_acceleration = -speed_error <= deceleration_recovery_speed ?
        0.0 : -max_deceleration;
    }
    acceleration_ += clamp(
      desired_acceleration - acceleration_, -max_jerk * dt, max_jerk * dt);
    acceleration_ = clamp(acceleration_, -max_deceleration, max_acceleration);
    speed_ = std::max(0.0, speed_ + acceleration_ * dt);

    if ((target_speed - previous_speed) * (target_speed - speed_) <= 0.0) {
      speed_ = target_speed;
      acceleration_ = (speed_ - previous_speed) / dt;
    }

    return {
      speed_, acceleration_, 0.5 * (previous_speed + speed_) * dt};
  }

  double speed() const {return speed_;}
  double acceleration() const {return acceleration_;}

  void synchronize(double speed, double acceleration)
  {
    speed_ = std::max(0.0, speed);
    acceleration_ = acceleration;
  }

  double stopping_distance(double max_deceleration, double max_jerk) const
  {
    constexpr double kSimulationDt = 0.01;
    constexpr int kMaximumSteps = 2000;
    SpeedPlanner simulation = *this;
    double distance = 0.0;
    for (int step_index = 0; step_index < kMaximumSteps; ++step_index) {
      if (simulation.speed() <= 1e-3 && std::abs(simulation.acceleration()) <= 1e-2) {
        break;
      }
      const auto step = simulation.update(
        0.0, kSimulationDt, max_deceleration, max_deceleration, max_jerk);
      distance += step.distance;
    }
    return distance;
  }

private:
  double speed_{0.0};
  double acceleration_{0.0};
};

class SignedSpeedPlanner
{
public:
  void reset(double speed = 0.0)
  {
    speed_ = speed;
    acceleration_ = 0.0;
  }

  double update(double target_speed, double dt, double max_acceleration, double max_jerk)
  {
    if (!std::isfinite(target_speed) || !std::isfinite(dt) || dt <= kSmall) {
      return speed_;
    }
    const double previous_speed = speed_;
    const double speed_error = target_speed - speed_;
    double desired_acceleration = clamp(
      speed_error / std::max(dt, 0.05), -max_acceleration, max_acceleration);
    const double recovery_speed = acceleration_ * acceleration_ / (2.0 * max_jerk);
    if (std::abs(speed_error) <= recovery_speed && speed_error * acceleration_ < 0.0) {
      desired_acceleration = 0.0;
    }
    acceleration_ += clamp(
      desired_acceleration - acceleration_, -max_jerk * dt, max_jerk * dt);
    acceleration_ = clamp(acceleration_, -max_acceleration, max_acceleration);
    speed_ += acceleration_ * dt;
    if ((target_speed - previous_speed) * (target_speed - speed_) <= 0.0) {
      speed_ = target_speed;
      acceleration_ = (speed_ - previous_speed) / dt;
    }
    return speed_;
  }

  double speed() const {return speed_;}
  double acceleration() const {return acceleration_;}

private:
  double speed_{0.0};
  double acceleration_{0.0};
};

struct VelocityPathPoint
{
  Vec3 position{};
  Vec3 tangent{1.0, 0.0, 0.0};
  Vec3 normal{0.0, 1.0, 0.0};
  double progress{0.0};
  double curvature{0.0};
};

class PathProgressGovernor
{
public:
  struct Result
  {
    double target_speed{0.0};
    double lookahead{0.0};
    double lead{0.0};
  };

  static Result govern(
    double requested_speed, double reference_progress, double vehicle_progress,
    double vehicle_speed, double lookahead_time, double min_lookahead,
    double max_lookahead, double max_lead)
  {
    const double lead = reference_progress - vehicle_progress;
    const double lookahead = clamp(
      std::abs(vehicle_speed) * lookahead_time, min_lookahead, max_lookahead);
    double factor = 1.0;
    if (lead > lookahead) {
      factor = clamp(
        (max_lead - lead) / std::max(kSmall, max_lead - lookahead), 0.0, 1.0);
    }
    return {std::max(0.0, requested_speed) * factor, lookahead, lead};
  }
};
}  // namespace

class OffboardRouteOffsetNode : public rclcpp::Node
{
public:
  OffboardRouteOffsetNode()
  : Node("offboard_route_offset")
  {
    declare_parameters();
    load_parameters();
    validate_parameters();
    print_parameters();
    if (trajectory_control_backend_ == "velocity_path" && !velocity_backend_enabled()) {
      RCLCPP_WARN(
        get_logger(),
        "square stop_turn uses the retained position_feedforward corner state machine; "
        "select square_corner_mode=rounded for continuous velocity-path tracking");
    }

    const auto px4_qos = rclcpp::SensorDataQoS();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_qos.best_effort();
    pub_qos.durability_volatile();

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", pub_qos);
    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", pub_qos);
    trajectory_debug_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      "/offboard_route_offset/trajectory_debug", rclcpp::QoS(20));

    local_position_sub_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        local_position_ = *msg;
        local_position_received_ = true;
        last_local_position_time_ = steady_now();
      });

    vehicle_status_sub_ =
      create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        vehicle_status_ = *msg;
        vehicle_status_received_ = true;
      });

    vehicle_land_detected_sub_ =
      create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        vehicle_land_detected_ = *msg;
        vehicle_land_detected_received_ = true;
      });

    state_enter_time_ = steady_now();
    last_loop_time_ = steady_now();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / trajectory_setpoint_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {control_loop();});

    RCLCPP_WARN(
      get_logger(), "offboard_route_offset %s started", OFFBOARD_ROUTE_OFFSET_VERSION);
    RCLCPP_WARN(get_logger(),
      "Manual arm, takeoff, Offboard switch, and landing remain pilot tasks");
    RCLCPP_INFO(
      get_logger(),
      "PX4 input topics are shared with any Offboard controller; "
      "run only one Offboard node at a time");
  }

private:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;

  enum class State
  {
    WAIT_FOR_TAKEOFF_REFERENCE,
    IDLE,
    PRESTREAM,
    APPROACH_START,
    RUNNING_TRAJECTORY,
    TERMINAL_CAPTURE,
    MISSION_COMPLETE_HOLD,
    PAUSED_AFTER_OFFBOARD_EXIT,
    LANDING_OR_DISARMED,
    FAULT
  };

  enum class LinePhase
  {
    IDLE,
    ACCEL,
    CRUISE,
    BRAKE,
    SETTLE
  };

  enum class SquareState
  {
    IDLE,
    EDGE,
    VERTEX_SETTLE,
    YAW_TURN,
    YAW_SETTLE
  };

  struct LegProfile
  {
    double peak_speed{0.0};
    double accel_distance{0.0};
    double cruise_distance{0.0};
    double decel_distance{0.0};
    double cruise_time{0.0};
    double total_time{0.0};
  };

  void declare_parameters()
  {
    declare_parameter("trajectory_type", "line_reciprocating");
    declare_parameter("trajectory_control_backend", "velocity_path");
    declare_parameter("trajectory_setpoint_rate_hz", 50.0);
    declare_parameter("mission_start_altitude_m", 5.0);
    declare_parameter("mission_start_east_offset_m", 3.0);
    declare_parameter("mission_start_yaw_deg", 0.0);
    declare_parameter("approach_horizontal_speed_mps", 1.0);
    declare_parameter("approach_vertical_speed_mps", 0.5);
    declare_parameter("approach_arrival_tolerance_m", 0.20);
    declare_parameter("approach_yaw_tolerance_deg", 5.0);
    declare_parameter("approach_settle_time_s", 0.5);
    declare_parameter("approach_yaw_rate_dps", 45.0);
    declare_parameter("line_distance_m", 10.0);
    declare_parameter("line_speed_mps", 2.0);
    declare_parameter("line_reciprocation_count", 3);
    declare_parameter("line_endpoint_hold_time_s", 0.0);
    declare_parameter("line_endpoint_arrival_tolerance_m", 0.25);
    declare_parameter("line_endpoint_velocity_tolerance_mps", 0.25);
    declare_parameter("line_turn_yaw_rate_dps", 30.0);
    declare_parameter("line_turn_yaw_tolerance_deg", 5.0);
    declare_parameter("line_turn_settle_time_s", 0.2);
    declare_parameter("line_direction_yaw_deg", 0.0);
    declare_parameter("line_length", 3.0);
    declare_parameter("circle_radius", 1.5);
    declare_parameter("circle_turns", 1.0);
    declare_parameter("circle_direction", "cw");
    declare_parameter("continuous_trajectory", false);
    declare_parameter("square_side", 3.0);
    declare_parameter("sine_length", 4.0);
    declare_parameter("sine_amplitude", 1.0);
    declare_parameter("sine_cycles", 1.0);
    declare_parameter("z_width", 4.0);
    declare_parameter("z_height", 2.0);
    declare_parameter("z_repeats", 1);
    declare_parameter("curve_samples_per_meter", 8.0);
    declare_parameter("accel_time_s", 0.5);
    declare_parameter("decel_time_s", 0.5);
    declare_parameter("max_acceleration_mps2", 1.5);
    declare_parameter("max_deceleration_mps2", 1.8);
    declare_parameter("max_jerk_mps3", 4.0);
    declare_parameter("lookahead_time_s", 0.25);
    declare_parameter("min_lookahead_distance_m", 0.10);
    declare_parameter("max_lookahead_distance_m", 0.35);
    declare_parameter("max_reference_lead_distance_m", 0.50);
    declare_parameter("response_delay_s", 0.40);
    declare_parameter("stopping_safety_margin_m", 0.15);
    declare_parameter("endpoint_position_tolerance_m", 0.12);
    declare_parameter("endpoint_speed_tolerance_mps", 0.20);
    declare_parameter("endpoint_settle_time_s", 0.50);
    declare_parameter("endpoint_altitude_tolerance_m", 0.25);
    declare_parameter("endpoint_vertical_speed_tolerance_mps", 0.30);
    declare_parameter("max_safe_lateral_acceleration_mps2", 1.5);
    declare_parameter("cross_track_gain", 0.5);
    declare_parameter("max_cross_track_correction_speed_mps", 0.30);
    declare_parameter("cross_track_speed_ratio", 0.40);
    declare_parameter("cross_track_slowdown_start_m", 0.60);
    declare_parameter("cross_track_stop_error_m", 1.50);
    declare_parameter("projection_filter_time_constant_s", 0.15);
    declare_parameter("projection_search_window_m", 2.0);
    declare_parameter("terminal_capture_enabled", true);
    declare_parameter("terminal_capture_distance_m", 0.50);
    declare_parameter("terminal_gain", 0.5);
    declare_parameter("terminal_max_speed_mps", 0.15);
    declare_parameter("altitude_velocity_gain", 0.8);
    declare_parameter("max_vertical_speed_mps", 0.4);
    declare_parameter("max_vertical_acceleration_mps2", 0.5);
    declare_parameter("max_vertical_jerk_mps3", 1.0);
    declare_parameter("max_altitude_error_m", 2.0);
    declare_parameter("velocity_path_acceleration_feedforward_enabled", true);
    declare_parameter("square_corner_mode", "rounded");
    declare_parameter("square_corner_radius_m", 1.0);
    declare_parameter("line_turnaround_mode", "stop_reverse");
    declare_parameter("line_turn_radius_m", 2.0);
    declare_parameter("yaw_mode", "fixed");
    declare_parameter("yaw_rate_feedforward_enabled", true);
    declare_parameter("max_yaw_rate_dps", 45.0);
    declare_parameter("max_yaw_acceleration_dps2", 90.0);
    declare_parameter("vertex_yaw_rate_dps", 30.0);
    declare_parameter("vertex_position_tolerance_m", 0.20);
    declare_parameter("vertex_speed_tolerance_mps", 0.20);
    declare_parameter("vertex_yaw_tolerance_deg", 5.0);
    declare_parameter("vertex_yaw_settle_time_s", 0.30);
    declare_parameter("publish_rate_hz", 20.0);
    declare_parameter("local_position_timeout_s", 1.0);
    declare_parameter("prestream_after_exit_delay_s", 1.0);
    declare_parameter("minimum_start_altitude_m", 0.5);
    declare_parameter("takeoff_reference_candidate_timeout_s", 3.0);
  }

  void load_parameters()
  {
    trajectory_type_ = get_parameter("trajectory_type").as_string();
    trajectory_control_backend_ = get_parameter("trajectory_control_backend").as_string();
    trajectory_setpoint_rate_hz_ = get_parameter("trajectory_setpoint_rate_hz").as_double();
    mission_start_altitude_m_ = get_parameter("mission_start_altitude_m").as_double();
    mission_start_east_offset_m_ = get_parameter("mission_start_east_offset_m").as_double();
    mission_start_yaw_deg_ = wrap_360(get_parameter("mission_start_yaw_deg").as_double());
    approach_horizontal_speed_mps_ =
      get_parameter("approach_horizontal_speed_mps").as_double();
    approach_vertical_speed_mps_ = get_parameter("approach_vertical_speed_mps").as_double();
    approach_arrival_tolerance_m_ =
      get_parameter("approach_arrival_tolerance_m").as_double();
    approach_yaw_tolerance_deg_ = get_parameter("approach_yaw_tolerance_deg").as_double();
    approach_settle_time_s_ = get_parameter("approach_settle_time_s").as_double();
    approach_yaw_rate_dps_ = get_parameter("approach_yaw_rate_dps").as_double();
    line_distance_m_ = get_parameter("line_distance_m").as_double();
    line_speed_mps_ = get_parameter("line_speed_mps").as_double();
    line_reciprocation_count_ =
      static_cast<int>(get_parameter("line_reciprocation_count").as_int());
    line_endpoint_hold_time_s_ = get_parameter("line_endpoint_hold_time_s").as_double();
    line_endpoint_arrival_tolerance_m_ =
      get_parameter("line_endpoint_arrival_tolerance_m").as_double();
    line_endpoint_velocity_tolerance_mps_ =
      get_parameter("line_endpoint_velocity_tolerance_mps").as_double();
    line_turn_yaw_rate_dps_ = get_parameter("line_turn_yaw_rate_dps").as_double();
    line_turn_yaw_tolerance_deg_ = get_parameter("line_turn_yaw_tolerance_deg").as_double();
    line_turn_settle_time_s_ = get_parameter("line_turn_settle_time_s").as_double();
    line_direction_yaw_deg_ = wrap_360(get_parameter("line_direction_yaw_deg").as_double());
    line_length_ = get_parameter("line_length").as_double();
    circle_radius_ = get_parameter("circle_radius").as_double();
    circle_turns_ = get_parameter("circle_turns").as_double();
    circle_direction_ = get_parameter("circle_direction").as_string();
    continuous_trajectory_ = get_parameter("continuous_trajectory").as_bool();
    square_side_ = get_parameter("square_side").as_double();
    sine_length_ = get_parameter("sine_length").as_double();
    sine_amplitude_ = get_parameter("sine_amplitude").as_double();
    sine_cycles_ = get_parameter("sine_cycles").as_double();
    z_width_ = get_parameter("z_width").as_double();
    z_height_ = get_parameter("z_height").as_double();
    z_repeats_ = static_cast<int>(get_parameter("z_repeats").as_int());
    curve_samples_per_meter_ = get_parameter("curve_samples_per_meter").as_double();
    accel_time_s_ = get_parameter("accel_time_s").as_double();
    decel_time_s_ = get_parameter("decel_time_s").as_double();
    max_acceleration_mps2_ = get_parameter("max_acceleration_mps2").as_double();
    max_deceleration_mps2_ = get_parameter("max_deceleration_mps2").as_double();
    max_jerk_mps3_ = get_parameter("max_jerk_mps3").as_double();
    lookahead_time_s_ = get_parameter("lookahead_time_s").as_double();
    min_lookahead_distance_m_ = get_parameter("min_lookahead_distance_m").as_double();
    max_lookahead_distance_m_ = get_parameter("max_lookahead_distance_m").as_double();
    max_reference_lead_distance_m_ =
      get_parameter("max_reference_lead_distance_m").as_double();
    response_delay_s_ = get_parameter("response_delay_s").as_double();
    stopping_safety_margin_m_ = get_parameter("stopping_safety_margin_m").as_double();
    endpoint_position_tolerance_m_ = get_parameter("endpoint_position_tolerance_m").as_double();
    endpoint_speed_tolerance_mps_ = get_parameter("endpoint_speed_tolerance_mps").as_double();
    endpoint_settle_time_s_ = get_parameter("endpoint_settle_time_s").as_double();
    endpoint_altitude_tolerance_m_ =
      get_parameter("endpoint_altitude_tolerance_m").as_double();
    endpoint_vertical_speed_tolerance_mps_ =
      get_parameter("endpoint_vertical_speed_tolerance_mps").as_double();
    max_safe_lateral_acceleration_mps2_ =
      get_parameter("max_safe_lateral_acceleration_mps2").as_double();
    cross_track_gain_ = get_parameter("cross_track_gain").as_double();
    max_cross_track_correction_speed_mps_ =
      get_parameter("max_cross_track_correction_speed_mps").as_double();
    cross_track_speed_ratio_ = get_parameter("cross_track_speed_ratio").as_double();
    cross_track_slowdown_start_m_ = get_parameter("cross_track_slowdown_start_m").as_double();
    cross_track_stop_error_m_ = get_parameter("cross_track_stop_error_m").as_double();
    projection_filter_time_constant_s_ =
      get_parameter("projection_filter_time_constant_s").as_double();
    projection_search_window_m_ = get_parameter("projection_search_window_m").as_double();
    terminal_capture_enabled_ = get_parameter("terminal_capture_enabled").as_bool();
    terminal_capture_distance_m_ = get_parameter("terminal_capture_distance_m").as_double();
    terminal_gain_ = get_parameter("terminal_gain").as_double();
    terminal_max_speed_mps_ = get_parameter("terminal_max_speed_mps").as_double();
    altitude_velocity_gain_ = get_parameter("altitude_velocity_gain").as_double();
    max_vertical_speed_mps_ = get_parameter("max_vertical_speed_mps").as_double();
    max_vertical_acceleration_mps2_ =
      get_parameter("max_vertical_acceleration_mps2").as_double();
    max_vertical_jerk_mps3_ = get_parameter("max_vertical_jerk_mps3").as_double();
    max_altitude_error_m_ = get_parameter("max_altitude_error_m").as_double();
    velocity_path_acceleration_feedforward_enabled_ =
      get_parameter("velocity_path_acceleration_feedforward_enabled").as_bool();
    square_corner_mode_ = get_parameter("square_corner_mode").as_string();
    square_corner_radius_m_ = get_parameter("square_corner_radius_m").as_double();
    line_turnaround_mode_ = get_parameter("line_turnaround_mode").as_string();
    line_turn_radius_m_ = get_parameter("line_turn_radius_m").as_double();
    yaw_mode_ = get_parameter("yaw_mode").as_string();
    yaw_rate_feedforward_enabled_ = get_parameter("yaw_rate_feedforward_enabled").as_bool();
    max_yaw_rate_dps_ = get_parameter("max_yaw_rate_dps").as_double();
    max_yaw_acceleration_dps2_ = get_parameter("max_yaw_acceleration_dps2").as_double();
    vertex_yaw_rate_dps_ = get_parameter("vertex_yaw_rate_dps").as_double();
    vertex_position_tolerance_m_ = get_parameter("vertex_position_tolerance_m").as_double();
    vertex_speed_tolerance_mps_ = get_parameter("vertex_speed_tolerance_mps").as_double();
    vertex_yaw_tolerance_deg_ = get_parameter("vertex_yaw_tolerance_deg").as_double();
    vertex_yaw_settle_time_s_ = get_parameter("vertex_yaw_settle_time_s").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    local_position_timeout_s_ = get_parameter("local_position_timeout_s").as_double();
    prestream_after_exit_delay_s_ =
      get_parameter("prestream_after_exit_delay_s").as_double();
    minimum_start_altitude_m_ = get_parameter("minimum_start_altitude_m").as_double();
    takeoff_reference_candidate_timeout_s_ =
      get_parameter("takeoff_reference_candidate_timeout_s").as_double();
  }

  void validate_parameters() const
  {
    if (trajectory_type_ != "line_reciprocating" && trajectory_type_ != "line" &&
      trajectory_type_ != "circle" && trajectory_type_ != "sine" &&
      trajectory_type_ != "square" && trajectory_type_ != "z_shape")
    {
      throw std::runtime_error(
              "trajectory_type must be line_reciprocating, line, circle, sine, square, or z_shape");
    }
    if (trajectory_control_backend_ != "position_feedforward" &&
      trajectory_control_backend_ != "velocity_path")
    {
      throw std::runtime_error(
              "trajectory_control_backend must be position_feedforward or velocity_path");
    }
    require_finite_gt(
      "trajectory_setpoint_rate_hz", trajectory_setpoint_rate_hz_, kMinPublishRateHz);
    require_finite_le("trajectory_setpoint_rate_hz", trajectory_setpoint_rate_hz_, 200.0);
    require_finite_gt("mission_start_altitude_m", mission_start_altitude_m_,
      minimum_start_altitude_m_);
    require_finite("mission_start_east_offset_m", mission_start_east_offset_m_);
    require_finite("mission_start_yaw_deg", mission_start_yaw_deg_);
    require_finite_gt("approach_horizontal_speed_mps", approach_horizontal_speed_mps_, 0.0);
    require_finite_gt("approach_vertical_speed_mps", approach_vertical_speed_mps_, 0.0);
    require_finite_gt("approach_arrival_tolerance_m", approach_arrival_tolerance_m_, 0.0);
    require_finite_gt("approach_yaw_tolerance_deg", approach_yaw_tolerance_deg_, 0.0);
    require_finite_ge("approach_settle_time_s", approach_settle_time_s_, 0.0);
    require_finite_gt("approach_yaw_rate_dps", approach_yaw_rate_dps_, 0.0);
    require_finite_gt("line_distance_m", line_distance_m_, 0.0);
    require_finite_gt("line_speed_mps", line_speed_mps_, 0.0);
    if (line_reciprocation_count_ < 0) {
      throw std::runtime_error("line_reciprocation_count must be >= 0");
    }
    require_finite_ge("line_endpoint_hold_time_s", line_endpoint_hold_time_s_, 0.0);
    require_finite_gt(
      "line_endpoint_arrival_tolerance_m", line_endpoint_arrival_tolerance_m_, 0.0);
    require_finite_ge(
      "line_endpoint_velocity_tolerance_mps", line_endpoint_velocity_tolerance_mps_, 0.0);
    require_finite_gt("line_turn_yaw_rate_dps", line_turn_yaw_rate_dps_, 0.0);
    require_finite_gt("line_turn_yaw_tolerance_deg", line_turn_yaw_tolerance_deg_, 0.0);
    require_finite_ge("line_turn_settle_time_s", line_turn_settle_time_s_, 0.0);
    require_finite("line_direction_yaw_deg", line_direction_yaw_deg_);
    require_finite_gt("line_length", line_length_, 0.0);
    require_finite_gt("circle_radius", circle_radius_, 0.0);
    require_finite_gt("circle_turns", circle_turns_, 0.0);
    if (circle_direction_ != "cw" && circle_direction_ != "ccw") {
      throw std::runtime_error("circle_direction must be cw or ccw");
    }
    require_finite_gt("square_side", square_side_, 0.0);
    require_finite_gt("sine_length", sine_length_, 0.0);
    require_finite_ge("sine_amplitude", sine_amplitude_, 0.0);
    require_finite_gt("sine_cycles", sine_cycles_, 0.0);
    require_finite_gt("z_width", z_width_, 0.0);
    require_finite_gt("z_height", z_height_, 0.0);
    if (z_repeats_ < 1) {
      throw std::runtime_error("z_repeats must be >= 1");
    }
    require_finite_gt("curve_samples_per_meter", curve_samples_per_meter_, 0.0);
    require_finite_gt("accel_time_s", accel_time_s_, 0.0);
    require_finite_gt("decel_time_s", decel_time_s_, 0.0);
    require_finite_gt("max_acceleration_mps2", max_acceleration_mps2_, 0.0);
    require_finite_gt("max_deceleration_mps2", max_deceleration_mps2_, 0.0);
    require_finite_gt("max_jerk_mps3", max_jerk_mps3_, 0.0);
    require_finite_ge("lookahead_time_s", lookahead_time_s_, 0.0);
    require_finite_ge("min_lookahead_distance_m", min_lookahead_distance_m_, 0.0);
    require_finite_gt("max_lookahead_distance_m", max_lookahead_distance_m_, 0.0);
    require_finite_gt("max_reference_lead_distance_m", max_reference_lead_distance_m_, 0.0);
    if (min_lookahead_distance_m_ > max_lookahead_distance_m_ ||
      max_lookahead_distance_m_ >= max_reference_lead_distance_m_)
    {
      throw std::runtime_error(
              "lookahead distances must satisfy min <= max < max_reference_lead_distance_m");
    }
    require_finite_ge("response_delay_s", response_delay_s_, 0.0);
    require_finite_ge("stopping_safety_margin_m", stopping_safety_margin_m_, 0.0);
    require_finite_gt("endpoint_position_tolerance_m", endpoint_position_tolerance_m_, 0.0);
    require_finite_ge("endpoint_speed_tolerance_mps", endpoint_speed_tolerance_mps_, 0.0);
    require_finite_ge("endpoint_settle_time_s", endpoint_settle_time_s_, 0.0);
    require_finite_gt("endpoint_altitude_tolerance_m", endpoint_altitude_tolerance_m_, 0.0);
    require_finite_ge(
      "endpoint_vertical_speed_tolerance_mps", endpoint_vertical_speed_tolerance_mps_, 0.0);
    require_finite_gt(
      "max_safe_lateral_acceleration_mps2", max_safe_lateral_acceleration_mps2_, 0.0);
    require_finite_ge("cross_track_gain", cross_track_gain_, 0.0);
    require_finite_le("cross_track_gain", cross_track_gain_, 10.0);
    require_finite_gt(
      "max_cross_track_correction_speed_mps", max_cross_track_correction_speed_mps_, 0.0);
    require_finite_le(
      "max_cross_track_correction_speed_mps", max_cross_track_correction_speed_mps_, 10.0);
    require_finite_gt("cross_track_speed_ratio", cross_track_speed_ratio_, 0.0);
    if (cross_track_speed_ratio_ > 1.0) {
      throw std::runtime_error("cross_track_speed_ratio must be <= 1.0");
    }
    require_finite_ge(
      "cross_track_slowdown_start_m", cross_track_slowdown_start_m_, 0.0);
    require_finite_gt("cross_track_stop_error_m", cross_track_stop_error_m_, 0.0);
    if (cross_track_stop_error_m_ <= cross_track_slowdown_start_m_) {
      throw std::runtime_error(
              "cross_track_stop_error_m must be greater than cross_track_slowdown_start_m");
    }
    require_finite_ge(
      "projection_filter_time_constant_s", projection_filter_time_constant_s_, 0.0);
    require_finite_le(
      "projection_filter_time_constant_s", projection_filter_time_constant_s_, 10.0);
    require_finite_gt("projection_search_window_m", projection_search_window_m_, 0.0);
    require_finite_le("projection_search_window_m", projection_search_window_m_, 100.0);
    require_finite_gt("terminal_capture_distance_m", terminal_capture_distance_m_, 0.0);
    require_finite_le("terminal_capture_distance_m", terminal_capture_distance_m_, 100.0);
    require_finite_gt("terminal_gain", terminal_gain_, 0.0);
    require_finite_le("terminal_gain", terminal_gain_, 10.0);
    require_finite_gt("terminal_max_speed_mps", terminal_max_speed_mps_, 0.0);
    require_finite_le("terminal_max_speed_mps", terminal_max_speed_mps_, 10.0);
    require_finite_gt("altitude_velocity_gain", altitude_velocity_gain_, 0.0);
    require_finite_le("altitude_velocity_gain", altitude_velocity_gain_, 10.0);
    require_finite_gt("max_vertical_speed_mps", max_vertical_speed_mps_, 0.0);
    require_finite_le("max_vertical_speed_mps", max_vertical_speed_mps_, 10.0);
    require_finite_gt(
      "max_vertical_acceleration_mps2", max_vertical_acceleration_mps2_, 0.0);
    require_finite_le(
      "max_vertical_acceleration_mps2", max_vertical_acceleration_mps2_, 20.0);
    require_finite_gt("max_vertical_jerk_mps3", max_vertical_jerk_mps3_, 0.0);
    require_finite_le("max_vertical_jerk_mps3", max_vertical_jerk_mps3_, 100.0);
    require_finite_gt("max_altitude_error_m", max_altitude_error_m_, 0.0);
    if (square_corner_mode_ != "stop_turn" && square_corner_mode_ != "rounded") {
      throw std::runtime_error("square_corner_mode must be stop_turn or rounded");
    }
    require_finite_gt("square_corner_radius_m", square_corner_radius_m_, 0.0);
    if (line_turnaround_mode_ != "stop_reverse" && line_turnaround_mode_ != "racetrack") {
      throw std::runtime_error("line_turnaround_mode must be stop_reverse or racetrack");
    }
    require_finite_gt("line_turn_radius_m", line_turn_radius_m_, 0.0);
    if (yaw_mode_ != "fixed" && yaw_mode_ != "tangent") {
      throw std::runtime_error("yaw_mode must be fixed or tangent");
    }
    require_finite_gt("max_yaw_rate_dps", max_yaw_rate_dps_, 0.0);
    require_finite_le("max_yaw_rate_dps", max_yaw_rate_dps_, 360.0);
    require_finite_gt("max_yaw_acceleration_dps2", max_yaw_acceleration_dps2_, 0.0);
    require_finite_le("max_yaw_acceleration_dps2", max_yaw_acceleration_dps2_, 2000.0);
    require_finite_gt("vertex_yaw_rate_dps", vertex_yaw_rate_dps_, 0.0);
    require_finite_gt("vertex_position_tolerance_m", vertex_position_tolerance_m_, 0.0);
    require_finite_ge("vertex_speed_tolerance_mps", vertex_speed_tolerance_mps_, 0.0);
    require_finite_gt("vertex_yaw_tolerance_deg", vertex_yaw_tolerance_deg_, 0.0);
    require_finite_ge("vertex_yaw_settle_time_s", vertex_yaw_settle_time_s_, 0.0);
    require_finite_gt("publish_rate_hz", publish_rate_hz_, kMinPublishRateHz);
    require_finite_gt("local_position_timeout_s", local_position_timeout_s_, 0.0);
    require_finite_ge("prestream_after_exit_delay_s", prestream_after_exit_delay_s_, 0.0);
    require_finite_ge("minimum_start_altitude_m", minimum_start_altitude_m_, 0.0);
    require_finite_gt(
      "takeoff_reference_candidate_timeout_s", takeoff_reference_candidate_timeout_s_, 0.0);
  }

  static void require_finite(const char * name, double value)
  {
    if (!std::isfinite(value)) {
      throw std::runtime_error(std::string(name) + " must be finite");
    }
  }

  static void require_finite_gt(const char * name, double value, double limit)
  {
    if (!std::isfinite(value) || value <= limit) {
      std::ostringstream error;
      error << name << " must be > " << limit;
      throw std::runtime_error(error.str());
    }
  }

  static void require_finite_ge(const char * name, double value, double limit)
  {
    if (!std::isfinite(value) || value < limit) {
      std::ostringstream error;
      error << name << " must be >= " << limit;
      throw std::runtime_error(error.str());
    }
  }

  static void require_finite_le(const char * name, double value, double limit)
  {
    if (!std::isfinite(value) || value > limit) {
      std::ostringstream error;
      error << name << " must be <= " << limit;
      throw std::runtime_error(error.str());
    }
  }

  void print_parameters()
  {
    RCLCPP_INFO(
      get_logger(),
      "params: start_alt=%.2f m east_offset=%.2f m start_yaw=%.1f deg "
      "trajectory=%s backend=%s yaw_mode=%s approach_h=%.2f approach_v=%.2f "
      "line_distance=%.2f line_speed=%.2f repeat=%d "
      "accel=%.2f decel=%.2f max_accel=%.2f max_decel=%.2f jerk=%.2f "
      "max_lead=%.2f turn_yaw_rate=%.1f dps publish=%.1f Hz",
      mission_start_altitude_m_, mission_start_east_offset_m_, mission_start_yaw_deg_,
      trajectory_type_.c_str(), trajectory_control_backend_.c_str(), yaw_mode_.c_str(),
      approach_horizontal_speed_mps_, approach_vertical_speed_mps_,
      line_distance_m_,
      line_speed_mps_, line_reciprocation_count_, accel_time_s_, decel_time_s_,
      max_acceleration_mps2_, max_deceleration_mps2_, max_jerk_mps3_,
      max_reference_lead_distance_m_, line_turn_yaw_rate_dps_, trajectory_setpoint_rate_hz_);
    RCLCPP_INFO(
      get_logger(),
      "progress params: lookahead_time=%.2f s lookahead=[%.2f, %.2f] m response_delay=%.2f s "
      "stop_margin=%.2f m endpoint_h=[%.2f m, %.2f m/s] endpoint_v=[%.2f m, %.2f m/s] "
      "settle=%.2f s lateral_accel=%.2f m/s^2 "
      "vertex=[%.1f dps, %.2f m, %.2f m/s, %.1f deg, %.2f s]",
      lookahead_time_s_, min_lookahead_distance_m_, max_lookahead_distance_m_, response_delay_s_,
      stopping_safety_margin_m_, endpoint_position_tolerance_m_, endpoint_speed_tolerance_mps_,
      endpoint_altitude_tolerance_m_, endpoint_vertical_speed_tolerance_mps_,
      endpoint_settle_time_s_, max_safe_lateral_acceleration_mps2_, vertex_yaw_rate_dps_,
      vertex_position_tolerance_m_, vertex_speed_tolerance_mps_, vertex_yaw_tolerance_deg_,
      vertex_yaw_settle_time_s_);
    RCLCPP_INFO(
      get_logger(),
      "velocity path: cross_gain=%.2f 1/s cross_max=%.2f m/s ratio=%.2f slowdown=[%.2f, %.2f] m "
      "projection=[tau %.2f s, window %.2f m] terminal=[%s, %.2f m, gain %.2f 1/s, %.2f m/s]",
      cross_track_gain_, max_cross_track_correction_speed_mps_, cross_track_speed_ratio_,
      cross_track_slowdown_start_m_, cross_track_stop_error_m_,
      projection_filter_time_constant_s_, projection_search_window_m_,
      terminal_capture_enabled_ ? "on" : "off", terminal_capture_distance_m_, terminal_gain_,
      terminal_max_speed_mps_);
    RCLCPP_INFO(
      get_logger(),
      "vertical=[gain %.2f 1/s, speed %.2f m/s, accel %.2f m/s^2, jerk %.2f m/s^3] "
      "accel_ff=%s square=%s radius=%.2f m line_turn=%s radius=%.2f m "
      "yaw_ff=%s yaw_limits=[%.1f deg/s, %.1f deg/s^2] setpoint=%.1f Hz",
      altitude_velocity_gain_, max_vertical_speed_mps_, max_vertical_acceleration_mps2_,
      max_vertical_jerk_mps3_,
      velocity_path_acceleration_feedforward_enabled_ ? "on" : "off",
      square_corner_mode_.c_str(), square_corner_radius_m_, line_turnaround_mode_.c_str(),
      line_turn_radius_m_, yaw_rate_feedforward_enabled_ ? "on" : "off", max_yaw_rate_dps_,
      max_yaw_acceleration_dps2_, trajectory_setpoint_rate_hz_);
  }

  SteadyTime steady_now() const
  {
    return SteadyClock::now();
  }

  static double seconds_between(SteadyTime start, SteadyTime end)
  {
    return std::chrono::duration<double>(end - start).count();
  }

  double seconds_in_state() const
  {
    return seconds_between(state_enter_time_, steady_now());
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  static const char * state_name(State state)
  {
    switch (state) {
      case State::WAIT_FOR_TAKEOFF_REFERENCE:
        return "WAIT_FOR_TAKEOFF_REFERENCE";
      case State::IDLE:
        return "IDLE";
      case State::PRESTREAM:
        return "PRESTREAM";
      case State::APPROACH_START:
        return "APPROACH_START";
      case State::RUNNING_TRAJECTORY:
        return "RUNNING_TRAJECTORY";
      case State::TERMINAL_CAPTURE:
        return "TERMINAL_CAPTURE";
      case State::MISSION_COMPLETE_HOLD:
        return "MISSION_COMPLETE_HOLD";
      case State::PAUSED_AFTER_OFFBOARD_EXIT:
        return "PAUSED_AFTER_OFFBOARD_EXIT";
      case State::LANDING_OR_DISARMED:
        return "LANDING_OR_DISARMED";
      case State::FAULT:
        return "FAULT";
    }
    return "UNKNOWN";
  }

  void transition_to(State next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    state_ = next;
    state_enter_time_ = steady_now();
    if (!reason.empty()) {
      RCLCPP_INFO(get_logger(), "[%s] %s", state_name(state_), reason.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "[%s] entered", state_name(state_));
    }
  }

  bool local_position_fresh() const
  {
    return local_position_received_ &&
           seconds_between(last_local_position_time_, steady_now()) <= local_position_timeout_s_;
  }

  bool local_position_valid() const
  {
    return local_position_received_ &&
           local_position_.xy_valid &&
           local_position_.z_valid &&
           finite3(local_position_.x, local_position_.y, local_position_.z);
  }

  bool local_velocity_valid() const
  {
    return local_position_received_ &&
           local_position_.v_xy_valid &&
           local_position_.v_z_valid &&
           finite3(local_position_.vx, local_position_.vy, local_position_.vz);
  }

  bool local_pose_ready() const
  {
    return local_position_valid() && local_velocity_valid() && local_position_fresh();
  }

  Vec3 current_position() const
  {
    return {local_position_.x, local_position_.y, local_position_.z};
  }

  Vec3 current_velocity() const
  {
    return {local_position_.vx, local_position_.vy, local_position_.vz};
  }

  struct EndpointStatus
  {
    double horizontal_position_error{0.0};
    double altitude_error{0.0};
    double horizontal_speed{0.0};
    double vertical_speed{0.0};
    bool within_limits{false};
  };

  EndpointStatus endpoint_status(
    const Vec3 & target, double horizontal_position_tolerance,
    double horizontal_speed_tolerance, bool use_hysteresis) const
  {
    const auto position_error = current_position() - target;
    const auto velocity = current_velocity();
    const double factor = use_hysteresis ? 1.5 : 1.0;
    EndpointStatus status{};
    status.horizontal_position_error = norm2(position_error.x, position_error.y);
    status.altitude_error = std::abs(position_error.z);
    status.horizontal_speed = norm2(velocity.x, velocity.y);
    status.vertical_speed = std::abs(velocity.z);
    status.within_limits =
      status.horizontal_position_error <= factor * horizontal_position_tolerance &&
      status.altitude_error <= factor * endpoint_altitude_tolerance_m_ &&
      status.horizontal_speed <= factor * horizontal_speed_tolerance &&
      status.vertical_speed <= factor * endpoint_vertical_speed_tolerance_mps_;
    return status;
  }

  double current_heading() const
  {
    if (std::isfinite(local_position_.heading)) {
      return local_position_.heading;
    }
    return current_yaw_;
  }

  bool is_armed() const
  {
    return vehicle_status_received_ &&
           vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool is_disarmed() const
  {
    return vehicle_status_received_ &&
           vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
  }

  bool is_offboard() const
  {
    return vehicle_status_received_ &&
           vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool return_or_landing_nav_state() const
  {
    if (!vehicle_status_received_) {
      return false;
    }
    const auto nav_state = vehicle_status_.nav_state;
    return nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND ||
           nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND ||
           nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND ||
           nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL;
  }

  bool land_detector_on_ground() const
  {
    return vehicle_land_detected_received_ &&
           (vehicle_land_detected_.landed || vehicle_land_detected_.maybe_landed ||
           vehicle_land_detected_.ground_contact);
  }

  bool landing_or_disarmed() const
  {
    return return_or_landing_nav_state() || is_disarmed() || land_detector_on_ground();
  }

  bool safe_to_capture_takeoff_reference() const
  {
    if (!local_position_valid()) {
      return false;
    }
    if (!vehicle_land_detected_received_) {
      return true;
    }
    return land_detector_on_ground();
  }

  bool safe_to_update_ground_reference_candidate() const
  {
    if (!local_position_valid()) {
      return false;
    }
    if (!vehicle_land_detected_received_) {
      return true;
    }
    return land_detector_on_ground();
  }

  bool ground_reference_candidate_fresh() const
  {
    return ground_reference_candidate_valid_ &&
           seconds_between(ground_reference_candidate_time_, steady_now()) <=
           takeoff_reference_candidate_timeout_s_;
  }

  void update_ground_reference_candidate()
  {
    if (is_armed() || !safe_to_update_ground_reference_candidate()) {
      return;
    }
    ground_reference_candidate_ = current_position();
    ground_reference_candidate_valid_ = true;
    ground_reference_candidate_time_ = steady_now();
  }

  void clear_takeoff_reference(const std::string & reason)
  {
    if (takeoff_reference_valid_) {
      RCLCPP_INFO(get_logger(), "mission reset: %s", reason.c_str());
    }
    takeoff_reference_valid_ = false;
    have_setpoint_ = false;
    reset_runtime();
  }

  void capture_takeoff_reference(const std::string & reason)
  {
    takeoff_reference_ = current_position();
    takeoff_reference_valid_ = true;
    RCLCPP_INFO(
      get_logger(),
      "takeoff reference captured (%s): NED [%.3f, %.3f, %.3f]",
      reason.c_str(), takeoff_reference_.x, takeoff_reference_.y, takeoff_reference_.z);
  }

  void capture_takeoff_reference_from_candidate(const std::string & reason)
  {
    takeoff_reference_ = ground_reference_candidate_;
    takeoff_reference_valid_ = true;
    RCLCPP_INFO(
      get_logger(),
      "takeoff reference captured (%s): NED [%.3f, %.3f, %.3f]",
      reason.c_str(), takeoff_reference_.x, takeoff_reference_.y, takeoff_reference_.z);
  }

  void update_takeoff_reference_from_arming()
  {
    if (!vehicle_status_received_) {
      return;
    }

    const bool armed_now = is_armed();
    if (!armed_now) {
      update_ground_reference_candidate();
    }
    if (!arming_state_initialized_) {
      arming_state_initialized_ = true;
      was_armed_ = armed_now;
      if (armed_now) {
        if (safe_to_capture_takeoff_reference() && land_detector_on_ground()) {
          capture_takeoff_reference("node started while armed but land detector reports ground");
        } else {
          RCLCPP_WARN(
            get_logger(),
            "vehicle already armed when node started and no reliable "
            "ground reference is available; "
            "trajectory is disabled until the next disarm/arm cycle");
        }
      }
      return;
    }

    if (!was_armed_ && armed_now) {
      if (safe_to_capture_takeoff_reference()) {
        capture_takeoff_reference("arming transition");
      } else if (ground_reference_candidate_fresh()) {
        capture_takeoff_reference_from_candidate("recent pre-arm ground reference");
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "arming transition detected but neither current local/landed state nor recent pre-arm "
          "ground reference is reliable; "
          "takeoff reference was not captured");
      }
    } else if (was_armed_ && !armed_now) {
      clear_takeoff_reference("vehicle disarmed");
      ground_reference_candidate_valid_ = false;
    }
    was_armed_ = armed_now;
  }

  Vec3 configured_start_point() const
  {
    // PX4 local position is NED: x North, y East, z Down.
    // Positive mission_start_altitude_m means "up", so NED z decreases.
    return {
      takeoff_reference_.x,
      takeoff_reference_.y + mission_start_east_offset_m_,
      takeoff_reference_.z - mission_start_altitude_m_};
  }

  double mission_start_yaw_rad() const
  {
    // User definition: 0 deg North, 90 deg East. PX4 NED yaw uses the same convention.
    return wrap_pi(deg_to_rad(mission_start_yaw_deg_));
  }

  double yaw_from_horizontal_vector(const Vec3 & direction, double fallback_yaw) const
  {
    // PX4 NED yaw is atan2(East, North). Vertical-only motion keeps the previous/fallback yaw.
    if (norm2(direction.x, direction.y) <= kSmall) {
      return wrap_pi(fallback_yaw);
    }
    return wrap_pi(std::atan2(direction.y, direction.x));
  }

  double path_yaw_from_velocity_or_direction(
    const Vec3 & velocity, const Vec3 & direction, double fallback_yaw) const
  {
    if (norm2(velocity.x, velocity.y) > kSmall) {
      return yaw_from_horizontal_vector(velocity, fallback_yaw);
    }
    return yaw_from_horizontal_vector(direction, fallback_yaw);
  }

  double configured_trajectory_yaw(const Vec3 & direction, double fallback_yaw) const
  {
    return yaw_mode_ == "fixed" ? mission_start_yaw_rad() :
           yaw_from_horizontal_vector(direction, fallback_yaw);
  }

  void update_current_hold_setpoint()
  {
    if (!local_position_valid()) {
      have_setpoint_ = false;
      return;
    }
    current_setpoint_ = current_position();
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = current_heading();
    have_setpoint_ = true;
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = timestamp_us();
    const bool velocity_control = velocity_backend_enabled() &&
      (state_ == State::RUNNING_TRAJECTORY || state_ == State::TERMINAL_CAPTURE);
    msg.position = !velocity_control;
    msg.velocity = velocity_control;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    if (!have_setpoint_) {
      return;
    }
    const bool velocity_control = velocity_backend_enabled() &&
      (state_ == State::RUNNING_TRAJECTORY || state_ == State::TERMINAL_CAPTURE);
    if ((!velocity_control &&
      !finite3(current_setpoint_.x, current_setpoint_.y, current_setpoint_.z)) ||
      !finite3(current_velocity_ff_.x, current_velocity_ff_.y, current_velocity_ff_.z) ||
      (acceleration_feedforward_valid_ &&
      !finite3(current_acceleration_ff_.x, current_acceleration_ff_.y,
      current_acceleration_ff_.z)) ||
      !std::isfinite(current_yaw_))
    {
      transition_to(State::FAULT, "invalid setpoint; publication stopped");
      return;
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = timestamp_us();
    msg.position = velocity_control ? std::array<float, 3>{nan, nan, nan} :
    std::array<float, 3>{
      static_cast<float>(current_setpoint_.x),
      static_cast<float>(current_setpoint_.y),
      static_cast<float>(current_setpoint_.z)};
    msg.velocity = {
      static_cast<float>(current_velocity_ff_.x),
      static_cast<float>(current_velocity_ff_.y),
      static_cast<float>(current_velocity_ff_.z)};
    if (acceleration_feedforward_valid_) {
      msg.acceleration = {
        static_cast<float>(current_acceleration_ff_.x),
        static_cast<float>(current_acceleration_ff_.y),
        static_cast<float>(current_acceleration_ff_.z)};
    } else {
      msg.acceleration = {nan, nan, nan};
    }
    msg.jerk = {nan, nan, nan};
    msg.yaw = static_cast<float>(wrap_pi(current_yaw_));
    msg.yawspeed = velocity_control && std::isfinite(current_yawspeed_) ?
      static_cast<float>(current_yawspeed_) : nan;
    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_trajectory_debug()
  {
    if (state_ != State::RUNNING_TRAJECTORY && state_ != State::TERMINAL_CAPTURE) {
      return;
    }
    if (velocity_backend_enabled() && velocity_path_active_) {
      const auto sample = sample_velocity_path(velocity_s_filtered_);
      const double settle_elapsed = velocity_endpoint_settle_started_ ?
        seconds_between(velocity_endpoint_settle_start_time_, steady_now()) : 0.0;
      double trajectory_code = 0.0;
      if (trajectory_type_ == "line_reciprocating") {
        trajectory_code = 1.0;
      } else if (trajectory_type_ == "line") {
        trajectory_code = 2.0;
      } else if (trajectory_type_ == "circle") {
        trajectory_code = 3.0;
      } else if (trajectory_type_ == "sine") {
        trajectory_code = 4.0;
      } else if (trajectory_type_ == "square") {
        trajectory_code = 5.0;
      } else if (trajectory_type_ == "z_shape") {trajectory_code = 6.0;}
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {
        static_cast<double>(timestamp_us()) * 1e-6,
        trajectory_code,
        static_cast<double>(state_),
        1.0,
        yaw_mode_ == "tangent" ? 1.0 : 0.0,
        velocity_s_raw_,
        velocity_s_filtered_,
        velocity_s_plan_,
        velocity_remaining_distance_,
        velocity_projected_point_.x,
        velocity_projected_point_.y,
        velocity_cross_track_error_,
        velocity_along_track_error_,
        velocity_planner_target_speed_,
        velocity_planner_speed_,
        velocity_planner_acceleration_,
        velocity_tangent_speed_command_,
        velocity_normal_speed_command_,
        velocity_total_speed_command_,
        velocity_vertical_speed_command_,
        velocity_actual_horizontal_speed_,
        velocity_actual_tangent_speed_,
        velocity_actual_normal_speed_,
        std::abs(local_position_.vz),
        sample.curvature,
        velocity_curve_limit_mps_,
        velocity_terminal_speed_limit_mps_,
        velocity_cross_track_scale_,
        current_yaw_,
        current_yawspeed_,
        current_heading(),
        velocity_endpoint_position_ok_ ? 1.0 : 0.0,
        velocity_endpoint_speed_ok_ ? 1.0 : 0.0,
        velocity_endpoint_altitude_ok_ ? 1.0 : 0.0,
        velocity_endpoint_vertical_speed_ok_ ? 1.0 : 0.0,
        settle_elapsed};
      trajectory_debug_pub_->publish(msg);
      return;
    }
    double phase = 0.0;
    double s_ref = 0.0;
    double s_vehicle = 0.0;
    double lead = 0.0;
    double actual_speed = 0.0;
    double planned_speed = 0.0;
    double remaining = 0.0;
    double braking_distance = 0.0;
    double planned_acceleration = 0.0;
    double target_speed = 0.0;
    double lookahead = 0.0;
    double motion_state = 0.0;
    if (square_progress_active_) {
      phase = static_cast<double>(square_phase_);
      s_ref = square_s_ref_;
      s_vehicle = square_s_vehicle_;
      lead = square_lead_;
      actual_speed = square_actual_speed_;
      planned_speed = square_s_dot_ref_;
      remaining = square_remaining_distance_;
      braking_distance = square_braking_distance_;
      planned_acceleration = square_s_ddot_ref_;
      target_speed = square_target_speed_;
      lookahead = square_dynamic_lookahead_;
      motion_state = static_cast<double>(square_state_);
    } else if (line_leg_active_) {
      phase = static_cast<double>(last_line_phase_);
      s_ref = line_s_ref_;
      s_vehicle = line_s_vehicle_;
      lead = line_lead_;
      actual_speed = line_actual_speed_;
      planned_speed = line_s_dot_ref_;
      remaining = line_remaining_distance_;
      braking_distance = line_braking_distance_;
      planned_acceleration = line_s_ddot_ref_;
      target_speed = line_target_speed_;
      lookahead = line_dynamic_lookahead_;
    } else if (circle_progress_active_) {
      phase = static_cast<double>(circle_phase_);
      s_ref = circle_s_ref_;
      s_vehicle = circle_s_vehicle_;
      lead = circle_lead_;
      actual_speed = circle_actual_speed_;
      planned_speed = circle_s_dot_ref_;
      remaining = circle_remaining_distance_;
      braking_distance = circle_braking_distance_;
      planned_acceleration = circle_s_ddot_ref_;
      target_speed = circle_target_speed_;
      lookahead = circle_dynamic_lookahead_;
    } else if (sine_progress_active_) {
      phase = static_cast<double>(sine_phase_);
      s_ref = sine_s_ref_;
      s_vehicle = sine_s_vehicle_;
      lead = sine_lead_;
      actual_speed = sine_actual_speed_;
      planned_speed = sine_s_dot_ref_;
      remaining = sine_remaining_distance_;
      braking_distance = sine_braking_distance_;
      planned_acceleration = sine_s_ddot_ref_;
      target_speed = sine_target_speed_;
      lookahead = sine_dynamic_lookahead_;
    } else {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      phase, s_ref, s_vehicle, lead, actual_speed, planned_speed, remaining,
      braking_distance, planned_acceleration, target_speed, lookahead, motion_state};
    trajectory_debug_pub_->publish(msg);
  }

  bool offboard_control_mode_allowed() const
  {
    if (landing_or_disarmed()) {
      return false;
    }
    return state_ == State::PRESTREAM ||
           state_ == State::APPROACH_START ||
           state_ == State::RUNNING_TRAJECTORY ||
           state_ == State::TERMINAL_CAPTURE ||
           state_ == State::MISSION_COMPLETE_HOLD ||
           (state_ == State::PAUSED_AFTER_OFFBOARD_EXIT &&
           is_armed() && takeoff_reference_valid_ &&
           seconds_in_state() >= prestream_after_exit_delay_s_);
  }

  bool trajectory_setpoint_allowed() const
  {
    if (landing_or_disarmed()) {
      return false;
    }
    return state_ == State::PRESTREAM ||
           state_ == State::APPROACH_START ||
           state_ == State::RUNNING_TRAJECTORY ||
           state_ == State::TERMINAL_CAPTURE ||
           state_ == State::MISSION_COMPLETE_HOLD;
  }

  void publish_if_allowed()
  {
    if (!offboard_control_mode_allowed()) {
      return;
    }
    publish_offboard_control_mode();
    if (trajectory_setpoint_allowed()) {
      publish_trajectory_setpoint();
    }
  }

  void start_approach()
  {
    if (!takeoff_reference_valid_) {
      transition_to(State::FAULT, "Offboard entered without a valid takeoff reference");
      return;
    }
    if (!local_pose_ready()) {
      transition_to(State::FAULT, "Offboard entered without fresh local position");
      return;
    }

    reset_runtime();
    approach_start_position_ = current_position();
    approach_target_position_ = configured_start_point();
    const auto delta = approach_target_position_ - approach_start_position_;
    approach_start_yaw_ = current_heading();
    approach_target_yaw_ = mission_start_yaw_rad();
    const double horizontal_distance = norm2(delta.x, delta.y);
    const double vertical_distance = std::abs(delta.z);

    // Quintic smootherstep reaches a peak velocity 1.875 times the average.
    const double speed_shape_factor = 1.875;
    const double horizontal_time =
      horizontal_distance > kSmall ?
      speed_shape_factor * horizontal_distance / approach_horizontal_speed_mps_ : 0.0;
    const double vertical_time =
      vertical_distance > kSmall ?
      speed_shape_factor * vertical_distance / approach_vertical_speed_mps_ : 0.0;
    approach_duration_s_ = std::max({horizontal_time, vertical_time, 0.2});
    approach_start_time_ = steady_now();

    current_setpoint_ = approach_start_position_;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = yaw_from_horizontal_vector(delta, current_heading());
    have_setpoint_ = true;
    transition_to(State::APPROACH_START, "offboard entered; approaching configured start point");

    RCLCPP_INFO(
      get_logger(),
      "approach target NED [%.3f, %.3f, %.3f], start/path yaw %.1f deg, duration %.2f s",
      approach_target_position_.x, approach_target_position_.y, approach_target_position_.z,
      rad_to_deg(approach_target_yaw_), approach_duration_s_);
  }

  void update_approach()
  {
    const double elapsed = seconds_between(approach_start_time_, steady_now());
    const double u = clamp(elapsed / approach_duration_s_, 0.0, 1.0);
    const double s = smootherstep(u);
    const double ds = smootherstep_derivative(u) / approach_duration_s_;
    const auto delta = approach_target_position_ - approach_start_position_;

    current_setpoint_ = approach_start_position_ + delta * s;
    current_velocity_ff_ = delta * ds;

    current_yaw_ =
      path_yaw_from_velocity_or_direction(current_velocity_ff_, delta, approach_target_yaw_);
    have_setpoint_ = true;

    if (u < 1.0) {
      return;
    }

    current_setpoint_ = approach_target_position_;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = approach_target_yaw_;

    if (!local_pose_ready()) {
      return;
    }

    const double pos_error = norm3(current_position() - approach_target_position_);
    const double yaw_error_deg = std::abs(rad_to_deg(wrap_pi(current_heading() -
      approach_target_yaw_)));
    const bool approach_settle_complete =
      seconds_between(approach_settle_start_time_, steady_now()) >= approach_settle_time_s_;
    if (pos_error <= approach_arrival_tolerance_m_ &&
      yaw_error_deg <= approach_yaw_tolerance_deg_)
    {
      if (!approach_settle_started_) {
        approach_settle_started_ = true;
        approach_settle_start_time_ = steady_now();
        RCLCPP_INFO(get_logger(), "configured start point reached; settling");
      } else if (approach_settle_complete) {
        start_selected_trajectory();
      }
    } else {
      approach_settle_started_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting at configured start point: position error %.3f m, yaw error %.2f deg",
        pos_error, yaw_error_deg);
    }
  }

  LegProfile make_leg_profile(double distance) const
  {
    LegProfile profile{};
    const double full_speed_ramp_distance =
      0.5 * line_speed_mps_ * accel_time_s_ + 0.5 * line_speed_mps_ * decel_time_s_;
    if (distance >= full_speed_ramp_distance) {
      profile.peak_speed = line_speed_mps_;
    } else {
      profile.peak_speed = distance / (0.5 * (accel_time_s_ + decel_time_s_));
      RCLCPP_WARN_ONCE(
        get_logger(),
        "line distance is too short for requested speed; "
        "using triangular profile with peak %.3f m/s",
        profile.peak_speed);
    }

    profile.accel_distance = 0.5 * profile.peak_speed * accel_time_s_;
    profile.decel_distance = 0.5 * profile.peak_speed * decel_time_s_;
    profile.cruise_distance =
      std::max(0.0, distance - profile.accel_distance - profile.decel_distance);
    profile.cruise_time = profile.cruise_distance > kSmall ?
      profile.cruise_distance / profile.peak_speed : 0.0;
    profile.total_time = accel_time_s_ + profile.cruise_time + decel_time_s_;
    return profile;
  }

  Vec3 line_unit_vector() const
  {
    const double yaw = mission_start_yaw_rad();
    return {std::cos(yaw), std::sin(yaw), 0.0};
  }

  double line_leg_yaw() const
  {
    const auto direction = line_unit_ * (line_forward_leg_ ? 1.0 : -1.0);
    return yaw_from_horizontal_vector(direction, mission_start_yaw_rad());
  }

  Vec3 make_path_point(double forward, double right) const
  {
    const double yaw = mission_start_yaw_rad();
    const double north = forward * std::cos(yaw) - right * std::sin(yaw);
    const double east = forward * std::sin(yaw) + right * std::cos(yaw);
    const auto origin = configured_start_point();
    return {origin.x + north, origin.y + east, origin.z};
  }

  bool velocity_backend_enabled() const
  {
    return trajectory_control_backend_ == "velocity_path" &&
           !(trajectory_type_ == "square" && square_corner_mode_ == "stop_turn");
  }

  void append_velocity_path_point(const Vec3 & point)
  {
    if (!velocity_path_.empty() && norm3(point - velocity_path_.back().position) < 1e-4) {
      return;
    }
    VelocityPathPoint path_point{};
    path_point.position = point;
    velocity_path_.push_back(path_point);
  }

  void append_velocity_path_arc(
    double center_forward, double center_right, double radius, double angle_start,
    double angle_end)
  {
    const double arc_length = std::abs(angle_end - angle_start) * radius;
    const int samples = std::max(
      4, static_cast<int>(std::ceil(arc_length * curve_samples_per_meter_)));
    for (int index = 0; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / samples;
      const double angle = angle_start + (angle_end - angle_start) * ratio;
      append_velocity_path_point(make_path_point(
        center_forward + radius * std::cos(angle),
        center_right + radius * std::sin(angle)));
    }
  }

  bool finalize_velocity_path()
  {
    if (velocity_path_.size() < 2U) {
      return false;
    }
    double progress = 0.0;
    velocity_path_.front().progress = 0.0;
    for (std::size_t index = 1; index < velocity_path_.size(); ++index) {
      progress += norm3(
        velocity_path_[index].position - velocity_path_[index - 1U].position);
      velocity_path_[index].progress = progress;
    }
    velocity_path_length_ = progress;
    if (velocity_path_length_ <= kSmall) {
      return false;
    }
    for (std::size_t index = 0; index < velocity_path_.size(); ++index) {
      Vec3 delta{};
      if (index == 0U) {
        delta = velocity_path_[1U].position - velocity_path_[0U].position;
      } else if (index + 1U == velocity_path_.size()) {
        delta = velocity_path_[index].position - velocity_path_[index - 1U].position;
      } else {
        delta = velocity_path_[index + 1U].position - velocity_path_[index - 1U].position;
      }
      velocity_path_[index].tangent = normalized_xy(delta);
      velocity_path_[index].normal = {
        -velocity_path_[index].tangent.y, velocity_path_[index].tangent.x, 0.0};
    }
    for (std::size_t index = 1; index + 1U < velocity_path_.size(); ++index) {
      const auto & before = velocity_path_[index - 1U];
      const auto & after = velocity_path_[index + 1U];
      const double ds = std::max(kSmall, after.progress - before.progress);
      const double cross = before.tangent.x * after.tangent.y -
        before.tangent.y * after.tangent.x;
      const double cosine = clamp(
        before.tangent.x * after.tangent.x + before.tangent.y * after.tangent.y, -1.0, 1.0);
      velocity_path_[index].curvature = std::atan2(cross, cosine) / ds;
    }
    velocity_path_.front().curvature = velocity_path_[1U].curvature;
    velocity_path_.back().curvature = velocity_path_[velocity_path_.size() - 2U].curvature;
    return true;
  }

  bool build_velocity_line_leg()
  {
    velocity_path_.clear();
    line_leg_origin_ = line_forward_leg_ ?
      line_start_position_ : line_start_position_ + line_unit_ * line_distance_active_;
    line_leg_unit_ = line_unit_ * (line_forward_leg_ ? 1.0 : -1.0);
    append_velocity_path_point(line_leg_origin_);
    append_velocity_path_point(line_leg_origin_ + line_leg_unit_ * line_distance_active_);
    velocity_path_closed_ = false;
    velocity_mission_length_ = line_distance_active_;
    return finalize_velocity_path();
  }

  bool build_velocity_racetrack()
  {
    velocity_path_.clear();
    const double minimum_radius = line_speed_mps_ * line_speed_mps_ /
      max_safe_lateral_acceleration_mps2_;
    velocity_effective_turn_radius_m_ = std::max(line_turn_radius_m_, minimum_radius);
    if (velocity_effective_turn_radius_m_ > line_turn_radius_m_ + 1e-3) {
      RCLCPP_WARN(
        get_logger(), "racetrack radius increased from %.2f to %.2f m to satisfy v^2/a_lat",
        line_turn_radius_m_, velocity_effective_turn_radius_m_);
    }
    append_velocity_path_point(make_path_point(0.0, 0.0));
    append_velocity_path_point(make_path_point(line_distance_m_, 0.0));
    append_velocity_path_arc(
      line_distance_m_, velocity_effective_turn_radius_m_,
      velocity_effective_turn_radius_m_, -0.5 * kPi, 0.5 * kPi);
    append_velocity_path_point(make_path_point(0.0, 2.0 * velocity_effective_turn_radius_m_));
    append_velocity_path_arc(
      0.0, velocity_effective_turn_radius_m_, velocity_effective_turn_radius_m_,
      0.5 * kPi, 1.5 * kPi);
    velocity_path_closed_ = true;
    if (!finalize_velocity_path()) {
      return false;
    }
    velocity_mission_length_ = line_reciprocation_count_ == 0 ?
      std::numeric_limits<double>::infinity() :
      velocity_path_length_ * static_cast<double>(line_reciprocation_count_);
    return true;
  }

  void append_rounded_open_polyline(const std::vector<Vec3> & points, double corner_trim)
  {
    if (points.size() < 2U) {
      return;
    }
    append_velocity_path_point(points.front());
    for (std::size_t index = 1; index + 1U < points.size(); ++index) {
      const auto incoming_delta = points[index] - points[index - 1U];
      const auto outgoing_delta = points[index + 1U] - points[index];
      const double incoming_length = norm3(incoming_delta);
      const double outgoing_length = norm3(outgoing_delta);
      const double trim = std::min(
        corner_trim, 0.4 * std::min(incoming_length, outgoing_length));
      const auto incoming = incoming_delta * (1.0 / std::max(kSmall, incoming_length));
      const auto outgoing = outgoing_delta * (1.0 / std::max(kSmall, outgoing_length));
      const auto trim_in = points[index] - incoming * trim;
      const auto trim_out = points[index] + outgoing * trim;
      append_velocity_path_point(trim_in);
      const int samples = std::max(
        4, static_cast<int>(std::ceil(2.0 * trim * curve_samples_per_meter_)));
      for (int sample_index = 1; sample_index <= samples; ++sample_index) {
        const double u = static_cast<double>(sample_index) / samples;
        const double one_minus_u = 1.0 - u;
        append_velocity_path_point(
          trim_in * (one_minus_u * one_minus_u) +
          points[index] * (2.0 * one_minus_u * u) + trim_out * (u * u));
      }
    }
    append_velocity_path_point(points.back());
  }

  bool build_velocity_path()
  {
    velocity_path_.clear();
    velocity_path_closed_ = false;
    velocity_mission_length_ = 0.0;
    if (trajectory_type_ == "line" || trajectory_type_ == "line_reciprocating") {
      line_start_position_ = configured_start_point();
      line_unit_ = line_unit_vector();
      line_single_pass_ = trajectory_type_ == "line";
      line_distance_active_ = line_single_pass_ ? line_length_ : line_distance_m_;
      line_forward_leg_ = true;
      completed_reciprocations_ = 0;
      if (!line_single_pass_ && line_turnaround_mode_ == "racetrack") {
        return build_velocity_racetrack();
      }
      return build_velocity_line_leg();
    }
    if (trajectory_type_ == "circle") {
      const double turns = continuous_trajectory_ ? 1.0 : circle_turns_;
      const double lateral_sign = circle_direction_ == "ccw" ? -1.0 : 1.0;
      const int samples = std::max(
        24, static_cast<int>(std::ceil(
          2.0 * kPi * circle_radius_ * turns * curve_samples_per_meter_)));
      for (int index = 0; index <= samples; ++index) {
        const double theta = 2.0 * kPi * turns * static_cast<double>(index) / samples;
        append_velocity_path_point(make_path_point(
          circle_radius_ * std::sin(theta),
          lateral_sign * circle_radius_ * (1.0 - std::cos(theta))));
      }
      velocity_path_closed_ = continuous_trajectory_;
      if (!finalize_velocity_path()) {
        return false;
      }
      velocity_mission_length_ = continuous_trajectory_ ?
        std::numeric_limits<double>::infinity() : velocity_path_length_;
      return true;
    }
    if (trajectory_type_ == "sine") {
      const int samples = std::max(
        8, static_cast<int>(std::ceil(sine_length_ * curve_samples_per_meter_)));
      for (int index = 0; index <= samples; ++index) {
        const double ratio = static_cast<double>(index) / samples;
        append_velocity_path_point(make_path_point(
          sine_length_ * ratio,
          sine_amplitude_ * std::sin(2.0 * kPi * sine_cycles_ * ratio)));
      }
    } else if (trajectory_type_ == "square" && square_corner_mode_ == "rounded") {
      const double radius = std::min(square_corner_radius_m_, 0.49 * square_side_);
      append_velocity_path_point(make_path_point(0.0, 0.0));
      append_velocity_path_point(make_path_point(square_side_ - 2.0 * radius, 0.0));
      append_velocity_path_arc(
        square_side_ - 2.0 * radius, radius, radius, -0.5 * kPi, 0.0);
      append_velocity_path_point(make_path_point(square_side_ - radius, square_side_ - radius));
      append_velocity_path_arc(
        square_side_ - 2.0 * radius, square_side_ - radius, radius, 0.0, 0.5 * kPi);
      append_velocity_path_point(make_path_point(0.0, square_side_));
      append_velocity_path_arc(
        0.0, square_side_ - radius, radius, 0.5 * kPi, kPi);
      append_velocity_path_point(make_path_point(-radius, radius));
      append_velocity_path_arc(0.0, radius, radius, kPi, 1.5 * kPi);
      velocity_path_closed_ = continuous_trajectory_;
    } else {
      if (!generate_waypoint_trajectory()) {
        return false;
      }
      if (trajectory_type_ == "z_shape") {
        append_rounded_open_polyline(waypoint_points_, square_corner_radius_m_);
      } else {
        for (const auto & point : waypoint_points_) {
          append_velocity_path_point(point);
        }
      }
      if (trajectory_type_ == "square") {
        RCLCPP_WARN(
          get_logger(),
          "square stop_turn retains sharp, zero-speed corners; rounded is recommended");
      } else if (trajectory_type_ == "z_shape") {
        RCLCPP_WARN(get_logger(), "z_shape velocity path remains experimental");
      }
    }
    if (!finalize_velocity_path()) {
      return false;
    }
    velocity_mission_length_ = velocity_path_closed_ ?
      std::numeric_limits<double>::infinity() : velocity_path_length_;
    return true;
  }

  VelocityPathPoint sample_velocity_path(double progress) const
  {
    if (velocity_path_.empty()) {
      return {};
    }
    double local_progress = progress;
    if (velocity_path_closed_) {
      local_progress = std::fmod(local_progress, velocity_path_length_);
      if (local_progress < 0.0) {
        local_progress += velocity_path_length_;
      }
    } else {
      local_progress = clamp(local_progress, 0.0, velocity_path_length_);
    }
    auto upper = std::upper_bound(
      velocity_path_.begin(), velocity_path_.end(), local_progress,
      [](double value, const VelocityPathPoint & point) {return value < point.progress;});
    std::size_t index = static_cast<std::size_t>(std::distance(velocity_path_.begin(), upper));
    index = clamp_index(index, 1U, velocity_path_.size() - 1U);
    const auto & start = velocity_path_[index - 1U];
    const auto & end = velocity_path_[index];
    const double ratio = clamp(
      (local_progress - start.progress) / std::max(kSmall, end.progress - start.progress),
      0.0, 1.0);
    VelocityPathPoint result{};
    result.position = start.position + (end.position - start.position) * ratio;
    result.tangent = normalized_xy(
      start.tangent * (1.0 - ratio) + end.tangent * ratio, start.tangent);
    result.normal = {-result.tangent.y, result.tangent.x, 0.0};
    result.curvature = start.curvature * (1.0 - ratio) + end.curvature * ratio;
    result.progress = progress;
    return result;
  }

  double project_velocity_path(const Vec3 & position)
  {
    double best_error = std::numeric_limits<double>::infinity();
    double best_progress = velocity_projection_initialized_ ? velocity_s_raw_ : 0.0;
    const double lap = velocity_path_closed_ && velocity_path_length_ > kSmall ?
      std::floor(velocity_s_filtered_ / velocity_path_length_) : 0.0;
    for (std::size_t index = 1; index < velocity_path_.size(); ++index) {
      const auto & start = velocity_path_[index - 1U];
      const auto & end = velocity_path_[index];
      const auto segment = end.position - start.position;
      const double length_squared = dot3(segment, segment);
      if (length_squared <= kSmall) {
        continue;
      }
      const double ratio = clamp(dot3(position - start.position, segment) / length_squared, 0.0,
        1.0);
      const auto projected = start.position + segment * ratio;
      const auto error = position - projected;
      const double error_squared = error.x * error.x + error.y * error.y;
      const double local_progress = start.progress + (end.progress - start.progress) * ratio;
      for (int lap_offset = velocity_path_closed_ ? -1 : 0;
        lap_offset <= (velocity_path_closed_ ? 1 : 0); ++lap_offset)
      {
        const double candidate = local_progress +
          (lap + static_cast<double>(lap_offset)) * velocity_path_length_;
        if (velocity_projection_initialized_ &&
          std::abs(candidate - velocity_s_filtered_) > projection_search_window_m_)
        {
          continue;
        }
        if (error_squared < best_error) {
          best_error = error_squared;
          best_progress = candidate;
        }
      }
    }
    if (!velocity_projection_initialized_) {
      velocity_s_raw_ = std::max(0.0, best_progress);
      velocity_s_filtered_ = velocity_s_raw_;
      velocity_projection_initialized_ = true;
      return velocity_s_raw_;
    }
    const double horizontal_speed = norm2(local_position_.vx, local_position_.vy);
    const double maximum_step = std::min(
      projection_search_window_m_, std::max(0.05, (horizontal_speed + 1.0) * control_dt_s_));
    velocity_s_raw_ = clamp(
      best_progress, velocity_s_raw_ - maximum_step, velocity_s_raw_ + maximum_step);
    if (!velocity_path_closed_) {
      velocity_s_raw_ = clamp(velocity_s_raw_, 0.0, velocity_path_length_);
    }
    const double alpha = projection_filter_time_constant_s_ <= kSmall ? 1.0 :
      control_dt_s_ / (projection_filter_time_constant_s_ + control_dt_s_);
    velocity_s_filtered_ += alpha * (velocity_s_raw_ - velocity_s_filtered_);
    return velocity_s_raw_;
  }

  double velocity_curve_speed_limit(double progress) const
  {
    constexpr int kPreviewSamples = 16;
    const double preview_distance = std::max(
      max_lookahead_distance_m_,
      line_speed_mps_ * line_speed_mps_ / (2.0 * max_deceleration_mps2_) +
      line_speed_mps_ * response_delay_s_);
    double maximum_curvature = 0.0;
    for (int index = 0; index <= kPreviewSamples; ++index) {
      const double sample_progress = progress +
        preview_distance * static_cast<double>(index) / kPreviewSamples;
      if (!velocity_path_closed_ && sample_progress > velocity_path_length_) {
        break;
      }
      maximum_curvature = std::max(
        maximum_curvature, std::abs(sample_velocity_path(sample_progress).curvature));
    }
    if (maximum_curvature <= 1e-4) {
      return line_speed_mps_;
    }
    return std::min(
      line_speed_mps_,
      std::sqrt(max_safe_lateral_acceleration_mps2_ / maximum_curvature));
  }

  void initialize_yaw_controller(const Vec3 & tangent)
  {
    (void)tangent;
    velocity_fixed_yaw_ = mission_start_yaw_rad();
    velocity_yaw_unwrapped_ = current_heading();
    velocity_yaw_rate_ = 0.0;
    velocity_yaw_acceleration_ = 0.0;
    current_yaw_ = yaw_mode_ == "fixed" ? velocity_fixed_yaw_ : current_heading();
    velocity_yaw_unwrapped_ = current_yaw_;
    current_yawspeed_ = 0.0;
    RCLCPP_INFO(
      get_logger(), "mission yaw mode=%s fixed yaw=%.1f deg",
      yaw_mode_.c_str(), rad_to_deg(velocity_fixed_yaw_));
  }

  void update_velocity_yaw(const VelocityPathPoint & sample, double signed_tangent_speed)
  {
    if (yaw_mode_ == "fixed") {
      current_yaw_ = velocity_fixed_yaw_;
      current_yawspeed_ = 0.0;
      velocity_yaw_rate_ = 0.0;
      velocity_yaw_acceleration_ = 0.0;
      return;
    }
    const Vec3 direction = sample.tangent * (signed_tangent_speed < 0.0 ? -1.0 : 1.0);
    const double wrapped_target = yaw_from_horizontal_vector(direction, current_yaw_);
    const double unwrapped_target = velocity_yaw_unwrapped_ +
      wrap_pi(wrapped_target - wrap_pi(velocity_yaw_unwrapped_));
    const double max_rate = deg_to_rad(max_yaw_rate_dps_);
    const double max_acceleration = deg_to_rad(max_yaw_acceleration_dps2_);
    const double yaw_error = unwrapped_target - velocity_yaw_unwrapped_;
    const double feedforward_rate = yaw_rate_feedforward_enabled_ ?
      sample.curvature * signed_tangent_speed : 0.0;
    const double desired_rate = clamp(
      feedforward_rate + yaw_error / std::max(0.05, control_dt_s_), -max_rate, max_rate);
    const double previous_rate = velocity_yaw_rate_;
    velocity_yaw_rate_ += clamp(
      desired_rate - velocity_yaw_rate_, -max_acceleration * control_dt_s_,
      max_acceleration * control_dt_s_);
    const double next_yaw = velocity_yaw_unwrapped_ + velocity_yaw_rate_ * control_dt_s_;
    if ((unwrapped_target - velocity_yaw_unwrapped_) * (unwrapped_target - next_yaw) <= 0.0) {
      velocity_yaw_unwrapped_ = unwrapped_target;
      velocity_yaw_rate_ = 0.0;
    } else {
      velocity_yaw_unwrapped_ = next_yaw;
    }
    velocity_yaw_acceleration_ = (velocity_yaw_rate_ - previous_rate) / control_dt_s_;
    current_yaw_ = wrap_pi(velocity_yaw_unwrapped_);
    current_yawspeed_ = velocity_yaw_rate_;
  }

  bool reset_velocity_path_tracking()
  {
    velocity_projection_initialized_ = false;
    velocity_s_raw_ = 0.0;
    velocity_s_filtered_ = 0.0;
    project_velocity_path(current_position());
    const auto sample = sample_velocity_path(velocity_s_filtered_);
    velocity_actual_tangent_speed_ = dot3(current_velocity(), sample.tangent);
    velocity_speed_planner_.reset(std::max(0.0, velocity_actual_tangent_speed_));
    vertical_speed_planner_.reset(
      std::isfinite(local_position_.vz) ? local_position_.vz : 0.0);
    velocity_s_plan_ = velocity_s_filtered_;
    velocity_terminal_direction_ = 1.0;
    velocity_endpoint_settle_started_ = false;
    velocity_path_active_ = true;
    initialize_yaw_controller(sample.tangent);
    return true;
  }

  bool initialize_velocity_path_controller()
  {
    return build_velocity_path() && reset_velocity_path_tracking();
  }

  void start_velocity_path_trajectory()
  {
    if (!initialize_velocity_path_controller()) {
      transition_to(State::FAULT, "failed to build velocity path");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "velocity path started: %.2f m, %zu points, backend=%s",
      velocity_path_length_, velocity_path_.size(), trajectory_control_backend_.c_str());
    transition_to(State::RUNNING_TRAJECTORY, "velocity path trajectory started");
  }

  double cross_track_speed_scale(double absolute_error) const
  {
    if (absolute_error <= cross_track_slowdown_start_m_) {
      return 1.0;
    }
    const double ratio = clamp(
      (absolute_error - cross_track_slowdown_start_m_) /
      (cross_track_stop_error_m_ - cross_track_slowdown_start_m_), 0.0, 1.0);
    return 1.0 - smootherstep(ratio);
  }

  bool update_velocity_endpoint_completion(const Vec3 & endpoint)
  {
    const auto status = endpoint_status(
      endpoint, endpoint_position_tolerance_m_, endpoint_speed_tolerance_mps_,
      velocity_endpoint_settle_started_);
    velocity_endpoint_position_ok_ =
      std::abs(velocity_remaining_distance_ + velocity_along_track_error_) <=
      endpoint_position_tolerance_m_ &&
      status.horizontal_position_error <= endpoint_position_tolerance_m_;
    velocity_endpoint_speed_ok_ = status.horizontal_speed <= endpoint_speed_tolerance_mps_;
    velocity_endpoint_altitude_ok_ = status.altitude_error <= endpoint_altitude_tolerance_m_;
    velocity_endpoint_vertical_speed_ok_ =
      status.vertical_speed <= endpoint_vertical_speed_tolerance_mps_;
    const bool all_ok = velocity_endpoint_position_ok_ && velocity_endpoint_speed_ok_ &&
      velocity_endpoint_altitude_ok_ && velocity_endpoint_vertical_speed_ok_;
    if (!all_ok) {
      velocity_endpoint_settle_started_ = false;
      return false;
    }
    if (!velocity_endpoint_settle_started_) {
      velocity_endpoint_settle_started_ = true;
      velocity_endpoint_settle_start_time_ = steady_now();
      return false;
    }
    if (seconds_between(velocity_endpoint_settle_start_time_, steady_now()) <
      endpoint_settle_time_s_ + line_endpoint_hold_time_s_)
    {
      return false;
    }
    finish_velocity_path_segment(endpoint);
    return true;
  }

  void update_velocity_path_controller(bool terminal_capture)
  {
    if (!control_dt_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "abnormal control dt; retaining previous velocity command without advancing planners");
      have_setpoint_ = finite3(
        current_velocity_ff_.x, current_velocity_ff_.y, current_velocity_ff_.z);
      return;
    }
    if (velocity_reverse_yaw_pending_) {
      const auto turn_sample = sample_velocity_path(0.0);
      current_velocity_ff_ = {0.0, 0.0, vertical_speed_planner_.update(
        clamp(
          altitude_velocity_gain_ * (configured_start_point().z - local_position_.z),
          -max_vertical_speed_mps_, max_vertical_speed_mps_),
        control_dt_s_, max_vertical_acceleration_mps2_, max_vertical_jerk_mps3_)};
      update_velocity_yaw(turn_sample, 0.1);
      acceleration_feedforward_valid_ = false;
      have_setpoint_ = true;
      const double yaw_error = std::abs(rad_to_deg(wrap_pi(
        yaw_from_horizontal_vector(turn_sample.tangent, current_yaw_) - current_heading())));
      if (yaw_error <= line_turn_yaw_tolerance_deg_) {
        const bool yaw_settle_elapsed = velocity_reverse_yaw_settle_started_ &&
          seconds_between(velocity_reverse_yaw_settle_start_time_, steady_now()) >=
          line_turn_settle_time_s_;
        if (!velocity_reverse_yaw_settle_started_) {
          velocity_reverse_yaw_settle_started_ = true;
          velocity_reverse_yaw_settle_start_time_ = steady_now();
        } else if (yaw_settle_elapsed) {
          velocity_reverse_yaw_pending_ = false;
          velocity_reverse_yaw_settle_started_ = false;
          RCLCPP_INFO(get_logger(), "reverse tangent yaw settled; acceleration resumed");
        }
      } else {
        velocity_reverse_yaw_settle_started_ = false;
      }
      return;
    }
    project_velocity_path(current_position());
    const auto sample = sample_velocity_path(velocity_s_filtered_);
    velocity_projected_point_ = sample.position;
    const auto position_error = sample.position - current_position();
    velocity_cross_track_error_ = dot3(position_error, sample.normal);
    velocity_along_track_error_ = dot3(position_error, sample.tangent);
    const auto actual_velocity = current_velocity();
    velocity_actual_tangent_speed_ = dot3(actual_velocity, sample.tangent);
    velocity_actual_normal_speed_ = dot3(actual_velocity, sample.normal);
    velocity_actual_horizontal_speed_ = norm2(actual_velocity.x, actual_velocity.y);
    velocity_remaining_distance_ = std::isfinite(velocity_mission_length_) ?
      std::max(0.0, velocity_mission_length_ - velocity_s_filtered_) :
      std::numeric_limits<double>::infinity();
    velocity_curve_limit_mps_ = velocity_curve_speed_limit(velocity_s_filtered_);
    velocity_cross_track_scale_ = cross_track_speed_scale(std::abs(velocity_cross_track_error_));
    const double signed_endpoint_error =
      velocity_remaining_distance_ + velocity_along_track_error_;
    const double terminal_position_error =
      norm2(signed_endpoint_error, velocity_cross_track_error_);

    const double braking_margin = terminal_capture_enabled_ ? stopping_safety_margin_m_ : 0.0;
    const double endpoint_speed_limit = std::isfinite(velocity_remaining_distance_) ?
      std::sqrt(2.0 * max_deceleration_mps2_ *
      std::max(0.0, velocity_remaining_distance_ - braking_margin)) :
      line_speed_mps_;
    velocity_terminal_speed_limit_mps_ = terminal_capture ?
      std::min(terminal_max_speed_mps_, terminal_gain_ *
      terminal_position_error) :
      endpoint_speed_limit;
    double requested_speed = terminal_capture ?
      std::min(
      {line_speed_mps_, velocity_curve_limit_mps_, velocity_terminal_speed_limit_mps_}) :
      std::min({line_speed_mps_, velocity_curve_limit_mps_, endpoint_speed_limit});
    if (terminal_capture) {
      const double desired_direction = signed_endpoint_error < 0.0 ? -1.0 : 1.0;
      if (desired_direction != velocity_terminal_direction_) {
        if (velocity_speed_planner_.speed() <= 0.03) {
          velocity_terminal_direction_ = desired_direction;
          velocity_speed_planner_.reset();
        } else {
          requested_speed = 0.0;
        }
      }
    }
    requested_speed *= velocity_cross_track_scale_;
    velocity_planner_target_speed_ = requested_speed;
    const auto speed_step = velocity_speed_planner_.update(
      requested_speed, control_dt_s_, max_acceleration_mps2_, max_deceleration_mps2_,
      max_jerk_mps3_);
    velocity_planner_speed_ = speed_step.speed;
    velocity_planner_acceleration_ = speed_step.acceleration;
    velocity_s_plan_ = velocity_s_filtered_ +
      clamp(velocity_planner_speed_ * lookahead_time_s_, min_lookahead_distance_m_,
      max_lookahead_distance_m_);

    if (!terminal_capture && terminal_capture_enabled_ &&
      std::isfinite(velocity_remaining_distance_) &&
      velocity_remaining_distance_ <= terminal_capture_distance_m_)
    {
      transition_to(State::TERMINAL_CAPTURE, "terminal capture entered");
      velocity_endpoint_settle_started_ = false;
    }

    const double normal_limit = std::min(
      max_cross_track_correction_speed_mps_,
      cross_track_speed_ratio_ * velocity_planner_speed_);
    velocity_normal_speed_command_ = clamp(
      cross_track_gain_ * velocity_cross_track_error_, -normal_limit, normal_limit);
    velocity_tangent_speed_command_ = std::sqrt(std::max(
      0.0, velocity_planner_speed_ * velocity_planner_speed_ -
      velocity_normal_speed_command_ * velocity_normal_speed_command_));
    if (terminal_capture) {
      velocity_tangent_speed_command_ = std::min(
        velocity_tangent_speed_command_, terminal_gain_ * std::abs(signed_endpoint_error));
    }

    const double altitude_error = configured_start_point().z - local_position_.z;
    if (!std::isfinite(altitude_error) || std::abs(altitude_error) > max_altitude_error_m_) {
      have_setpoint_ = false;
      transition_to(State::FAULT, "invalid or excessive altitude error in velocity path");
      return;
    }
    const double vertical_target = clamp(
      altitude_velocity_gain_ * altitude_error, -max_vertical_speed_mps_,
      max_vertical_speed_mps_);
    velocity_vertical_speed_command_ = vertical_speed_planner_.update(
      vertical_target, control_dt_s_, max_vertical_acceleration_mps2_,
      max_vertical_jerk_mps3_);

    current_setpoint_ = sample.position;
    current_velocity_ff_ =
      sample.tangent * (velocity_terminal_direction_ * velocity_tangent_speed_command_) +
      sample.normal * velocity_normal_speed_command_;
    current_velocity_ff_.z = velocity_vertical_speed_command_;
    velocity_total_speed_command_ = norm2(current_velocity_ff_.x, current_velocity_ff_.y);
    current_acceleration_ff_ =
      sample.tangent * velocity_planner_acceleration_ +
      sample.normal * (sample.curvature * velocity_tangent_speed_command_ *
      velocity_tangent_speed_command_);
    acceleration_feedforward_valid_ =
      velocity_path_acceleration_feedforward_enabled_ && !terminal_capture;
    update_velocity_yaw(sample, velocity_terminal_direction_ * velocity_tangent_speed_command_);
    have_setpoint_ = true;

    const bool endpoint_completion_active = terminal_capture ||
      (!terminal_capture_enabled_ && std::isfinite(velocity_remaining_distance_) &&
      velocity_remaining_distance_ <= terminal_capture_distance_m_);
    if (endpoint_completion_active) {
      const auto endpoint = sample_velocity_path(velocity_mission_length_).position;
      if (update_velocity_endpoint_completion(endpoint)) {
        return;
      }
    }
  }

  void finish_velocity_path_segment(const Vec3 & endpoint)
  {
    if (trajectory_type_ == "line_reciprocating" && line_turnaround_mode_ == "stop_reverse") {
      if (!line_forward_leg_) {
        ++completed_reciprocations_;
      }
      if (line_reciprocation_count_ == 0 ||
        completed_reciprocations_ < line_reciprocation_count_)
      {
        line_forward_leg_ = !line_forward_leg_;
        const double previous_yaw_unwrapped = velocity_yaw_unwrapped_;
        const double previous_yaw_rate = velocity_yaw_rate_;
        const double previous_current_yaw = current_yaw_;
        if (!build_velocity_line_leg() || !reset_velocity_path_tracking()) {
          transition_to(State::FAULT, "failed to initialize reverse line leg");
          return;
        }
        if (yaw_mode_ == "tangent") {
          velocity_yaw_unwrapped_ = previous_yaw_unwrapped;
          velocity_yaw_rate_ = previous_yaw_rate;
          current_yaw_ = previous_current_yaw;
          velocity_reverse_yaw_pending_ = true;
          velocity_reverse_yaw_settle_started_ = false;
        }
        transition_to(State::RUNNING_TRAJECTORY,
          yaw_mode_ == "fixed" ? "reverse leg started with fixed yaw" :
          "reverse leg started with tangent yaw");
        return;
      }
    }
    velocity_path_active_ = false;
    mission_complete_position_ = endpoint;
    mission_complete_yaw_ = current_yaw_;
    current_setpoint_ = mission_complete_position_;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    transition_to(State::MISSION_COMPLETE_HOLD, "velocity path complete; holding exact endpoint");
  }

  void start_selected_trajectory()
  {
    if (velocity_backend_enabled()) {
      start_velocity_path_trajectory();
      return;
    }
    if (trajectory_type_ == "line_reciprocating" || trajectory_type_ == "line") {
      start_line_trajectory();
      return;
    }
    start_waypoint_trajectory();
  }

  void start_line_trajectory()
  {
    line_single_pass_ = trajectory_type_ == "line";
    line_distance_active_ = line_single_pass_ ? line_length_ : line_distance_m_;
    line_start_position_ = configured_start_point();
    line_unit_ = line_unit_vector();
    line_forward_leg_ = true;
    completed_reciprocations_ = 0;
    line_leg_index_ = 0;
    current_yaw_ = yaw_from_horizontal_vector(line_unit_, mission_start_yaw_rad());
    start_next_line_leg("trajectory started");
    transition_to(State::RUNNING_TRAJECTORY, "trajectory started");
  }

  void start_waypoint_trajectory()
  {
    if (!generate_waypoint_trajectory()) {
      transition_to(State::FAULT, "failed to generate selected trajectory");
      return;
    }
    waypoint_index_ = 1;
    waypoint_segment_active_ = false;
    waypoint_complete_ = false;
    current_yaw_ = mission_start_yaw_rad();
    if (waypoint_uses_continuous_profile()) {
      start_continuous_waypoint_trajectory("trajectory started");
    } else {
      start_next_waypoint_segment("trajectory started");
    }
    transition_to(State::RUNNING_TRAJECTORY, "trajectory started");
  }

  bool generate_waypoint_trajectory()
  {
    waypoint_points_.clear();
    waypoint_points_.push_back(configured_start_point());

    if (trajectory_type_ == "line") {
      waypoint_points_.push_back(make_path_point(line_length_, 0.0));
    } else if (trajectory_type_ == "square") {
      waypoint_points_.push_back(make_path_point(square_side_, 0.0));
      waypoint_points_.push_back(make_path_point(square_side_, square_side_));
      waypoint_points_.push_back(make_path_point(0.0, square_side_));
      waypoint_points_.push_back(make_path_point(0.0, 0.0));
    } else if (trajectory_type_ == "z_shape") {
      const double band_height = z_height_ / static_cast<double>(z_repeats_);
      for (int i = 0; i < z_repeats_; ++i) {
        const double y0 = static_cast<double>(i) * band_height;
        const double y1 = static_cast<double>(i + 1) * band_height;
        if (i % 2 == 0) {
          waypoint_points_.push_back(make_path_point(z_width_, y0));
          waypoint_points_.push_back(make_path_point(0.0, y1));
          waypoint_points_.push_back(make_path_point(z_width_, y1));
        } else {
          waypoint_points_.push_back(make_path_point(0.0, y0));
          waypoint_points_.push_back(make_path_point(z_width_, y1));
          waypoint_points_.push_back(make_path_point(0.0, y1));
        }
      }
    } else if (trajectory_type_ == "sine") {
      const int samples = static_cast<int>(
        std::max(2.0, std::ceil(sine_length_ * curve_samples_per_meter_)));
      for (int i = 1; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples);
        waypoint_points_.push_back(make_path_point(
          sine_length_ * ratio,
          sine_amplitude_ * std::sin(2.0 * kPi * sine_cycles_ * ratio)));
      }
    } else if (trajectory_type_ == "circle") {
      const double turns = continuous_trajectory_ ? 1.0 : circle_turns_;
      const double lateral_sign = circle_direction_ == "ccw" ? -1.0 : 1.0;
      const double circumference = 2.0 * kPi * circle_radius_ * turns;
      const int samples = static_cast<int>(
        std::max(12.0, std::ceil(circumference * curve_samples_per_meter_)));
      for (int i = 1; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples);
        const double theta = 2.0 * kPi * turns * ratio;
        // Local path frame: forward is mission_start_yaw_deg, right is +90 deg from it.
        // The circle starts at the fixed start point with tangent along +forward.
        // cw turns toward +right, ccw turns toward -right, and integer turns close at the start.
        waypoint_points_.push_back(make_path_point(
          circle_radius_ * std::sin(theta),
          lateral_sign * circle_radius_ * (1.0 - std::cos(theta))));
      }
    }

    if (waypoint_points_.size() < 2U) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "generated '%s' trajectory with %zu point(s)",
      trajectory_type_.c_str(), waypoint_points_.size());
    return true;
  }

  bool waypoint_pass_through_segment() const
  {
    return false;
  }

  bool waypoint_uses_continuous_profile() const
  {
    return trajectory_type_ == "circle" || trajectory_type_ == "sine";
  }

  bool build_waypoint_cumulative_lengths()
  {
    waypoint_cumulative_lengths_.clear();
    waypoint_cumulative_lengths_.reserve(waypoint_points_.size());
    waypoint_cumulative_lengths_.push_back(0.0);
    double total = 0.0;
    for (std::size_t i = 1; i < waypoint_points_.size(); ++i) {
      total += norm3(waypoint_points_[i] - waypoint_points_[i - 1U]);
      waypoint_cumulative_lengths_.push_back(total);
    }
    waypoint_total_length_ = total;
    return waypoint_total_length_ > kSmall;
  }

  void start_continuous_waypoint_trajectory(const std::string & log_reason)
  {
    if (!build_waypoint_cumulative_lengths()) {
      transition_to(State::FAULT, "continuous waypoint path has zero length");
      return;
    }
    waypoint_continuous_active_ = true;
    waypoint_segment_active_ = false;
    waypoint_continuous_start_time_ = steady_now();
    waypoint_profile_ = make_leg_profile(waypoint_total_length_);
    current_setpoint_ = waypoint_points_.front();
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = mission_start_yaw_rad();
    have_setpoint_ = true;
    if (trajectory_type_ == "circle") {
      initialize_circle_progress();
    } else if (trajectory_type_ == "sine") {
      initialize_sine_progress();
    }
    RCLCPP_INFO(
      get_logger(),
      "%s; continuous '%s' path length %.2f m with %zu point(s)",
      log_reason.empty() ? "trajectory started" : log_reason.c_str(),
      trajectory_type_.c_str(), waypoint_total_length_, waypoint_points_.size());
  }

  Vec3 interpolate_continuous_waypoint(double distance_along, Vec3 * unit_out) const
  {
    if (waypoint_points_.empty() || waypoint_cumulative_lengths_.empty()) {
      if (unit_out != nullptr) {
        *unit_out = {};
      }
      return configured_start_point();
    }

    const double distance = clamp(distance_along, 0.0, waypoint_total_length_);
    auto upper = std::upper_bound(
      waypoint_cumulative_lengths_.begin(), waypoint_cumulative_lengths_.end(), distance);
    std::size_t index = static_cast<std::size_t>(
      std::distance(waypoint_cumulative_lengths_.begin(), upper));
    index = clamp_index(index, 1U, waypoint_points_.size() - 1U);

    const double segment_start_distance = waypoint_cumulative_lengths_[index - 1U];
    const double segment_end_distance = waypoint_cumulative_lengths_[index];
    const double segment_length = std::max(kSmall, segment_end_distance - segment_start_distance);
    const auto delta = waypoint_points_[index] - waypoint_points_[index - 1U];
    const auto unit = delta * (1.0 / std::max(kSmall, norm3(delta)));
    if (unit_out != nullptr) {
      *unit_out = unit;
    }
    const double segment_ratio = clamp((distance - segment_start_distance) / segment_length, 0.0,
      1.0);
    return waypoint_points_[index - 1U] + delta * segment_ratio;
  }

  double project_continuous_waypoint(const Vec3 & position) const
  {
    if (waypoint_points_.size() < 2U || waypoint_cumulative_lengths_.size() < 2U) {
      return 0.0;
    }

    double best_distance_squared = std::numeric_limits<double>::infinity();
    double best_progress = 0.0;
    for (std::size_t index = 1; index < waypoint_points_.size(); ++index) {
      const auto segment = waypoint_points_[index] - waypoint_points_[index - 1U];
      const double length_squared = dot3(segment, segment);
      if (length_squared <= kSmall) {
        continue;
      }
      const double ratio = clamp(
        dot3(position - waypoint_points_[index - 1U], segment) / length_squared,
        0.0, 1.0);
      const auto projected = waypoint_points_[index - 1U] + segment * ratio;
      const auto error = position - projected;
      const double distance_squared = dot3(error, error);
      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        const double segment_length =
          waypoint_cumulative_lengths_[index] - waypoint_cumulative_lengths_[index - 1U];
        best_progress = waypoint_cumulative_lengths_[index - 1U] + segment_length * ratio;
      }
    }
    return clamp(best_progress, 0.0, waypoint_total_length_);
  }

  double sine_x_from_arc_length(double progress) const
  {
    if (waypoint_points_.size() < 2U || waypoint_cumulative_lengths_.size() < 2U) {
      return 0.0;
    }
    const double distance = clamp(progress, 0.0, waypoint_total_length_);
    auto upper = std::upper_bound(
      waypoint_cumulative_lengths_.begin(), waypoint_cumulative_lengths_.end(), distance);
    std::size_t index = static_cast<std::size_t>(
      std::distance(waypoint_cumulative_lengths_.begin(), upper));
    index = clamp_index(index, 1U, waypoint_points_.size() - 1U);
    const double segment_start = waypoint_cumulative_lengths_[index - 1U];
    const double segment_end = waypoint_cumulative_lengths_[index];
    const double ratio = clamp(
      (distance - segment_start) / std::max(kSmall, segment_end - segment_start),
      0.0, 1.0);
    const double denominator = static_cast<double>(waypoint_points_.size() - 1U);
    const double x0 = sine_length_ * static_cast<double>(index - 1U) / denominator;
    const double x1 = sine_length_ * static_cast<double>(index) / denominator;
    return x0 + (x1 - x0) * ratio;
  }

  PathSample sample_sine(double progress, Vec3 * curvature_vector) const
  {
    const double x = sine_x_from_arc_length(progress);
    const double wave_number = 2.0 * kPi * sine_cycles_ / sine_length_;
    const double phase = wave_number * x;
    const double slope = sine_amplitude_ * wave_number * std::cos(phase);
    const double second_derivative =
      -sine_amplitude_ * wave_number * wave_number * std::sin(phase);
    const double tangent_scale = std::sqrt(1.0 + slope * slope);
    const auto forward = line_unit_vector();
    const Vec3 right{-forward.y, forward.x, 0.0};
    const auto tangent = (forward + right * slope) * (1.0 / tangent_scale);
    const double curvature =
      std::abs(second_derivative) / (tangent_scale * tangent_scale * tangent_scale);
    if (curvature_vector != nullptr) {
      const double scale = second_derivative /
        (tangent_scale * tangent_scale * tangent_scale * tangent_scale);
      *curvature_vector = (right - forward * slope) * scale;
    }
    return {
      make_path_point(x, sine_amplitude_ * std::sin(phase)), tangent, curvature};
  }

  double sine_curve_speed_limit(double progress) const
  {
    constexpr int kPreviewSamples = 16;
    const double requested_braking_distance =
      line_speed_mps_ * line_speed_mps_ / (2.0 * max_deceleration_mps2_) +
      line_speed_mps_ * response_delay_s_ + max_lookahead_distance_m_;
    const double preview_distance = std::min(
      waypoint_total_length_ - progress,
      std::max(max_lookahead_distance_m_, requested_braking_distance));
    double maximum_curvature = 0.0;
    for (int index = 0; index <= kPreviewSamples; ++index) {
      const double ratio = static_cast<double>(index) / kPreviewSamples;
      maximum_curvature = std::max(
        maximum_curvature, sample_sine(progress + preview_distance * ratio, nullptr).curvature);
    }
    if (maximum_curvature <= 1e-4) {
      return line_speed_mps_;
    }
    return std::min(
      line_speed_mps_,
      std::sqrt(max_safe_lateral_acceleration_mps2_ / maximum_curvature));
  }

  static std::size_t clamp_index(std::size_t value, std::size_t low, std::size_t high)
  {
    return std::max(low, std::min(value, high));
  }

  void set_sine_phase(LinePhase phase)
  {
    if (sine_phase_ == phase) {
      return;
    }
    sine_phase_ = phase;
    if (phase == LinePhase::BRAKE) {
      RCLCPP_INFO(get_logger(), "sine endpoint deceleration started");
    } else if (phase == LinePhase::SETTLE) {
      RCLCPP_INFO(get_logger(), "sine reference stopped; waiting for vehicle settle");
    }
  }

  void initialize_sine_progress()
  {
    sine_s_vehicle_ = project_continuous_waypoint(current_position());
    sine_s_ref_ = sine_s_vehicle_;
    const auto sample = sample_sine(sine_s_vehicle_, nullptr);
    sine_actual_speed_ = std::max(0.0, dot3(current_velocity(), sample.tangent));
    sine_speed_planner_.reset(std::min(sine_actual_speed_, line_speed_mps_));
    sine_s_dot_ref_ = sine_speed_planner_.speed();
    sine_s_ddot_ref_ = 0.0;
    sine_lead_ = 0.0;
    sine_remaining_distance_ = waypoint_total_length_ - sine_s_vehicle_;
    sine_braking_distance_ = 0.0;
    sine_target_speed_ = 0.0;
    sine_dynamic_lookahead_ = min_lookahead_distance_m_;
    sine_curve_speed_limit_mps_ = sine_curve_speed_limit(sine_s_ref_);
    sine_phase_ = LinePhase::ACCEL;
    sine_progress_active_ = true;
    RCLCPP_INFO(
      get_logger(),
      "sine closed-loop progress initialized; path %.2f m, initial curve limit %.2f m/s",
      waypoint_total_length_, sine_curve_speed_limit_mps_);
  }

  void set_sine_reference(double progress, double speed, double acceleration)
  {
    Vec3 curvature_vector{};
    const auto sample = sample_sine(progress, &curvature_vector);
    current_setpoint_ = sample.position;
    current_velocity_ff_ = sample.tangent * speed;
    current_acceleration_ff_ =
      sample.tangent * acceleration + curvature_vector * (speed * speed);
    acceleration_feedforward_valid_ = true;
    current_yaw_ = configured_trajectory_yaw(sample.tangent, current_yaw_);
    have_setpoint_ = true;
  }

  void update_sine_progress_trajectory()
  {
    if (sine_phase_ == LinePhase::SETTLE) {
      set_sine_reference(waypoint_total_length_, 0.0, 0.0);
      waypoint_segment_end_ = waypoint_points_.back();
      waypoint_segment_yaw_ = current_yaw_;
      waypoint_index_ = waypoint_points_.size() - 1U;
      update_waypoint_endpoint_hold();
      return;
    }

    sine_s_vehicle_ = project_continuous_waypoint(current_position());
    const auto actual_sample = sample_sine(sine_s_vehicle_, nullptr);
    sine_actual_speed_ = std::max(0.0, dot3(current_velocity(), actual_sample.tangent));
    sine_remaining_distance_ = std::max(0.0, waypoint_total_length_ - sine_s_vehicle_);
    sine_braking_distance_ =
      sine_actual_speed_ * sine_actual_speed_ / (2.0 * max_deceleration_mps2_) +
      sine_actual_speed_ * response_delay_s_ + stopping_safety_margin_m_;
    const double reference_remaining =
      std::max(0.0, waypoint_total_length_ - sine_s_ref_);
    const double planner_stopping_distance = sine_speed_planner_.stopping_distance(
      max_deceleration_mps2_, max_jerk_mps3_);
    const double reference_stop_buffer =
      0.02 + 2.0 * sine_speed_planner_.speed() * control_dt_s_;
    const double reference_braking_distance =
      planner_stopping_distance + reference_stop_buffer;

    sine_curve_speed_limit_mps_ = sine_curve_speed_limit(sine_s_ref_);
    const bool cruise_reached =
      sine_speed_planner_.speed() >= 0.98 * sine_curve_speed_limit_mps_;
    if (sine_phase_ != LinePhase::BRAKE &&
      (sine_remaining_distance_ <= sine_braking_distance_ ||
      reference_remaining <= reference_braking_distance))
    {
      set_sine_phase(LinePhase::BRAKE);
    } else if (sine_phase_ == LinePhase::ACCEL && cruise_reached) {
      set_sine_phase(LinePhase::CRUISE);
    }

    double nominal_target_speed = sine_curve_speed_limit_mps_;
    if (sine_phase_ == LinePhase::BRAKE) {
      // Actual stopping distance starts braking early. Reference stopping distance then places
      // the jerk-limited reference stop at the geometric endpoint, without an early stop/restart.
      if (reference_remaining <= reference_braking_distance) {
        nominal_target_speed = 0.0;
      }
      if (nominal_target_speed <= kSmall &&
        sine_actual_speed_ <= endpoint_speed_tolerance_mps_ &&
        sine_remaining_distance_ <=
        stopping_safety_margin_m_ + endpoint_position_tolerance_m_ &&
        reference_remaining > 0.02)
      {
        nominal_target_speed = std::min(
          0.15,
          std::sqrt(2.0 * max_deceleration_mps2_ * (reference_remaining - 0.02)));
      }
    }

    const auto governed = PathProgressGovernor::govern(
      nominal_target_speed, sine_s_ref_, sine_s_vehicle_, sine_actual_speed_,
      lookahead_time_s_, min_lookahead_distance_m_, max_lookahead_distance_m_,
      max_reference_lead_distance_m_);
    sine_target_speed_ = governed.target_speed;
    sine_dynamic_lookahead_ = governed.lookahead;

    const double previous_speed = sine_s_dot_ref_;
    const auto speed_step = sine_speed_planner_.update(
      sine_target_speed_, control_dt_s_, max_acceleration_mps2_, max_deceleration_mps2_,
      max_jerk_mps3_);
    const double hard_progress_limit = std::min(
      waypoint_total_length_, sine_s_vehicle_ + max_reference_lead_distance_m_);
    const double unrestricted_progress = sine_s_ref_ + speed_step.distance;
    const double next_progress = std::min(
      unrestricted_progress, std::max(sine_s_ref_, hard_progress_limit));
    sine_s_dot_ref_ = (next_progress - sine_s_ref_) / control_dt_s_;
    sine_s_ddot_ref_ = clamp(
      (sine_s_dot_ref_ - previous_speed) / control_dt_s_,
      -max_deceleration_mps2_, max_acceleration_mps2_);
    if (next_progress + kSmall < unrestricted_progress) {
      sine_speed_planner_.synchronize(sine_s_dot_ref_, sine_s_ddot_ref_);
    }
    sine_s_ref_ = next_progress;
    sine_lead_ = sine_s_ref_ - sine_s_vehicle_;

    if (reference_remaining <= 0.02 && sine_speed_planner_.speed() <= 0.05) {
      sine_s_ref_ = waypoint_total_length_;
      sine_s_dot_ref_ = 0.0;
      sine_s_ddot_ref_ = 0.0;
      sine_lead_ = sine_s_ref_ - sine_s_vehicle_;
      set_sine_phase(LinePhase::SETTLE);
      return;
    }
    set_sine_reference(sine_s_ref_, sine_s_dot_ref_, sine_s_ddot_ref_);
  }

  double circle_raw_angle(const Vec3 & position) const
  {
    const auto relative = position - circle_center_;
    const double forward = dot3(relative, circle_forward_unit_);
    const double right = dot3(relative, circle_right_unit_);
    return std::atan2(forward, -circle_lateral_sign_ * right);
  }

  Vec3 circle_tangent(double angle) const
  {
    return circle_forward_unit_ * std::cos(angle) +
           circle_right_unit_ * (circle_lateral_sign_ * std::sin(angle));
  }

  void set_circle_phase(LinePhase phase)
  {
    if (circle_phase_ == phase) {
      return;
    }
    circle_phase_ = phase;
    if (phase == LinePhase::BRAKE) {
      RCLCPP_INFO(get_logger(), "circle endpoint deceleration started");
    } else if (phase == LinePhase::SETTLE) {
      RCLCPP_INFO(get_logger(), "circle reference stopped; waiting for vehicle settle");
    }
  }

  void initialize_circle_progress()
  {
    circle_forward_unit_ = line_unit_vector();
    circle_right_unit_ = {-circle_forward_unit_.y, circle_forward_unit_.x, 0.0};
    circle_lateral_sign_ = circle_direction_ == "ccw" ? -1.0 : 1.0;
    circle_center_ = configured_start_point() +
      circle_right_unit_ * (circle_lateral_sign_ * circle_radius_);
    circle_previous_raw_angle_ = circle_raw_angle(current_position());
    circle_vehicle_angle_unwrapped_ = 0.0;
    circle_s_vehicle_ = 0.0;
    circle_s_ref_ = 0.0;
    circle_s_dot_ref_ = std::max(
      0.0, dot3(current_velocity(), circle_tangent(circle_previous_raw_angle_)));
    circle_speed_planner_.reset(std::min(circle_s_dot_ref_, line_speed_mps_));
    circle_s_ddot_ref_ = 0.0;
    circle_phase_ = LinePhase::ACCEL;
    circle_progress_active_ = true;
    circle_speed_limit_mps_ = std::min(
      line_speed_mps_, std::sqrt(max_safe_lateral_acceleration_mps2_ * circle_radius_));
    if (circle_speed_limit_mps_ + 1e-3 < line_speed_mps_) {
      RCLCPP_WARN(
        get_logger(),
        "circle speed limited from %.2f to %.2f m/s by lateral acceleration %.2f m/s^2",
        line_speed_mps_, circle_speed_limit_mps_, max_safe_lateral_acceleration_mps2_);
    }
  }

  void update_circle_progress_trajectory()
  {
    if (circle_phase_ == LinePhase::SETTLE) {
      current_setpoint_ = waypoint_points_.back();
      current_velocity_ff_ = {0.0, 0.0, 0.0};
      current_acceleration_ff_ = {0.0, 0.0, 0.0};
      acceleration_feedforward_valid_ = true;
      have_setpoint_ = true;
      waypoint_segment_end_ = waypoint_points_.back();
      waypoint_segment_yaw_ = current_yaw_;
      waypoint_index_ = waypoint_points_.size() - 1U;
      update_waypoint_endpoint_hold();
      return;
    }

    const double raw_angle = circle_raw_angle(current_position());
    circle_vehicle_angle_unwrapped_ += wrap_pi(raw_angle - circle_previous_raw_angle_);
    circle_previous_raw_angle_ = raw_angle;
    const double total_length = waypoint_total_length_;
    const double raw_vehicle_progress =
      std::max(0.0, circle_vehicle_angle_unwrapped_ * circle_radius_);
    circle_s_vehicle_ = continuous_trajectory_ ?
      raw_vehicle_progress : clamp(raw_vehicle_progress, 0.0, total_length);
    const auto actual_tangent = circle_tangent(circle_vehicle_angle_unwrapped_);
    circle_actual_speed_ = std::max(0.0, dot3(current_velocity(), actual_tangent));
    circle_remaining_distance_ = continuous_trajectory_ ?
      std::numeric_limits<double>::infinity() :
      std::max(0.0, total_length - raw_vehicle_progress);
    circle_braking_distance_ =
      circle_actual_speed_ * circle_actual_speed_ / (2.0 * max_deceleration_mps2_) +
      circle_actual_speed_ * response_delay_s_ + stopping_safety_margin_m_;

    double nominal_target_speed = circle_speed_limit_mps_;
    const double reference_remaining = continuous_trajectory_ ?
      std::numeric_limits<double>::infinity() : std::max(0.0, total_length - circle_s_ref_);
    if (!continuous_trajectory_) {
      const double planner_stopping_distance = circle_speed_planner_.stopping_distance(
        max_deceleration_mps2_, max_jerk_mps3_);
      const double reference_stop_buffer =
        0.02 + 2.0 * circle_speed_planner_.speed() * control_dt_s_;
      const double reference_braking_distance =
        planner_stopping_distance + reference_stop_buffer;
      const bool circle_cruise_reached =
        circle_speed_planner_.speed() >= 0.98 * circle_speed_limit_mps_;
      if (circle_phase_ != LinePhase::BRAKE &&
        (circle_remaining_distance_ <= circle_braking_distance_ ||
        reference_remaining <= reference_braking_distance))
      {
        set_circle_phase(LinePhase::BRAKE);
      } else if (circle_phase_ == LinePhase::ACCEL && circle_cruise_reached) {
        set_circle_phase(LinePhase::CRUISE);
      }

      if (circle_phase_ == LinePhase::BRAKE) {
        if (reference_remaining <= reference_braking_distance) {
          nominal_target_speed = 0.0;
        }
        if (nominal_target_speed <= kSmall &&
          circle_actual_speed_ <= endpoint_speed_tolerance_mps_ &&
          circle_remaining_distance_ <=
          stopping_safety_margin_m_ + endpoint_position_tolerance_m_ &&
          reference_remaining > 0.02)
        {
          nominal_target_speed = std::min(
            0.15,
            std::sqrt(2.0 * max_deceleration_mps2_ * (reference_remaining - 0.02)));
        }
      }
    }

    const auto governed = PathProgressGovernor::govern(
      nominal_target_speed, circle_s_ref_, circle_s_vehicle_, circle_actual_speed_,
      lookahead_time_s_, min_lookahead_distance_m_, max_lookahead_distance_m_,
      max_reference_lead_distance_m_);
    circle_target_speed_ = governed.target_speed;
    circle_dynamic_lookahead_ = governed.lookahead;

    const double previous_speed = circle_s_dot_ref_;
    const auto speed_step = circle_speed_planner_.update(
      circle_target_speed_, control_dt_s_, max_acceleration_mps2_, max_deceleration_mps2_,
      max_jerk_mps3_);
    double hard_progress_limit = circle_s_vehicle_ + max_reference_lead_distance_m_;
    if (!continuous_trajectory_) {
      hard_progress_limit = std::min(hard_progress_limit, total_length);
    }
    const double unrestricted_progress = circle_s_ref_ + speed_step.distance;
    const double next_progress = std::min(
      unrestricted_progress, std::max(circle_s_ref_, hard_progress_limit));
    circle_s_dot_ref_ = (next_progress - circle_s_ref_) / control_dt_s_;
    circle_s_ddot_ref_ = clamp(
      (circle_s_dot_ref_ - previous_speed) / control_dt_s_,
      -max_deceleration_mps2_, max_acceleration_mps2_);
    if (next_progress + kSmall < unrestricted_progress) {
      circle_speed_planner_.synchronize(circle_s_dot_ref_, circle_s_ddot_ref_);
    }
    circle_s_ref_ = next_progress;
    circle_lead_ = circle_s_ref_ - circle_s_vehicle_;

    if (!continuous_trajectory_ && reference_remaining <= 0.02 &&
      circle_speed_planner_.speed() <= 0.05)
    {
      circle_s_ref_ = total_length;
      circle_s_dot_ref_ = 0.0;
      circle_s_ddot_ref_ = 0.0;
      set_circle_phase(LinePhase::SETTLE);
      return;
    }

    const double sample_distance = continuous_trajectory_ ?
      std::fmod(circle_s_ref_, total_length) : circle_s_ref_;
    Vec3 tangent{};
    current_setpoint_ = interpolate_continuous_waypoint(sample_distance, &tangent);
    current_velocity_ff_ = tangent * circle_s_dot_ref_;
    auto inward = circle_center_ - current_setpoint_;
    inward.z = 0.0;
    const double inward_norm = norm3(inward);
    if (inward_norm > kSmall) {
      inward = inward * (1.0 / inward_norm);
    }
    current_acceleration_ff_ =
      tangent * circle_s_ddot_ref_ +
      inward * (circle_s_dot_ref_ * circle_s_dot_ref_ / circle_radius_);
    acceleration_feedforward_valid_ = true;
    current_yaw_ = configured_trajectory_yaw(tangent, current_yaw_);
    have_setpoint_ = true;
  }

  void update_continuous_waypoint_trajectory()
  {
    if (trajectory_type_ == "circle" && circle_progress_active_) {
      update_circle_progress_trajectory();
      return;
    }
    if (trajectory_type_ == "sine" && sine_progress_active_) {
      update_sine_progress_trajectory();
      return;
    }
    const double elapsed = seconds_between(waypoint_continuous_start_time_, steady_now());

    double distance_along = waypoint_total_length_;
    double speed_along = 0.0;
    bool complete = false;

    if (continuous_trajectory_ && trajectory_type_ == "circle") {
      if (elapsed < accel_time_s_) {
        const double u = elapsed / accel_time_s_;
        distance_along = line_speed_mps_ * accel_time_s_ * smoothstep_integral(u);
        speed_along = line_speed_mps_ * smoothstep(u);
      } else {
        distance_along =
          0.5 * line_speed_mps_ * accel_time_s_ + line_speed_mps_ * (elapsed - accel_time_s_);
        speed_along = line_speed_mps_;
      }
      distance_along = std::fmod(distance_along, waypoint_total_length_);
      if (distance_along < 0.0) {
        distance_along += waypoint_total_length_;
      }
    } else if (elapsed < waypoint_profile_.total_time) {
      if (elapsed < accel_time_s_) {
        const double u = elapsed / accel_time_s_;
        distance_along = waypoint_profile_.peak_speed * accel_time_s_ * smoothstep_integral(u);
        speed_along = waypoint_profile_.peak_speed * smoothstep(u);
      } else if (elapsed < accel_time_s_ + waypoint_profile_.cruise_time) {
        const double cruise_elapsed = elapsed - accel_time_s_;
        distance_along = waypoint_profile_.accel_distance + waypoint_profile_.peak_speed *
          cruise_elapsed;
        speed_along = waypoint_profile_.peak_speed;
      } else {
        const double decel_elapsed = elapsed - accel_time_s_ - waypoint_profile_.cruise_time;
        const double u = decel_elapsed / decel_time_s_;
        distance_along =
          waypoint_profile_.accel_distance + waypoint_profile_.cruise_distance +
          waypoint_profile_.peak_speed * decel_time_s_ *
          (u - u * u * u + 0.5 * u * u * u * u);
        speed_along = waypoint_profile_.peak_speed * (1.0 - smoothstep(u));
      }
    } else {
      complete = true;
      distance_along = waypoint_total_length_;
      speed_along = 0.0;
    }

    Vec3 unit{};
    current_setpoint_ = interpolate_continuous_waypoint(distance_along, &unit);
    current_velocity_ff_ = unit * speed_along;
    current_yaw_ = configured_trajectory_yaw(unit, current_yaw_);
    have_setpoint_ = true;

    if (complete) {
      waypoint_segment_end_ = waypoint_points_.back();
      waypoint_segment_yaw_ = current_yaw_;
      waypoint_index_ = waypoint_points_.size() - 1U;
      update_waypoint_endpoint_hold();
    }
  }

  void set_square_edge_reference(double progress, double speed, double acceleration)
  {
    const auto sample = PathGeometry::sample_line(
      waypoint_segment_start_, waypoint_segment_unit_, waypoint_segment_length_, progress);
    current_setpoint_ = sample.position;
    current_velocity_ff_ = sample.tangent * speed;
    current_acceleration_ff_ = sample.tangent * acceleration;
    acceleration_feedforward_valid_ = true;
    current_yaw_ = configured_trajectory_yaw(sample.tangent, waypoint_segment_yaw_);
    have_setpoint_ = true;
  }

  void hold_square_vertex(double yaw)
  {
    current_setpoint_ = waypoint_segment_end_;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_acceleration_ff_ = {0.0, 0.0, 0.0};
    acceleration_feedforward_valid_ = true;
    current_yaw_ = yaw_mode_ == "fixed" ? mission_start_yaw_rad() : yaw;
    have_setpoint_ = true;
  }

  void set_square_phase(LinePhase phase)
  {
    if (square_phase_ == phase) {
      return;
    }
    square_phase_ = phase;
    if (phase == LinePhase::BRAKE) {
      RCLCPP_INFO(get_logger(), "square edge %zu deceleration started", waypoint_index_);
    } else if (phase == LinePhase::SETTLE) {
      RCLCPP_INFO(get_logger(), "square edge %zu reference stopped", waypoint_index_);
    }
  }

  void start_square_edge(const std::string & log_reason)
  {
    if (waypoint_index_ >= waypoint_points_.size()) {
      finish_waypoint_trajectory();
      return;
    }

    waypoint_segment_start_ = waypoint_points_[waypoint_index_ - 1U];
    waypoint_segment_end_ = waypoint_points_[waypoint_index_];
    const auto delta = waypoint_segment_end_ - waypoint_segment_start_;
    waypoint_segment_length_ = norm3(delta);
    if (waypoint_segment_length_ <= kSmall) {
      transition_to(State::FAULT, "square edge has zero length");
      return;
    }
    waypoint_segment_unit_ = delta * (1.0 / waypoint_segment_length_);
    waypoint_segment_yaw_ = yaw_from_horizontal_vector(delta, current_yaw_);
    waypoint_segment_active_ = true;
    square_progress_active_ = true;
    square_state_ = SquareState::EDGE;
    square_phase_ = LinePhase::ACCEL;
    square_vertex_settle_started_ = false;
    square_yaw_settle_started_ = false;

    square_s_vehicle_ = PathGeometry::project_line(
      current_position(), waypoint_segment_start_, waypoint_segment_unit_,
      waypoint_segment_length_);
    square_s_ref_ = square_s_vehicle_;
    square_actual_speed_ = std::max(0.0, dot3(current_velocity(), waypoint_segment_unit_));
    square_speed_planner_.reset(std::min(square_actual_speed_, line_speed_mps_));
    square_s_dot_ref_ = square_speed_planner_.speed();
    square_s_ddot_ref_ = 0.0;
    square_lead_ = square_s_ref_ - square_s_vehicle_;
    square_remaining_distance_ = waypoint_segment_length_ - square_s_vehicle_;
    square_braking_distance_ = 0.0;
    square_target_speed_ = line_speed_mps_;
    square_dynamic_lookahead_ = min_lookahead_distance_m_;
    set_square_edge_reference(square_s_ref_, square_s_dot_ref_, 0.0);

    RCLCPP_INFO(
      get_logger(), "%s; square edge %zu target [%.2f, %.2f, %.2f]",
      log_reason.empty() ? "edge acceleration started" : log_reason.c_str(),
      waypoint_index_, waypoint_segment_end_.x, waypoint_segment_end_.y,
      waypoint_segment_end_.z);
  }

  void start_square_yaw_turn()
  {
    const auto next_delta = waypoint_points_[waypoint_index_ + 1U] - waypoint_segment_end_;
    square_turn_start_yaw_ = current_heading();
    square_turn_target_yaw_ = yaw_from_horizontal_vector(next_delta, current_yaw_);
    const double yaw_delta = std::abs(wrap_pi(square_turn_target_yaw_ - square_turn_start_yaw_));
    square_turn_duration_s_ = std::max(0.1, yaw_delta / deg_to_rad(vertex_yaw_rate_dps_));
    square_turn_start_time_ = steady_now();
    square_state_ = SquareState::YAW_TURN;
    hold_square_vertex(square_turn_start_yaw_);
    RCLCPP_INFO(
      get_logger(), "square vertex turn started; target yaw %.1f deg, duration %.2f s",
      rad_to_deg(square_turn_target_yaw_), square_turn_duration_s_);
  }

  void update_square_vertex_settle()
  {
    hold_square_vertex(waypoint_segment_yaw_);
    if (!local_pose_ready()) {
      return;
    }

    const auto status = endpoint_status(
      waypoint_segment_end_, vertex_position_tolerance_m_, vertex_speed_tolerance_mps_,
      square_vertex_settle_started_);
    const bool vertex_settle_complete =
      seconds_between(square_vertex_settle_start_time_, steady_now()) >=
      endpoint_settle_time_s_ + line_endpoint_hold_time_s_;
    if (status.within_limits) {
      if (!square_vertex_settle_started_) {
        square_vertex_settle_started_ = true;
        square_vertex_settle_start_time_ = steady_now();
        RCLCPP_INFO(get_logger(), "square vertex %zu reached; waiting stationary settle",
          waypoint_index_);
      } else if (vertex_settle_complete) {
        if (waypoint_index_ + 1U >= waypoint_points_.size()) {
          square_progress_active_ = false;
          finish_waypoint_trajectory();
        } else if (yaw_mode_ == "fixed") {
          ++waypoint_index_;
          start_square_edge("fixed-yaw square vertex complete");
        } else {
          start_square_yaw_turn();
        }
      }
    } else {
      square_vertex_settle_started_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for square vertex settle: horizontal %.3f m, altitude %.3f m, "
        "horizontal speed %.3f m/s, vertical speed %.3f m/s",
        status.horizontal_position_error, status.altitude_error,
        status.horizontal_speed, status.vertical_speed);
    }
  }

  void update_square_yaw_turn()
  {
    const double elapsed = seconds_between(square_turn_start_time_, steady_now());
    const double u = clamp(elapsed / square_turn_duration_s_, 0.0, 1.0);
    const double yaw_delta = wrap_pi(square_turn_target_yaw_ - square_turn_start_yaw_);
    hold_square_vertex(wrap_pi(square_turn_start_yaw_ + yaw_delta * smootherstep(u)));
    if (u >= 1.0) {
      square_state_ = SquareState::YAW_SETTLE;
      square_yaw_settle_started_ = false;
      RCLCPP_INFO(get_logger(), "square vertex yaw reached; waiting stationary yaw settle");
    }
  }

  void update_square_yaw_settle()
  {
    hold_square_vertex(square_turn_target_yaw_);
    if (!local_pose_ready()) {
      return;
    }

    const auto status = endpoint_status(
      waypoint_segment_end_, vertex_position_tolerance_m_, vertex_speed_tolerance_mps_, true);
    const double yaw_error_deg =
      std::abs(rad_to_deg(wrap_pi(current_heading() - square_turn_target_yaw_)));
    const bool yaw_settle_complete =
      seconds_between(square_yaw_settle_start_time_, steady_now()) >=
      vertex_yaw_settle_time_s_;
    if (status.within_limits && yaw_error_deg <= vertex_yaw_tolerance_deg_) {
      if (!square_yaw_settle_started_) {
        square_yaw_settle_started_ = true;
        square_yaw_settle_start_time_ = steady_now();
      } else if (yaw_settle_complete) {
        ++waypoint_index_;
        start_square_edge("square vertex turn complete");
      }
    } else {
      square_yaw_settle_started_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for square yaw settle: horizontal %.3f m, altitude %.3f m, "
        "horizontal speed %.3f m/s, yaw %.2f deg",
        status.horizontal_position_error, status.altitude_error,
        status.horizontal_speed, yaw_error_deg);
    }
  }

  void update_square_edge()
  {
    const double raw_vehicle_progress = PathGeometry::project_line_unclamped(
      current_position(), waypoint_segment_start_, waypoint_segment_unit_);
    square_s_vehicle_ = clamp(raw_vehicle_progress, 0.0, waypoint_segment_length_);
    square_actual_speed_ = std::max(0.0, dot3(current_velocity(), waypoint_segment_unit_));
    square_remaining_distance_ = std::max(0.0, waypoint_segment_length_ - raw_vehicle_progress);
    square_braking_distance_ =
      square_actual_speed_ * square_actual_speed_ / (2.0 * max_deceleration_mps2_) +
      square_actual_speed_ * response_delay_s_ + stopping_safety_margin_m_;

    const double reference_remaining = std::max(0.0, waypoint_segment_length_ - square_s_ref_);
    const double planner_stopping_distance = square_speed_planner_.stopping_distance(
      max_deceleration_mps2_, max_jerk_mps3_);
    const double reference_stop_buffer =
      0.02 + 2.0 * square_speed_planner_.speed() * control_dt_s_;
    const double reference_braking_distance =
      planner_stopping_distance + reference_stop_buffer;
    const bool square_cruise_reached =
      square_speed_planner_.speed() >= 0.98 * line_speed_mps_;
    if (square_phase_ != LinePhase::BRAKE &&
      (square_remaining_distance_ <= square_braking_distance_ ||
      reference_remaining <= reference_braking_distance))
    {
      set_square_phase(LinePhase::BRAKE);
    } else if (square_phase_ == LinePhase::ACCEL && square_cruise_reached) {
      set_square_phase(LinePhase::CRUISE);
    }

    double nominal_target_speed = line_speed_mps_;
    if (square_phase_ == LinePhase::BRAKE) {
      if (reference_remaining <= reference_braking_distance) {
        nominal_target_speed = 0.0;
      }
      if (nominal_target_speed <= kSmall &&
        square_actual_speed_ <= endpoint_speed_tolerance_mps_ &&
        square_remaining_distance_ <= stopping_safety_margin_m_ + endpoint_position_tolerance_m_ &&
        reference_remaining > 0.02)
      {
        nominal_target_speed = std::min(
          0.15,
          std::sqrt(2.0 * max_deceleration_mps2_ * (reference_remaining - 0.02)));
      }
    }

    const auto governed = PathProgressGovernor::govern(
      nominal_target_speed, square_s_ref_, square_s_vehicle_, square_actual_speed_,
      lookahead_time_s_, min_lookahead_distance_m_, max_lookahead_distance_m_,
      max_reference_lead_distance_m_);
    square_target_speed_ = governed.target_speed;
    square_dynamic_lookahead_ = governed.lookahead;

    const double previous_speed = square_s_dot_ref_;
    const auto speed_step = square_speed_planner_.update(
      square_target_speed_, control_dt_s_, max_acceleration_mps2_, max_deceleration_mps2_,
      max_jerk_mps3_);
    const double hard_progress_limit = std::min(
      waypoint_segment_length_, square_s_vehicle_ + max_reference_lead_distance_m_);
    const double unrestricted_progress = square_s_ref_ + speed_step.distance;
    const double next_progress = std::min(
      unrestricted_progress, std::max(square_s_ref_, hard_progress_limit));
    square_s_dot_ref_ = (next_progress - square_s_ref_) / control_dt_s_;
    square_s_ddot_ref_ = clamp(
      (square_s_dot_ref_ - previous_speed) / control_dt_s_,
      -max_deceleration_mps2_, max_acceleration_mps2_);
    if (next_progress + kSmall < unrestricted_progress) {
      square_speed_planner_.synchronize(square_s_dot_ref_, square_s_ddot_ref_);
    }
    square_s_ref_ = next_progress;
    square_lead_ = square_s_ref_ - square_s_vehicle_;

    if (reference_remaining <= 0.02 && square_speed_planner_.speed() <= 0.05) {
      square_s_ref_ = waypoint_segment_length_;
      square_s_dot_ref_ = 0.0;
      square_s_ddot_ref_ = 0.0;
      set_square_phase(LinePhase::SETTLE);
      square_state_ = SquareState::VERTEX_SETTLE;
      square_vertex_settle_started_ = false;
      update_square_vertex_settle();
      return;
    }
    set_square_edge_reference(square_s_ref_, square_s_dot_ref_, square_s_ddot_ref_);
  }

  void update_square_trajectory()
  {
    switch (square_state_) {
      case SquareState::EDGE:
        update_square_edge();
        break;
      case SquareState::VERTEX_SETTLE:
        update_square_vertex_settle();
        break;
      case SquareState::YAW_TURN:
        update_square_yaw_turn();
        break;
      case SquareState::YAW_SETTLE:
        update_square_yaw_settle();
        break;
      case SquareState::IDLE:
        start_square_edge("square trajectory started");
        break;
    }
  }

  void start_next_waypoint_segment(const std::string & log_reason)
  {
    if (trajectory_type_ == "square") {
      start_square_edge(log_reason);
      return;
    }
    if (waypoint_index_ >= waypoint_points_.size()) {
      finish_waypoint_trajectory();
      return;
    }
    waypoint_segment_start_ = waypoint_points_[waypoint_index_ - 1U];
    waypoint_segment_end_ = waypoint_points_[waypoint_index_];
    const auto delta = waypoint_segment_end_ - waypoint_segment_start_;
    waypoint_segment_length_ = norm3(delta);
    waypoint_segment_unit_ = waypoint_segment_length_ > kSmall ?
      delta * (1.0 / waypoint_segment_length_) : Vec3{};
    waypoint_segment_yaw_ = yaw_from_horizontal_vector(delta, current_yaw_);
    waypoint_profile_ = make_leg_profile(std::max(waypoint_segment_length_, kSmall));
    waypoint_segment_start_time_ = steady_now();
    waypoint_segment_active_ = true;
    waypoint_endpoint_settle_started_ = false;
    current_setpoint_ = waypoint_segment_start_;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = yaw_mode_ == "fixed" ? mission_start_yaw_rad() : waypoint_segment_yaw_;
    have_setpoint_ = true;
    RCLCPP_INFO(
      get_logger(), "%s; segment target %zu [%.2f, %.2f, %.2f]",
      log_reason.empty() ? "segment started" : log_reason.c_str(),
      waypoint_index_, waypoint_segment_end_.x, waypoint_segment_end_.y, waypoint_segment_end_.z);
  }

  void update_waypoint_trajectory()
  {
    if (trajectory_type_ == "square" && square_progress_active_) {
      update_square_trajectory();
      return;
    }
    if (waypoint_continuous_active_) {
      update_continuous_waypoint_trajectory();
      return;
    }

    if (!waypoint_segment_active_) {
      start_next_waypoint_segment("");
      return;
    }

    const double elapsed = seconds_between(waypoint_segment_start_time_, steady_now());
    if (elapsed >= waypoint_profile_.total_time) {
      update_waypoint_endpoint_hold();
      return;
    }

    double distance_along = waypoint_segment_length_;
    double speed_along = 0.0;
    if (elapsed < accel_time_s_) {
      const double u = elapsed / accel_time_s_;
      distance_along = waypoint_profile_.peak_speed * accel_time_s_ * smoothstep_integral(u);
      speed_along = waypoint_profile_.peak_speed * smoothstep(u);
    } else if (elapsed < accel_time_s_ + waypoint_profile_.cruise_time) {
      const double cruise_elapsed = elapsed - accel_time_s_;
      distance_along = waypoint_profile_.accel_distance + waypoint_profile_.peak_speed *
        cruise_elapsed;
      speed_along = waypoint_profile_.peak_speed;
    } else {
      const double decel_elapsed = elapsed - accel_time_s_ - waypoint_profile_.cruise_time;
      const double u = decel_elapsed / decel_time_s_;
      distance_along =
        waypoint_profile_.accel_distance + waypoint_profile_.cruise_distance +
        waypoint_profile_.peak_speed * decel_time_s_ *
        (u - u * u * u + 0.5 * u * u * u * u);
      speed_along = waypoint_profile_.peak_speed * (1.0 - smoothstep(u));
    }

    current_setpoint_ =
      waypoint_segment_start_ + waypoint_segment_unit_ * clamp(distance_along, 0.0,
      waypoint_segment_length_);
    current_velocity_ff_ = waypoint_segment_unit_ * speed_along;
    current_yaw_ = configured_trajectory_yaw(waypoint_segment_unit_, waypoint_segment_yaw_);
    have_setpoint_ = true;
  }

  void update_waypoint_endpoint_hold()
  {
    current_setpoint_ = waypoint_segment_end_;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = yaw_mode_ == "fixed" ? mission_start_yaw_rad() : waypoint_segment_yaw_;
    have_setpoint_ = true;

    if (waypoint_pass_through_segment()) {
      ++waypoint_index_;
      waypoint_segment_active_ = false;
      return;
    }
    if (!local_pose_ready()) {
      return;
    }
    const auto status = endpoint_status(
      waypoint_segment_end_, endpoint_position_tolerance_m_, endpoint_speed_tolerance_mps_,
      waypoint_endpoint_settle_started_);
    const bool waypoint_settle_complete =
      seconds_between(waypoint_endpoint_settle_start_time_, steady_now()) >=
      endpoint_settle_time_s_ + line_endpoint_hold_time_s_;
    if (status.within_limits) {
      if (!waypoint_endpoint_settle_started_) {
        waypoint_endpoint_settle_started_ = true;
        waypoint_endpoint_settle_start_time_ = steady_now();
        RCLCPP_INFO(get_logger(), "waypoint %zu reached", waypoint_index_);
      } else if (waypoint_settle_complete) {
        ++waypoint_index_;
        waypoint_segment_active_ = false;
        if (waypoint_index_ >= waypoint_points_.size()) {
          finish_waypoint_trajectory();
        }
      }
    } else {
      waypoint_endpoint_settle_started_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for waypoint settle: horizontal %.3f m, altitude %.3f m, "
        "horizontal speed %.3f m/s, vertical speed %.3f m/s",
        status.horizontal_position_error, status.altitude_error,
        status.horizontal_speed, status.vertical_speed);
    }
  }

  void finish_waypoint_trajectory()
  {
    if (continuous_trajectory_ && trajectory_type_ == "square") {
      RCLCPP_INFO(get_logger(), "continuous trajectory loop restarting");
      start_waypoint_trajectory();
      return;
    }
    waypoint_complete_ = true;
    sine_progress_active_ = false;
    circle_progress_active_ = false;
    square_progress_active_ = false;
    square_state_ = SquareState::IDLE;
    waypoint_segment_active_ = false;
    waypoint_continuous_active_ = false;
    mission_complete_position_ =
      waypoint_points_.empty() ? configured_start_point() : waypoint_points_.back();
    mission_complete_yaw_ = current_yaw_;
    current_setpoint_ = mission_complete_position_;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = mission_complete_yaw_;
    transition_to(State::MISSION_COMPLETE_HOLD, "trajectory complete; holding final point");
  }

  void start_next_line_leg(const std::string & log_reason)
  {
    last_line_phase_ = LinePhase::IDLE;
    line_leg_active_ = true;
    line_turn_active_ = false;
    line_endpoint_settle_started_ = false;
    line_endpoint_reached_logged_ = false;
    ++line_leg_index_;
    line_leg_origin_ = line_forward_leg_ ?
      line_start_position_ : line_start_position_ + line_unit_ * line_distance_active_;
    line_leg_unit_ = line_unit_ * (line_forward_leg_ ? 1.0 : -1.0);
    line_s_vehicle_ = PathGeometry::project_line(
      current_position(), line_leg_origin_, line_leg_unit_, line_distance_active_);
    line_s_ref_ = line_s_vehicle_;
    line_actual_speed_ = std::max(0.0, dot3(current_velocity(), line_leg_unit_));
    line_speed_planner_.reset(std::min(line_actual_speed_, line_speed_mps_));
    line_s_dot_ref_ = line_speed_planner_.speed();
    line_s_ddot_ref_ = 0.0;
    line_lead_ = line_s_ref_ - line_s_vehicle_;
    line_remaining_distance_ = line_distance_active_ - line_s_vehicle_;
    line_braking_distance_ = 0.0;
    line_target_speed_ = line_speed_mps_;
    line_dynamic_lookahead_ = min_lookahead_distance_m_;
    set_line_leg_reference(line_s_ref_, line_s_dot_ref_, 0.0);
    log_phase_once(LinePhase::ACCEL);
    if (!log_reason.empty()) {
      RCLCPP_INFO(
        get_logger(), "%s; leg %zu direction %s",
        log_reason.c_str(), line_leg_index_, line_forward_leg_ ? "start_to_end" : "end_to_start");
    } else {
      RCLCPP_INFO(
        get_logger(), "reverse acceleration started; leg %zu direction %s",
        line_leg_index_, line_forward_leg_ ? "start_to_end" : "end_to_start");
    }
  }

  void start_line_turn()
  {
    line_leg_active_ = false;
    line_turn_active_ = true;
    line_turn_settle_started_ = false;
    line_single_pass_ = false;
    line_distance_active_ = line_distance_m_;
    line_s_ref_ = 0.0;
    line_s_vehicle_ = 0.0;
    line_s_dot_ref_ = 0.0;
    line_s_ddot_ref_ = 0.0;
    line_speed_planner_.reset();
    line_turn_start_time_ = steady_now();
    line_turn_start_yaw_ = current_heading();
    line_turn_target_yaw_ = line_leg_yaw();
    const double yaw_delta = std::abs(wrap_pi(line_turn_target_yaw_ - line_turn_start_yaw_));
    line_turn_duration_s_ = std::max(0.1, yaw_delta / deg_to_rad(line_turn_yaw_rate_dps_));

    const double hold_scalar = line_forward_leg_ ? 0.0 : line_distance_active_;
    current_setpoint_ = line_start_position_ + line_unit_ * hold_scalar;
    current_velocity_ff_ = {0.0, 0.0, 0.0};
    current_yaw_ = line_turn_start_yaw_;
    have_setpoint_ = true;

    RCLCPP_INFO(
      get_logger(),
      "endpoint turn started; target yaw %.1f deg, duration %.2f s",
      rad_to_deg(line_turn_target_yaw_), line_turn_duration_s_);
  }

  void update_line_turn()
  {
    const double hold_scalar = line_forward_leg_ ? 0.0 : line_distance_active_;
    current_setpoint_ = line_start_position_ + line_unit_ * hold_scalar;
    current_velocity_ff_ = {0.0, 0.0, 0.0};

    const double elapsed = seconds_between(line_turn_start_time_, steady_now());
    const double u = clamp(elapsed / line_turn_duration_s_, 0.0, 1.0);
    const double s = smootherstep(u);
    const double yaw_delta = wrap_pi(line_turn_target_yaw_ - line_turn_start_yaw_);
    current_yaw_ = wrap_pi(line_turn_start_yaw_ + yaw_delta * s);
    have_setpoint_ = true;

    if (u < 1.0) {
      return;
    }

    current_yaw_ = line_turn_target_yaw_;
    if (!local_pose_ready()) {
      return;
    }

    const double yaw_error_deg =
      std::abs(rad_to_deg(wrap_pi(current_heading() - line_turn_target_yaw_)));
    const auto status = endpoint_status(
      line_start_position_ + line_unit_ * hold_scalar,
      endpoint_position_tolerance_m_, endpoint_speed_tolerance_mps_, true);
    const bool turn_settle_complete =
      seconds_between(line_turn_settle_start_time_, steady_now()) >= line_turn_settle_time_s_;
    if (yaw_error_deg <= line_turn_yaw_tolerance_deg_ && status.within_limits) {
      if (!line_turn_settle_started_) {
        line_turn_settle_started_ = true;
        line_turn_settle_start_time_ = steady_now();
        RCLCPP_INFO(get_logger(), "endpoint turn reached; waiting settle");
      } else if (turn_settle_complete) {
        line_turn_active_ = false;
        start_next_line_leg("turn complete; reverse acceleration started");
      }
    } else {
      line_turn_settle_started_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for endpoint turn: yaw %.2f deg, horizontal error %.3f m, "
        "horizontal speed %.3f m/s",
        yaw_error_deg, status.horizontal_position_error, status.horizontal_speed);
    }
  }

  void set_line_reference(double scalar_from_start, double scalar_speed)
  {
    current_setpoint_ = line_start_position_ + line_unit_ * scalar_from_start;
    current_velocity_ff_ = line_unit_ * scalar_speed;
    current_yaw_ = configured_trajectory_yaw(
      line_unit_ * (scalar_speed < 0.0 ? -1.0 : 1.0), line_leg_yaw());
    have_setpoint_ = true;
  }

  void set_line_leg_reference(double progress, double speed, double acceleration)
  {
    const auto sample = PathGeometry::sample_line(
      line_leg_origin_, line_leg_unit_, line_distance_active_, progress);
    current_setpoint_ = sample.position;
    current_velocity_ff_ = sample.tangent * speed;
    current_acceleration_ff_ = sample.tangent * acceleration;
    acceleration_feedforward_valid_ = true;
    current_yaw_ = configured_trajectory_yaw(sample.tangent, line_leg_yaw());
    have_setpoint_ = true;
  }

  void log_phase_once(LinePhase phase)
  {
    if (phase == last_line_phase_) {
      return;
    }
    last_line_phase_ = phase;
    if (phase == LinePhase::BRAKE) {
      RCLCPP_INFO(get_logger(), "endpoint deceleration started");
    } else if (phase == LinePhase::ACCEL && line_leg_index_ > 1) {
      RCLCPP_INFO(get_logger(), "reverse acceleration started");
    } else if (phase == LinePhase::SETTLE) {
      RCLCPP_INFO(get_logger(), "line reference stopped; waiting for vehicle settle");
    }
  }

  void update_line_trajectory()
  {
    if (line_turn_active_) {
      update_line_turn();
      return;
    }
    if (!line_leg_active_) {
      start_next_line_leg("");
      return;
    }

    if (last_line_phase_ == LinePhase::SETTLE) {
      update_line_endpoint_hold();
      return;
    }

    const double dt = control_dt_s_;
    const double raw_vehicle_progress = PathGeometry::project_line_unclamped(
      current_position(), line_leg_origin_, line_leg_unit_);
    line_s_vehicle_ = clamp(raw_vehicle_progress, 0.0, line_distance_active_);
    line_actual_speed_ = std::max(0.0, dot3(current_velocity(), line_leg_unit_));
    line_remaining_distance_ = std::max(0.0, line_distance_active_ - raw_vehicle_progress);
    line_braking_distance_ =
      line_actual_speed_ * line_actual_speed_ / (2.0 * max_deceleration_mps2_) +
      line_actual_speed_ * response_delay_s_ + stopping_safety_margin_m_;

    const double reference_remaining = std::max(0.0, line_distance_active_ - line_s_ref_);
    const double planner_stopping_distance = line_speed_planner_.stopping_distance(
      max_deceleration_mps2_, max_jerk_mps3_);
    const double reference_stop_buffer =
      0.02 + 2.0 * line_speed_planner_.speed() * control_dt_s_;
    const double reference_braking_distance =
      planner_stopping_distance + reference_stop_buffer;
    const bool line_cruise_reached =
      line_speed_planner_.speed() >= 0.98 * line_speed_mps_;

    if (last_line_phase_ != LinePhase::BRAKE &&
      (line_remaining_distance_ <= line_braking_distance_ ||
      reference_remaining <= reference_braking_distance))
    {
      log_phase_once(LinePhase::BRAKE);
    } else if (last_line_phase_ == LinePhase::ACCEL && line_cruise_reached) {
      log_phase_once(LinePhase::CRUISE);
    }

    double nominal_target_speed = line_speed_mps_;
    if (last_line_phase_ == LinePhase::BRAKE) {
      if (reference_remaining <= reference_braking_distance) {
        nominal_target_speed = 0.0;
      }

      // The safety margin can intentionally stop the vehicle just before the endpoint. Close the
      // final small gap with a jerk-limited crawl instead of jumping the position reference.
      if (nominal_target_speed <= kSmall &&
        line_actual_speed_ <= endpoint_speed_tolerance_mps_ &&
        line_remaining_distance_ <= stopping_safety_margin_m_ + endpoint_position_tolerance_m_ &&
        reference_remaining > 0.02)
      {
        nominal_target_speed = std::min(
          0.15,
          std::sqrt(2.0 * max_deceleration_mps2_ * (reference_remaining - 0.02)));
      }
    }

    const auto governed = PathProgressGovernor::govern(
      nominal_target_speed, line_s_ref_, line_s_vehicle_, line_actual_speed_,
      lookahead_time_s_, min_lookahead_distance_m_, max_lookahead_distance_m_,
      max_reference_lead_distance_m_);
    line_target_speed_ = governed.target_speed;
    line_dynamic_lookahead_ = governed.lookahead;

    const double previous_reference_speed = line_s_dot_ref_;
    const auto speed_step = line_speed_planner_.update(
      line_target_speed_, dt, max_acceleration_mps2_, max_deceleration_mps2_, max_jerk_mps3_);
    const double hard_progress_limit = std::min(
      line_distance_active_, line_s_vehicle_ + max_reference_lead_distance_m_);
    const double unrestricted_progress = line_s_ref_ + speed_step.distance;
    const double next_progress = std::min(
      unrestricted_progress, std::max(line_s_ref_, hard_progress_limit));
    line_s_dot_ref_ = dt > kSmall ? (next_progress - line_s_ref_) / dt : 0.0;
    line_s_ddot_ref_ = dt > kSmall ? clamp(
      (line_s_dot_ref_ - previous_reference_speed) / dt,
      -max_deceleration_mps2_, max_acceleration_mps2_) : 0.0;
    if (next_progress + kSmall < unrestricted_progress) {
      line_speed_planner_.synchronize(line_s_dot_ref_, line_s_ddot_ref_);
    }
    line_s_ref_ = next_progress;
    line_lead_ = line_s_ref_ - line_s_vehicle_;

    if (reference_remaining <= 0.02 && line_speed_planner_.speed() <= 0.05) {
      line_s_ref_ = line_distance_active_;
      line_s_dot_ref_ = 0.0;
      line_s_ddot_ref_ = 0.0;
      line_lead_ = line_s_ref_ - line_s_vehicle_;
      log_phase_once(LinePhase::SETTLE);
      update_line_endpoint_hold();
      return;
    }

    set_line_leg_reference(line_s_ref_, line_s_dot_ref_, line_s_ddot_ref_);
  }

  void finish_line_leg()
  {
    const double endpoint_scalar = line_forward_leg_ ? line_distance_active_ : 0.0;
    set_line_reference(endpoint_scalar, 0.0);

    if (line_single_pass_) {
      mission_complete_position_ = line_start_position_ + line_unit_ * line_distance_active_;
      mission_complete_yaw_ = current_yaw_;
      current_setpoint_ = mission_complete_position_;
      current_velocity_ff_ = {0.0, 0.0, 0.0};
      transition_to(State::MISSION_COMPLETE_HOLD, "trajectory complete; holding final point");
      return;
    }

    if (!line_forward_leg_) {
      ++completed_reciprocations_;
      RCLCPP_INFO(
        get_logger(), "completed reciprocation %d%s",
        completed_reciprocations_,
        line_reciprocation_count_ == 0 ? " (infinite mode)" : "");
      if (line_reciprocation_count_ > 0 &&
        completed_reciprocations_ >= line_reciprocation_count_)
      {
        mission_complete_position_ = line_start_position_;
        mission_complete_yaw_ = current_yaw_;
        current_setpoint_ = mission_complete_position_;
        current_velocity_ff_ = {0.0, 0.0, 0.0};
        current_yaw_ = mission_complete_yaw_;
        transition_to(State::MISSION_COMPLETE_HOLD,
          "trajectory complete; holding final start point");
        return;
      }
    }

    line_forward_leg_ = !line_forward_leg_;
    if (yaw_mode_ == "fixed") {
      start_next_line_leg("fixed yaw retained; reverse acceleration started");
    } else {
      start_line_turn();
    }
  }

  void update_line_endpoint_hold()
  {
    const double endpoint_scalar = line_forward_leg_ ? line_distance_active_ : 0.0;
    line_s_ref_ = line_distance_active_;
    line_s_dot_ref_ = 0.0;
    line_s_ddot_ref_ = 0.0;
    set_line_leg_reference(line_s_ref_, 0.0, 0.0);
    log_phase_once(LinePhase::SETTLE);

    if (!local_pose_ready()) {
      return;
    }

    const Vec3 endpoint = line_start_position_ + line_unit_ * endpoint_scalar;
    const auto status = endpoint_status(
      endpoint, endpoint_position_tolerance_m_, endpoint_speed_tolerance_mps_,
      line_endpoint_settle_started_);
    const bool endpoint_settle_complete =
      seconds_between(line_endpoint_settle_start_time_, steady_now()) >=
      endpoint_settle_time_s_ + line_endpoint_hold_time_s_;
    if (status.within_limits) {
      if (!line_endpoint_settle_started_) {
        line_endpoint_settle_started_ = true;
        line_endpoint_settle_start_time_ = steady_now();
        if (!line_endpoint_reached_logged_) {
          RCLCPP_INFO(get_logger(), "endpoint reached");
          line_endpoint_reached_logged_ = true;
        }
      } else if (endpoint_settle_complete) {
        finish_line_leg();
      }
    } else {
      line_endpoint_settle_started_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for endpoint settle: horizontal %.3f m, altitude %.3f m, "
        "horizontal speed %.3f m/s, vertical speed %.3f m/s",
        status.horizontal_position_error, status.altitude_error,
        status.horizontal_speed, status.vertical_speed);
    }
  }

  void reset_runtime()
  {
    approach_settle_started_ = false;
    approach_duration_s_ = 0.0;
    line_leg_active_ = false;
    line_turn_active_ = false;
    line_forward_leg_ = true;
    line_leg_index_ = 0;
    completed_reciprocations_ = 0;
    last_line_phase_ = LinePhase::IDLE;
    line_endpoint_settle_started_ = false;
    line_endpoint_reached_logged_ = false;
    line_turn_settle_started_ = false;
    waypoint_points_.clear();
    waypoint_index_ = 0;
    waypoint_segment_active_ = false;
    waypoint_continuous_active_ = false;
    waypoint_complete_ = false;
    waypoint_endpoint_settle_started_ = false;
    waypoint_cumulative_lengths_.clear();
    waypoint_total_length_ = 0.0;
    sine_progress_active_ = false;
    sine_phase_ = LinePhase::IDLE;
    sine_speed_planner_.reset();
    circle_progress_active_ = false;
    circle_phase_ = LinePhase::IDLE;
    circle_speed_planner_.reset();
    square_progress_active_ = false;
    square_state_ = SquareState::IDLE;
    square_phase_ = LinePhase::IDLE;
    square_speed_planner_.reset();
    square_vertex_settle_started_ = false;
    square_yaw_settle_started_ = false;
    velocity_path_.clear();
    velocity_path_active_ = false;
    velocity_projection_initialized_ = false;
    velocity_speed_planner_.reset();
    vertical_speed_planner_.reset();
    velocity_endpoint_settle_started_ = false;
    velocity_endpoint_position_ok_ = false;
    velocity_endpoint_speed_ok_ = false;
    velocity_endpoint_altitude_ok_ = false;
    velocity_endpoint_vertical_speed_ok_ = false;
    velocity_reverse_yaw_pending_ = false;
    velocity_reverse_yaw_settle_started_ = false;
  }

  void stop_active_mission_after_offboard_exit()
  {
    have_setpoint_ = false;
    reset_runtime();
    transition_to(
      State::PAUSED_AFTER_OFFBOARD_EXIT,
      "offboard exited; trajectory paused and active setpoint publication stopped");
  }

  void check_active_mode_loss()
  {
    const bool active =
      state_ == State::APPROACH_START ||
      state_ == State::RUNNING_TRAJECTORY ||
      state_ == State::TERMINAL_CAPTURE ||
      state_ == State::MISSION_COMPLETE_HOLD;
    if (active && !is_offboard()) {
      stop_active_mission_after_offboard_exit();
    }
  }

  void check_landing_state()
  {
    if (landing_or_disarmed() && state_ != State::WAIT_FOR_TAKEOFF_REFERENCE &&
      state_ != State::LANDING_OR_DISARMED)
    {
      have_setpoint_ = false;
      reset_runtime();
      transition_to(State::LANDING_OR_DISARMED, "landing detected or vehicle disarmed; silent");
    }
  }

  void control_loop()
  {
    const auto loop_now = steady_now();
    double dt = seconds_between(last_loop_time_, loop_now);
    last_loop_time_ = loop_now;
    control_dt_valid_ = std::isfinite(dt) && dt >= 0.001 && dt <= 0.1;
    if (!std::isfinite(dt)) {
      dt = 0.0;
    }
    control_dt_s_ = clamp(dt, 0.001, 0.1);
    acceleration_feedforward_valid_ = false;
    current_acceleration_ff_ = {0.0, 0.0, 0.0};

    update_takeoff_reference_from_arming();
    check_landing_state();
    check_active_mode_loss();

    switch (state_) {
      case State::WAIT_FOR_TAKEOFF_REFERENCE:
        have_setpoint_ = false;
        if (takeoff_reference_valid_) {
          transition_to(State::IDLE, "takeoff reference available; waiting for pilot Offboard");
        } else if (is_armed() && is_offboard()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Offboard is active but no takeoff reference was captured; refusing trajectory");
        } else if (!local_pose_ready()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "waiting for valid local position before takeoff reference capture");
        }
        break;

      case State::IDLE:
        have_setpoint_ = false;
        if (!takeoff_reference_valid_) {
          transition_to(State::WAIT_FOR_TAKEOFF_REFERENCE, "takeoff reference cleared");
        } else if (is_armed() && is_offboard()) {
          start_approach();
        } else if (is_armed() && !landing_or_disarmed()) {
          transition_to(State::PRESTREAM, "prestream started");
        }
        break;

      case State::PRESTREAM:
        update_current_hold_setpoint();
        if (!takeoff_reference_valid_) {
          transition_to(State::WAIT_FOR_TAKEOFF_REFERENCE, "takeoff reference cleared");
        } else if (!is_armed()) {
          transition_to(State::WAIT_FOR_TAKEOFF_REFERENCE, "waiting for next arming cycle");
        } else if (is_offboard()) {
          start_approach();
        } else if (!local_pose_ready()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "prestream waiting for fresh local position");
        }
        break;

      case State::APPROACH_START:
        if (!local_pose_ready()) {
          have_setpoint_ = false;
          transition_to(State::FAULT, "local position invalid during approach");
        } else {
          update_approach();
        }
        break;

      case State::RUNNING_TRAJECTORY:
        if (!local_pose_ready()) {
          have_setpoint_ = false;
          transition_to(State::FAULT, "local position invalid during trajectory");
        } else if (velocity_backend_enabled()) {
          update_velocity_path_controller(false);
        } else if (trajectory_type_ == "line_reciprocating" || trajectory_type_ == "line") {
          update_line_trajectory();
        } else {
          update_waypoint_trajectory();
        }
        break;

      case State::TERMINAL_CAPTURE:
        if (!local_pose_ready()) {
          have_setpoint_ = false;
          transition_to(State::FAULT, "local position invalid during terminal capture");
        } else if (!velocity_backend_enabled()) {
          transition_to(State::FAULT, "terminal capture requires velocity_path backend");
        } else {
          update_velocity_path_controller(true);
        }
        break;

      case State::MISSION_COMPLETE_HOLD:
        current_setpoint_ = mission_complete_position_;
        current_velocity_ff_ = {0.0, 0.0, 0.0};
        current_yaw_ = mission_complete_yaw_;
        have_setpoint_ = true;
        break;

      case State::PAUSED_AFTER_OFFBOARD_EXIT:
        have_setpoint_ = false;
        if (!takeoff_reference_valid_) {
          transition_to(State::WAIT_FOR_TAKEOFF_REFERENCE, "takeoff reference cleared");
        } else if (is_offboard()) {
          start_approach();
        }
        break;

      case State::LANDING_OR_DISARMED:
        have_setpoint_ = false;
        if (is_disarmed()) {
          clear_takeoff_reference("disarmed after landing or pilot stop");
          transition_to(State::WAIT_FOR_TAKEOFF_REFERENCE, "waiting for next flight");
        } else if (takeoff_reference_valid_ && is_armed() && !landing_or_disarmed()) {
          transition_to(State::IDLE, "landing state cleared; waiting for pilot Offboard");
        }
        break;

      case State::FAULT:
        have_setpoint_ = false;
        if (!is_armed()) {
          clear_takeoff_reference("fault reset by disarm");
          transition_to(State::WAIT_FOR_TAKEOFF_REFERENCE, "waiting for next flight");
        }
        break;
    }

    publish_if_allowed();
    publish_trajectory_debug();
  }

  std::string trajectory_type_{"line_reciprocating"};
  std::string trajectory_control_backend_{"velocity_path"};
  double trajectory_setpoint_rate_hz_{50.0};
  double mission_start_altitude_m_{5.0};
  double mission_start_east_offset_m_{3.0};
  double mission_start_yaw_deg_{0.0};
  double approach_horizontal_speed_mps_{1.0};
  double approach_vertical_speed_mps_{0.5};
  double approach_arrival_tolerance_m_{0.2};
  double approach_yaw_tolerance_deg_{5.0};
  double approach_settle_time_s_{0.5};
  double approach_yaw_rate_dps_{45.0};
  double line_distance_m_{10.0};
  double line_speed_mps_{2.0};
  int line_reciprocation_count_{3};
  double line_endpoint_hold_time_s_{0.0};
  double line_endpoint_arrival_tolerance_m_{0.25};
  double line_endpoint_velocity_tolerance_mps_{0.25};
  double line_turn_yaw_rate_dps_{30.0};
  double line_turn_yaw_tolerance_deg_{5.0};
  double line_turn_settle_time_s_{0.2};
  double line_direction_yaw_deg_{0.0};
  double line_length_{3.0};
  double circle_radius_{1.5};
  double circle_turns_{1.0};
  std::string circle_direction_{"cw"};
  bool continuous_trajectory_{false};
  double square_side_{3.0};
  double sine_length_{4.0};
  double sine_amplitude_{1.0};
  double sine_cycles_{1.0};
  double z_width_{4.0};
  double z_height_{2.0};
  int z_repeats_{1};
  double curve_samples_per_meter_{8.0};
  double accel_time_s_{0.5};
  double decel_time_s_{0.5};
  double max_acceleration_mps2_{1.5};
  double max_deceleration_mps2_{1.8};
  double max_jerk_mps3_{4.0};
  double lookahead_time_s_{0.25};
  double min_lookahead_distance_m_{0.10};
  double max_lookahead_distance_m_{0.35};
  double max_reference_lead_distance_m_{0.50};
  double response_delay_s_{0.40};
  double stopping_safety_margin_m_{0.15};
  double endpoint_position_tolerance_m_{0.12};
  double endpoint_speed_tolerance_mps_{0.20};
  double endpoint_settle_time_s_{0.50};
  double endpoint_altitude_tolerance_m_{0.25};
  double endpoint_vertical_speed_tolerance_mps_{0.30};
  double max_safe_lateral_acceleration_mps2_{1.5};
  double cross_track_gain_{0.5};
  double max_cross_track_correction_speed_mps_{0.30};
  double cross_track_speed_ratio_{0.40};
  double cross_track_slowdown_start_m_{0.60};
  double cross_track_stop_error_m_{1.50};
  double projection_filter_time_constant_s_{0.15};
  double projection_search_window_m_{2.0};
  bool terminal_capture_enabled_{true};
  double terminal_capture_distance_m_{0.50};
  double terminal_gain_{0.5};
  double terminal_max_speed_mps_{0.15};
  double altitude_velocity_gain_{0.8};
  double max_vertical_speed_mps_{0.4};
  double max_vertical_acceleration_mps2_{0.5};
  double max_vertical_jerk_mps3_{1.0};
  double max_altitude_error_m_{2.0};
  bool velocity_path_acceleration_feedforward_enabled_{true};
  std::string square_corner_mode_{"rounded"};
  double square_corner_radius_m_{1.0};
  std::string line_turnaround_mode_{"stop_reverse"};
  double line_turn_radius_m_{2.0};
  std::string yaw_mode_{"fixed"};
  bool yaw_rate_feedforward_enabled_{true};
  double max_yaw_rate_dps_{45.0};
  double max_yaw_acceleration_dps2_{90.0};
  double vertex_yaw_rate_dps_{30.0};
  double vertex_position_tolerance_m_{0.20};
  double vertex_speed_tolerance_mps_{0.20};
  double vertex_yaw_tolerance_deg_{5.0};
  double vertex_yaw_settle_time_s_{0.30};
  double publish_rate_hz_{20.0};
  double local_position_timeout_s_{1.0};
  double prestream_after_exit_delay_s_{1.0};
  double minimum_start_altitude_m_{0.5};
  double takeoff_reference_candidate_timeout_s_{3.0};

  State state_{State::WAIT_FOR_TAKEOFF_REFERENCE};
  SteadyTime state_enter_time_{};
  SteadyTime last_loop_time_{};
  SteadyTime last_local_position_time_{};

  bool local_position_received_{false};
  bool vehicle_status_received_{false};
  bool vehicle_land_detected_received_{false};
  bool arming_state_initialized_{false};
  bool was_armed_{false};
  bool takeoff_reference_valid_{false};
  bool ground_reference_candidate_valid_{false};
  bool have_setpoint_{false};

  px4_msgs::msg::VehicleLocalPosition local_position_{};
  px4_msgs::msg::VehicleStatus vehicle_status_{};
  px4_msgs::msg::VehicleLandDetected vehicle_land_detected_{};

  Vec3 takeoff_reference_{};
  Vec3 ground_reference_candidate_{};
  SteadyTime ground_reference_candidate_time_{};
  Vec3 current_setpoint_{};
  Vec3 current_velocity_ff_{};
  Vec3 current_acceleration_ff_{};
  double current_yaw_{0.0};
  double current_yawspeed_{0.0};
  bool acceleration_feedforward_valid_{false};
  double control_dt_s_{0.05};
  bool control_dt_valid_{true};

  std::vector<VelocityPathPoint> velocity_path_;
  bool velocity_path_active_{false};
  bool velocity_path_closed_{false};
  bool velocity_projection_initialized_{false};
  double velocity_path_length_{0.0};
  double velocity_mission_length_{0.0};
  double velocity_effective_turn_radius_m_{0.0};
  double velocity_s_raw_{0.0};
  double velocity_s_filtered_{0.0};
  double velocity_s_plan_{0.0};
  Vec3 velocity_projected_point_{};
  double velocity_cross_track_error_{0.0};
  double velocity_along_track_error_{0.0};
  double velocity_remaining_distance_{0.0};
  double velocity_planner_target_speed_{0.0};
  double velocity_planner_speed_{0.0};
  double velocity_planner_acceleration_{0.0};
  double velocity_tangent_speed_command_{0.0};
  double velocity_normal_speed_command_{0.0};
  double velocity_total_speed_command_{0.0};
  double velocity_vertical_speed_command_{0.0};
  double velocity_actual_horizontal_speed_{0.0};
  double velocity_actual_tangent_speed_{0.0};
  double velocity_actual_normal_speed_{0.0};
  double velocity_curve_limit_mps_{0.0};
  double velocity_terminal_speed_limit_mps_{0.0};
  double velocity_cross_track_scale_{1.0};
  double velocity_terminal_direction_{1.0};
  SpeedPlanner velocity_speed_planner_{};
  SignedSpeedPlanner vertical_speed_planner_{};
  double velocity_fixed_yaw_{0.0};
  double velocity_yaw_unwrapped_{0.0};
  double velocity_yaw_rate_{0.0};
  double velocity_yaw_acceleration_{0.0};
  bool velocity_endpoint_settle_started_{false};
  SteadyTime velocity_endpoint_settle_start_time_{};
  bool velocity_endpoint_position_ok_{false};
  bool velocity_endpoint_speed_ok_{false};
  bool velocity_endpoint_altitude_ok_{false};
  bool velocity_endpoint_vertical_speed_ok_{false};
  bool velocity_reverse_yaw_pending_{false};
  bool velocity_reverse_yaw_settle_started_{false};
  SteadyTime velocity_reverse_yaw_settle_start_time_{};

  Vec3 approach_start_position_{};
  Vec3 approach_target_position_{};
  double approach_start_yaw_{0.0};
  double approach_target_yaw_{0.0};
  double approach_duration_s_{0.0};
  SteadyTime approach_start_time_{};
  bool approach_settle_started_{false};
  SteadyTime approach_settle_start_time_{};

  LegProfile line_profile_{};
  Vec3 line_start_position_{};
  Vec3 line_unit_{1.0, 0.0, 0.0};
  Vec3 line_leg_origin_{};
  Vec3 line_leg_unit_{1.0, 0.0, 0.0};
  bool line_leg_active_{false};
  bool line_single_pass_{false};
  bool line_forward_leg_{true};
  double line_distance_active_{10.0};
  double line_s_ref_{0.0};
  double line_s_vehicle_{0.0};
  double line_s_dot_ref_{0.0};
  double line_s_ddot_ref_{0.0};
  double line_actual_speed_{0.0};
  double line_lead_{0.0};
  double line_remaining_distance_{0.0};
  double line_braking_distance_{0.0};
  double line_target_speed_{0.0};
  double line_dynamic_lookahead_{0.0};
  SpeedPlanner line_speed_planner_{};
  std::size_t line_leg_index_{0};
  int completed_reciprocations_{0};
  LinePhase last_line_phase_{LinePhase::IDLE};
  SteadyTime line_leg_start_time_{};
  bool line_endpoint_settle_started_{false};
  bool line_endpoint_reached_logged_{false};
  SteadyTime line_endpoint_settle_start_time_{};
  bool line_turn_active_{false};
  bool line_turn_settle_started_{false};
  double line_turn_start_yaw_{0.0};
  double line_turn_target_yaw_{0.0};
  double line_turn_duration_s_{0.0};
  SteadyTime line_turn_start_time_{};
  SteadyTime line_turn_settle_start_time_{};

  std::vector<Vec3> waypoint_points_;
  std::vector<double> waypoint_cumulative_lengths_;
  std::size_t waypoint_index_{0};
  bool waypoint_segment_active_{false};
  bool waypoint_continuous_active_{false};
  bool waypoint_complete_{false};
  double waypoint_total_length_{0.0};
  Vec3 waypoint_segment_start_{};
  Vec3 waypoint_segment_end_{};
  Vec3 waypoint_segment_unit_{};
  double waypoint_segment_yaw_{0.0};
  double waypoint_segment_length_{0.0};
  LegProfile waypoint_profile_{};
  SteadyTime waypoint_segment_start_time_{};
  SteadyTime waypoint_continuous_start_time_{};
  bool waypoint_endpoint_settle_started_{false};
  SteadyTime waypoint_endpoint_settle_start_time_{};

  bool sine_progress_active_{false};
  double sine_s_ref_{0.0};
  double sine_s_vehicle_{0.0};
  double sine_s_dot_ref_{0.0};
  double sine_s_ddot_ref_{0.0};
  double sine_actual_speed_{0.0};
  double sine_lead_{0.0};
  double sine_remaining_distance_{0.0};
  double sine_braking_distance_{0.0};
  double sine_target_speed_{0.0};
  double sine_dynamic_lookahead_{0.0};
  double sine_curve_speed_limit_mps_{0.0};
  LinePhase sine_phase_{LinePhase::IDLE};
  SpeedPlanner sine_speed_planner_{};

  bool circle_progress_active_{false};
  Vec3 circle_center_{};
  Vec3 circle_forward_unit_{1.0, 0.0, 0.0};
  Vec3 circle_right_unit_{0.0, 1.0, 0.0};
  double circle_lateral_sign_{1.0};
  double circle_previous_raw_angle_{0.0};
  double circle_vehicle_angle_unwrapped_{0.0};
  double circle_s_ref_{0.0};
  double circle_s_vehicle_{0.0};
  double circle_s_dot_ref_{0.0};
  double circle_s_ddot_ref_{0.0};
  double circle_actual_speed_{0.0};
  double circle_lead_{0.0};
  double circle_remaining_distance_{0.0};
  double circle_braking_distance_{0.0};
  double circle_target_speed_{0.0};
  double circle_dynamic_lookahead_{0.0};
  double circle_speed_limit_mps_{0.0};
  LinePhase circle_phase_{LinePhase::IDLE};
  SpeedPlanner circle_speed_planner_{};

  bool square_progress_active_{false};
  SquareState square_state_{SquareState::IDLE};
  LinePhase square_phase_{LinePhase::IDLE};
  double square_s_ref_{0.0};
  double square_s_vehicle_{0.0};
  double square_s_dot_ref_{0.0};
  double square_s_ddot_ref_{0.0};
  double square_actual_speed_{0.0};
  double square_lead_{0.0};
  double square_remaining_distance_{0.0};
  double square_braking_distance_{0.0};
  double square_target_speed_{0.0};
  double square_dynamic_lookahead_{0.0};
  SpeedPlanner square_speed_planner_{};
  bool square_vertex_settle_started_{false};
  SteadyTime square_vertex_settle_start_time_{};
  bool square_yaw_settle_started_{false};
  SteadyTime square_yaw_settle_start_time_{};
  double square_turn_start_yaw_{0.0};
  double square_turn_target_yaw_{0.0};
  double square_turn_duration_s_{0.0};
  SteadyTime square_turn_start_time_{};

  Vec3 mission_complete_position_{};
  double mission_complete_yaw_{0.0};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_debug_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OffboardRouteOffsetNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("offboard_route_offset"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
