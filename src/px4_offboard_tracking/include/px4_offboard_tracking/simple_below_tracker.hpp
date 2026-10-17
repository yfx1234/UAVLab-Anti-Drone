#pragma once

#include <array>
#include <string>

namespace px4_offboard_tracking
{

enum class SimpleBelowTrackingMode
{
  DIRECT,
  PREDICTIVE
};

SimpleBelowTrackingMode parse_simple_below_tracking_mode(const std::string & value);
const char * to_string(SimpleBelowTrackingMode mode);

struct SimpleBelowTrackerConfig
{
  double horizontal_position_gain{0.6};
  double prediction_horizon_s{0.2};
  double relative_velocity_filter_time_constant_s{0.15};
  double minimum_observation_dt_s{0.005};
  double maximum_observation_dt_s{0.25};
  double maximum_relative_speed_mps{12.0};
  double maximum_prediction_distance_m{2.0};
  double maximum_horizontal_speed_mps{3.0};
  double maximum_horizontal_acceleration_mps2{1.5};
};

struct SimpleBelowTrackerResult
{
  bool observation_valid{false};
  bool relative_velocity_valid{false};
  std::array<double, 2> relative_position_ned_m{};
  std::array<double, 2> relative_velocity_ned_mps{};
  std::array<double, 2> estimated_target_velocity_ned_mps{};
  std::array<double, 2> predicted_relative_position_ned_m{};
  std::array<double, 2> velocity_setpoint_ned_mps{};
};

class SimpleBelowTracker
{
public:
  explicit SimpleBelowTracker(const SimpleBelowTrackerConfig & config = {});

  bool update_observation(
    const std::array<double, 2> & relative_position_ned_m,
    double acquisition_time_s);

  SimpleBelowTrackerResult step(
    SimpleBelowTrackingMode mode, bool tracking_enabled, double control_dt_s,
    const std::array<double, 2> & observer_velocity_ned_mps);

  void reset_observation();

private:
  SimpleBelowTrackerConfig config_;
  std::array<double, 2> relative_position_ned_m_{};
  std::array<double, 2> previous_relative_position_ned_m_{};
  std::array<double, 2> relative_velocity_ned_mps_{};
  std::array<double, 2> velocity_setpoint_ned_mps_{};
  double previous_acquisition_time_s_{0.0};
  bool observation_valid_{false};
  bool previous_observation_valid_{false};
  bool relative_velocity_valid_{false};
};

}  // namespace px4_offboard_tracking
