#pragma once

#include <string>

namespace px4_offboard_tracking
{

struct SafetyManagerConfig
{
  double minimum_range_m{5.0};
  double maximum_tilt_rad{0.45};
  double maximum_speed_mps{4.0};
  double maximum_altitude_error_m{20.0};
  double recovery_maximum_tilt_rad{0.12};
  double recovery_maximum_speed_mps{0.5};
  double recovery_maximum_altitude_error_m{0.6};
  double recovery_required_hold_time_s{1.0};
};

struct SafetyManagerInput
{
  double timestamp_s{};
  bool monitoring_active{false};
  bool recovery_active{false};
  bool local_state_valid{false};
  bool attitude_valid{false};
  bool range_valid{false};
  double range_m{};
  double tilt_rad{};
  double speed_mps{};
  double altitude_error_m{};
};

struct SafetyManagerResult
{
  bool abort_requested{false};
  bool recovery_complete{false};
  bool local_state_ok{false};
  bool attitude_ok{false};
  bool range_ok{false};
  bool tilt_ok{false};
  bool speed_ok{false};
  bool altitude_ok{false};
  bool recovery_instantaneous_pass{false};
  double recovery_elapsed_s{};
  std::string reason{"monitoring inactive"};
};

class SafetyManager
{
public:
  explicit SafetyManager(const SafetyManagerConfig & config);

  SafetyManagerResult update(const SafetyManagerInput & input);
  void reset();

private:
  SafetyManagerConfig config_;
  bool timestamp_initialized_{false};
  double previous_timestamp_s_{};
  bool recovery_hold_active_{false};
  double recovery_hold_started_s_{};
};

}  // namespace px4_offboard_tracking
