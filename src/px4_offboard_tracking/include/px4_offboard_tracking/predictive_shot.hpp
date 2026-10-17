#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace px4_offboard_tracking
{

struct PredictiveShotConfig
{
  double projectile_speed_mps{400.0};
  double gravity_mps2{9.80665};
  double minimum_time_of_flight_s{0.001};
  double maximum_time_of_flight_s{0.25};
  double maximum_target_acceleration_mps2{20.0};
  double convergence_tolerance_s{1e-7};
  std::size_t maximum_iterations{12};
};

struct PredictiveShotInput
{
  std::array<double, 3> relative_position_ned{};
  std::array<double, 3> relative_velocity_ned{};
  std::array<double, 3> estimated_target_acceleration_ned{};
  // IMM-EKF position/velocity/acceleration covariance, row major.
  std::array<double, 81> state_covariance{};
};

struct PredictiveShotResult
{
  std::array<double, 3> shot_direction_ned{};
  std::array<double, 3> predicted_relative_intercept_ned{};
  double time_of_flight_s{};
  double range_m{};
  double lead_angle_rad{};
  double predicted_lateral_sigma_m{};
  bool valid{false};
  std::string reason{"not solved"};
};

class PredictiveShotSolver
{
public:
  explicit PredictiveShotSolver(const PredictiveShotConfig & config);

  PredictiveShotResult solve(const PredictiveShotInput & input) const;

private:
  PredictiveShotConfig config_;
};

}  // namespace px4_offboard_tracking
