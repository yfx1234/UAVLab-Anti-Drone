#include "px4_offboard_tracking/predictive_shot.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

using Vector = std::array<double, 3>;

double dot(const Vector & first, const Vector & second)
{
  return first[0] * second[0] +
         first[1] * second[1] +
         first[2] * second[2];
}

double norm(const Vector & value)
{
  return std::sqrt(dot(value, value));
}

Vector add(const Vector & first, const Vector & second)
{
  return {
    first[0] + second[0],
    first[1] + second[1],
    first[2] + second[2]};
}

Vector scale(const Vector & value, double factor)
{
  return {factor * value[0], factor * value[1], factor * value[2]};
}

Vector normalized(const Vector & value)
{
  const double length = norm(value);
  if (!std::isfinite(length) || length < 1e-12) {
    return {};
  }
  return scale(value, 1.0 / length);
}

bool finite(const Vector & value)
{
  return
    std::isfinite(value[0]) &&
    std::isfinite(value[1]) &&
    std::isfinite(value[2]);
}

double angle_between(const Vector & first, const Vector & second)
{
  return std::acos(std::clamp(
    dot(normalized(first), normalized(second)), -1.0, 1.0));
}

Vector limited(const Vector & value, double maximum)
{
  const double length = norm(value);
  if (length <= maximum || length < 1e-12) {
    return value;
  }
  return scale(value, maximum / length);
}

}  // namespace

PredictiveShotSolver::PredictiveShotSolver(const PredictiveShotConfig & config)
: config_(config)
{
  if (config_.projectile_speed_mps <= 0.0 ||
    config_.gravity_mps2 < 0.0 ||
    config_.minimum_time_of_flight_s < 0.0 ||
    config_.maximum_time_of_flight_s <= config_.minimum_time_of_flight_s ||
    config_.maximum_target_acceleration_mps2 < 0.0 ||
    config_.convergence_tolerance_s <= 0.0 ||
    config_.maximum_iterations == 0)
  {
    throw std::invalid_argument("invalid predictive shot configuration");
  }
}

PredictiveShotResult PredictiveShotSolver::solve(
  const PredictiveShotInput & input) const
{
  PredictiveShotResult output;
  if (!finite(input.relative_position_ned) ||
    !finite(input.relative_velocity_ned) ||
    !finite(input.estimated_target_acceleration_ned))
  {
    output.reason = "non-finite state";
    return output;
  }

  output.range_m = norm(input.relative_position_ned);
  if (output.range_m < 1e-6) {
    output.reason = "zero relative range";
    return output;
  }

  const double relative_speed_squared = dot(
    input.relative_velocity_ned, input.relative_velocity_ned);
  const double quadratic = config_.projectile_speed_mps *
    config_.projectile_speed_mps - relative_speed_squared;
  if (quadratic <= 1e-9) {
    output.reason = "relative speed exceeds projectile speed";
    return output;
  }

  const double radial_product = dot(
    input.relative_position_ned, input.relative_velocity_ned);
  const double discriminant =
    radial_product * radial_product + quadratic * output.range_m * output.range_m;
  if (!std::isfinite(discriminant) || discriminant < 0.0) {
    output.reason = "no positive constant-velocity intercept";
    return output;
  }

  double time = (
    radial_product + std::sqrt(discriminant)) / quadratic;
  if (!std::isfinite(time) || time <= 0.0) {
    output.reason = "invalid constant-velocity intercept time";
    return output;
  }

  const Vector target_acceleration = limited(
    input.estimated_target_acceleration_ned,
    config_.maximum_target_acceleration_mps2);
  const Vector gravity_ned{0.0, 0.0, config_.gravity_mps2};
  const Vector compensated_acceleration{
    target_acceleration[0] - gravity_ned[0],
    target_acceleration[1] - gravity_ned[1],
    target_acceleration[2] - gravity_ned[2]};

  Vector intercept{};
  bool converged = false;
  for (std::size_t iteration = 0; iteration < config_.maximum_iterations; ++iteration) {
    intercept = add(
      add(
        input.relative_position_ned,
        scale(input.relative_velocity_ned, time)),
      scale(compensated_acceleration, 0.5 * time * time));
    const double next_time = norm(intercept) / config_.projectile_speed_mps;
    if (!std::isfinite(next_time) || next_time <= 0.0) {
      output.reason = "accelerated intercept iteration diverged";
      return output;
    }
    if (std::abs(next_time - time) <= config_.convergence_tolerance_s) {
      time = next_time;
      converged = true;
      break;
    }
    time = next_time;
  }
  if (!converged) {
    output.reason = "accelerated intercept iteration did not converge";
    return output;
  }
  if (time < config_.minimum_time_of_flight_s ||
    time > config_.maximum_time_of_flight_s)
  {
    output.reason = "intercept time outside configured window";
    return output;
  }

  intercept = add(
    add(
      input.relative_position_ned,
      scale(input.relative_velocity_ned, time)),
    scale(compensated_acceleration, 0.5 * time * time));
  output.shot_direction_ned = normalized(intercept);
  if (!finite(output.shot_direction_ned) ||
    norm(output.shot_direction_ned) < 0.99)
  {
    output.reason = "invalid shot direction";
    return output;
  }

  output.predicted_relative_intercept_ned = intercept;
  output.time_of_flight_s = time;
  output.lead_angle_rad = angle_between(
    input.relative_position_ned, output.shot_direction_ned);

  std::array<double, 9> impact_covariance{};
  const std::array<double, 3> coefficients{1.0, time, 0.5 * time * time};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      double value = 0.0;
      for (std::size_t first_block = 0; first_block < 3; ++first_block) {
        for (std::size_t second_block = 0; second_block < 3; ++second_block) {
          const std::size_t state_row = 3 * first_block + row;
          const std::size_t state_column = 3 * second_block + column;
          const double covariance =
            input.state_covariance[9 * state_row + state_column];
          if (!std::isfinite(covariance)) {
            output.reason = "non-finite state covariance";
            return output;
          }
          value += coefficients[first_block] * coefficients[second_block] * covariance;
        }
      }
      impact_covariance[3 * row + column] = value;
    }
  }

  double trace = 0.0;
  double axial_variance = 0.0;
  for (std::size_t row = 0; row < 3; ++row) {
    trace += impact_covariance[3 * row + row];
    for (std::size_t column = 0; column < 3; ++column) {
      axial_variance += output.shot_direction_ned[row] *
        impact_covariance[3 * row + column] *
        output.shot_direction_ned[column];
    }
  }
  output.predicted_lateral_sigma_m =
    std::sqrt(std::max(0.0, trace - axial_variance));
  output.valid =
    std::isfinite(output.lead_angle_rad) &&
    std::isfinite(output.predicted_lateral_sigma_m);
  output.reason = output.valid ? "valid intercept" : "non-finite result";
  return output;
}

}  // namespace px4_offboard_tracking
