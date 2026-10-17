#include "netgun_intercept/netgun_model.hpp"

#include <cmath>

namespace netgun_intercept
{

NetgunModel::NetgunModel(const NetgunModelConfig & config)
: config_(config)
{
}

double NetgunModel::time_of_flight_s(double range_m) const
{
  if (!(config_.muzzle_speed_mps > 0.0) || !(range_m >= 0.0)) {
    return 0.0;
  }
  return range_m / config_.muzzle_speed_mps;
}

double NetgunModel::trigger_lead_s(double range_m) const
{
  return time_of_flight_s(range_m) + config_.deployment_time_s;
}

bool NetgunModel::covered(const CoverageInput & input) const
{
  const double half = config_.coverage_half_extent_m;
  return std::isfinite(input.lateral_x_m) && std::isfinite(input.lateral_y_m) &&
         std::isfinite(input.range_m) &&
         std::abs(input.lateral_x_m) <= half &&
         std::abs(input.lateral_y_m) <= half &&
         range_within_effective(input.range_m);
}

bool NetgunModel::range_within_effective(double range_m) const
{
  return std::isfinite(range_m) &&
         range_m >= config_.effective_range_min_m &&
         range_m <= config_.effective_range_max_m;
}

const NetgunModelConfig & NetgunModel::config() const
{
  return config_;
}

}  // namespace netgun_intercept
