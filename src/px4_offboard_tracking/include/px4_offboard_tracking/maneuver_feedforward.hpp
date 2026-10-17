#pragma once

#include <array>

namespace px4_offboard_tracking
{

struct ManeuverFeedforwardConfig
{
  double minimum_ca_probability{0.25};
  double full_ca_probability{0.75};
  double maximum_acceleration_stddev_mps2{5.0};
  double filter_time_constant_s{0.20};
  double maximum_target_acceleration_mps2{4.0};
};

struct ManeuverFeedforwardResult
{
  std::array<double, 3> filtered_target_acceleration_ned{};
  double confidence{};
  bool active{false};
};

// Converts the noisy IMM constant-acceleration state into a bounded guidance
// feed-forward input. The CA probability identifies a maneuver; acceleration
// covariance suppresses feed-forward when perception is lost or uncertain.
class ManeuverFeedforward
{
public:
  explicit ManeuverFeedforward(const ManeuverFeedforwardConfig & config);

  ManeuverFeedforwardResult update(
    const std::array<double, 3> & target_acceleration_ned,
    double ca_probability,
    double maximum_acceleration_stddev_mps2,
    bool estimate_fresh,
    double dt_s);

  void reset();

private:
  ManeuverFeedforwardConfig config_;
  std::array<double, 3> filtered_acceleration_{};
};

}  // namespace px4_offboard_tracking
