#pragma once

#include <array>
#include <cstdint>

namespace px4_offboard_tracking
{

struct BearingHandoverConfig
{
  double maximum_axis_rate_rad_s{0.8};
  double los_rate_filter_time_constant_s{0.15};
  double los_rate_lead_s{0.10};
  double maximum_sample_interval_s{0.25};
};

struct BearingHandoverInput
{
  std::array<double, 3> measured_los_common{};
  std::array<double, 3> current_attack_axis_common{};
  double sample_time_s{};
  std::uint32_t sequence{};
  bool visible{false};
};

struct BearingHandoverResult
{
  std::array<double, 3> commanded_axis_common{{0.0, 0.0, -1.0}};
  std::array<double, 3> los_rate_common{};
  double commanded_axis_rate_rad_s{};
  bool new_measurement{false};
  bool valid{false};
};

// Range-free handover controller. It consumes only a unit line of sight and
// bounds the commanded attack-axis angular speed at camera sample time.
class BearingHandover
{
public:
  explicit BearingHandover(const BearingHandoverConfig & config);

  BearingHandoverResult update(const BearingHandoverInput & input);
  BearingHandoverResult state() const;
  void reset();

private:
  BearingHandoverConfig config_;
  BearingHandoverResult state_;
  std::array<double, 3> previous_los_{};
  double previous_time_s_{};
  std::uint32_t previous_sequence_{};
  bool initialized_{false};
};

}  // namespace px4_offboard_tracking
