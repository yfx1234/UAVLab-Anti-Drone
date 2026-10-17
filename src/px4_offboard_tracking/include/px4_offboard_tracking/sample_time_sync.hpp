#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "px4_offboard_tracking/camera_model.hpp"

namespace px4_offboard_tracking
{

// Small, ROS-independent buffer used by simulated and real perception adapters.
// All timestamps passed to one instance must share the same clock domain.
class SampleTimeAttitudeBuffer
{
public:
  SampleTimeAttitudeBuffer(double history_duration_s, double synchronization_tolerance_s);

  bool add(double timestamp_s, const Quaterniond & q_body_to_ned);
  bool sample(double timestamp_s, Quaterniond & q_body_to_ned) const;
  void clear();
  std::size_t size() const;

  // PX4 timestamps are translated into the ROS clock domain by uXRCE-DDS when
  // available. Fall back to receipt time if an installation exposes raw HRT.
  static double px4_sample_or_receipt_time(
    std::uint64_t timestamp_sample_us, double receipt_time_s,
    double maximum_clock_domain_difference_s);
  static double px4_sample_or_receipt_time(
    std::uint64_t timestamp_us, std::uint64_t timestamp_sample_us,
    double receipt_time_s, double maximum_clock_domain_difference_s);

private:
  struct Sample
  {
    double timestamp_s{};
    Quaterniond q_body_to_ned{};
  };

  static Quaterniond normalized(const Quaterniond & quaternion);
  static Quaterniond slerp(
    const Quaterniond & first, const Quaterniond & second, double fraction);

  double history_duration_s_;
  double synchronization_tolerance_s_;
  std::deque<Sample> samples_;
};

}  // namespace px4_offboard_tracking
