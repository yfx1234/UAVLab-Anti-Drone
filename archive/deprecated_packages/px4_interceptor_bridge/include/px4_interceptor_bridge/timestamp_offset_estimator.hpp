#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>

namespace px4_interceptor_bridge
{

struct TimestampOffsetUpdate
{
  std::int64_t offset_ns{};
  bool initialized{};
  bool accepted{};
  bool confirmed_jump{};
  bool latency_outlier{};
};

class TimestampOffsetEstimator
{
public:
  TimestampOffsetEstimator(
    std::int64_t jump_threshold_ns, double filter_alpha,
    std::size_t window_size, unsigned jump_confirmation_samples)
  : jump_threshold_ns_(jump_threshold_ns),
    filter_alpha_(filter_alpha),
    window_size_(window_size),
    jump_confirmation_samples_(jump_confirmation_samples)
  {}

  TimestampOffsetUpdate update(std::int64_t observed_offset_ns)
  {
    TimestampOffsetUpdate result;
    observations_.push_back(observed_offset_ns);
    while (observations_.size() > window_size_) {
      observations_.pop_front();
    }
    const auto minimum = *std::min_element(observations_.begin(), observations_.end());
    if (!initialized_) {
      offset_ns_ = minimum;
      initialized_ = true;
      result.accepted = true;
    } else if (std::llabs(minimum - offset_ns_) <= jump_threshold_ns_) {
      offset_ns_ += static_cast<std::int64_t>(
        filter_alpha_ * static_cast<double>(minimum - offset_ns_));
      jump_candidate_count_ = 0;
      result.accepted = true;
      result.latency_outlier = observed_offset_ns > offset_ns_ + jump_threshold_ns_;
    } else {
      if (jump_candidate_count_ == 0 ||
        std::llabs(minimum - jump_candidate_ns_) > jump_threshold_ns_ / 2)
      {
        jump_candidate_ns_ = minimum;
        jump_candidate_count_ = 1;
      } else {
        ++jump_candidate_count_;
      }
      if (jump_candidate_count_ >= jump_confirmation_samples_) {
        offset_ns_ = minimum;
        observations_.clear();
        observations_.push_back(observed_offset_ns);
        jump_candidate_count_ = 0;
        result.accepted = true;
        result.confirmed_jump = true;
      }
    }
    result.offset_ns = offset_ns_;
    result.initialized = initialized_;
    return result;
  }

  bool initialized() const {return initialized_;}
  std::int64_t offset_ns() const {return offset_ns_;}

private:
  std::int64_t jump_threshold_ns_{};
  double filter_alpha_{};
  std::size_t window_size_{};
  unsigned jump_confirmation_samples_{};
  std::deque<std::int64_t> observations_;
  std::int64_t offset_ns_{};
  std::int64_t jump_candidate_ns_{};
  unsigned jump_candidate_count_{};
  bool initialized_{};
};

}  // namespace px4_interceptor_bridge
