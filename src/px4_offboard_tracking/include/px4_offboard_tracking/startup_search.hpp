#pragma once

namespace px4_offboard_tracking
{

struct StartupSearchConfig
{
  bool enabled{false};
  double yaw_rate_rad_s{0.5};
  double turns{1.0};
  double settle_time_s{0.5};
  bool stop_on_acquisition{true};
  bool repeat_until_acquired{true};
  double acquisition_hold_time_s{0.2};
};

struct StartupSearchResult
{
  bool pending{false};
  bool active{false};
  bool completed{true};
  bool target_acquired{false};
  double desired_yaw_rad{0.0};
  double progress{1.0};
};

// Optional mission-chain stage: hold position and rotate about body Z after takeoff.
// The class is ROS-independent so the search policy can be tested and reused.
class StartupSearch
{
public:
  explicit StartupSearch(const StartupSearchConfig & config);

  StartupSearchResult update(
    bool takeoff_ready, bool target_visible, double now_s, double current_yaw_rad);

  void reset();
  bool enabled() const {return config_.enabled;}

private:
  StartupSearchConfig config_;
  bool started_{false};
  bool completed_{false};
  bool completed_by_acquisition_{false};
  double start_time_s_{0.0};
  double start_yaw_rad_{0.0};
  double acquisition_started_s_{0.0};
  bool acquisition_pending_{false};
};

}  // namespace px4_offboard_tracking
