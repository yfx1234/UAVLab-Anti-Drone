#pragma once

#include <array>
#include <limits>

#include "px4_offboard_tracking/acceleration_guidance.hpp"

namespace px4_offboard_tracking
{

struct Unified3dGuidanceConfig
{
  double navigation_constant{3.5};
  // Symmetric radial acceleration/braking limit.
  double radial_acceleration_mps2{1.5};
  double radial_velocity_gain{0.35};
  double desired_closing_speed_mps{4.0};
  double target_acceleration_feedforward_gain{0.0};
  double maximum_acceleration_mps2{5.0};
  double maximum_tilt_rad{0.65};
  double depth_fov_constraint_rad{0.13962634015954636};  // 8 deg, inside 8.5 deg HFOV.
  // Tighten to the 1 deg aim target before entering the 15-35 m capture window.
  double terminal_alignment_start_range_m{45.0};
  double capture_alignment_range_m{35.0};
  double capture_alignment_rad{0.017453292519943295};  // 1 deg target.
  // Match target motion perpendicular to the LOS even when radial closing
  // speed is zero (the classical crossing-target blind spot of PN).
  double transverse_velocity_gain{1.0};
};

struct Unified3dGuidanceInput
{
  std::array<double, 3> relative_position_ned{};
  std::array<double, 3> relative_velocity_ned{};
  // This must be target acceleration in common NED, never raw relative acceleration.
  // Leave it zero until pursuer acceleration is measured and added to the relative estimate.
  std::array<double, 3> target_acceleration_ned{};
  // Optional mission-level speed schedule. NaN selects the configured intercept speed.
  double desired_closing_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  // Optional FOV-barrier override. This is a per-cycle constraint generated
  // from the measured horizontal/vertical camera errors and their short
  // prediction, not a fixed gain schedule.
  double maximum_axis_error_rad{std::numeric_limits<double>::quiet_NaN()};
  // Keep the wider depth-camera tracking cone until the mission has completed
  // its stable-track delay. Tight one-degree alignment is only physically
  // required while an attack may actually be confirmed.
  bool terminal_alignment_required{true};
  // Confidence supplied by the IMM/covariance gate. Zero disables maneuver
  // feed-forward without disabling position/velocity guidance.
  double target_acceleration_confidence{1.0};
  // Mission-level ramp from the wide tracking cone (0) to the range-shaped
  // terminal alignment cone (1). This lets velocity matching settle before
  // alignment tightens and avoids a one-cycle command change when a fire
  // authorization timer expires.
  double terminal_alignment_strength{1.0};
  // Optional receding-horizon launch-position command.  A positive selector
  // replaces the legacy radial controller with the planner's radial
  // acceleration; PN, transverse matching and the common FOV/tilt/acceleration
  // constraints remain active.  Radial commands are never summed or blended.
  std::array<double, 3> planned_acceleration_ned{};
  double launch_planner_blend{0.0};
};

struct Unified3dGuidanceResult
{
  AccelerationCommand acceleration{};
  std::array<double, 3> line_of_sight_ned{};
  std::array<double, 3> proportional_navigation_acceleration_ned{};
  std::array<double, 3> transverse_velocity_acceleration_ned{};
  std::array<double, 3> target_acceleration_feedforward_ned{};
  std::array<double, 3> free_attack_axis_ned{};
  std::array<double, 3> constrained_attack_axis_ned{};
  double range_m{};
  double closing_speed_mps{};
  double line_of_sight_rate_rad_s{};
  double attack_axis_error_rad{};
  double allowed_axis_error_rad{};
  double launch_planner_blend{};
  double selected_radial_acceleration_mps2{};
  bool planned_radial_control_active{false};
  bool field_of_view_constrained{false};
  bool valid{false};
};

// A single vector guidance law for the whole depth-tracking phase.
//
// It computes three-dimensional PN plus radial closing acceleration, then applies
// the rigid upward-camera field-of-view constraint in thrust-axis space. The
// returned acceleration is reconstructed from that one constrained thrust vector,
// so the attitude and collective command cannot silently override the guidance.
class Unified3dGuidance
{
public:
  explicit Unified3dGuidance(const Unified3dGuidanceConfig & config);

  Unified3dGuidanceResult update(const Unified3dGuidanceInput & input) const;

private:
  Unified3dGuidanceConfig config_;
};

}  // namespace px4_offboard_tracking
