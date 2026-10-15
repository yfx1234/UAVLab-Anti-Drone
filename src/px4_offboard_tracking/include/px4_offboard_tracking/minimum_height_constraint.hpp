#pragma once

namespace px4_offboard_tracking
{

struct MinimumHeightConstraintConfig
{
  double minimum_height_m{3.0};
  // Only intervene when the finite-horizon vertical prediction would cross
  // the hard floor.  Above the floor and outside that prediction window the
  // requested acceleration is passed through unchanged.
  double prediction_horizon_s{1.0};
  double maximum_upward_acceleration_mps2{3.0};
  double recovery_height_m{3.3};
  double recovery_gain_s2{1.0};
  double recovery_damping_s_inv{1.0};
};

struct MinimumHeightConstraintInput
{
  // Height above the captured local takeoff origin. Positive is upward.
  double height_m{};
  // PX4 local-NED vertical velocity. Positive is downward.
  double vertical_velocity_down_mps{};
  // Requested inertial acceleration in NED. Positive is downward.
  double requested_acceleration_down_mps2{};
};

struct MinimumHeightConstraintResult
{
  double acceleration_down_mps2{};
  double maximum_allowed_acceleration_down_mps2{};
  bool active{};
};

// Predictive hard-floor constraint.  It leaves normal control untouched above
// the floor unless the current downward motion and requested acceleration
// predict a crossing within prediction_horizon_s.  Below the floor it commands
// a bounded recovery only to recovery_height_m (normally about 3.3 m), never
// back to the nominal takeoff/hover height.
class MinimumHeightConstraint
{
public:
  explicit MinimumHeightConstraint(
    const MinimumHeightConstraintConfig & config = {});

  MinimumHeightConstraintResult apply(
    const MinimumHeightConstraintInput & input) const;

  const MinimumHeightConstraintConfig & config() const;

private:
  MinimumHeightConstraintConfig config_;
};

}  // namespace px4_offboard_tracking
