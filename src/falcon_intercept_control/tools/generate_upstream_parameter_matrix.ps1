param(
  [Parameter(Mandatory=$true)][string]$UpstreamRoot,
  [Parameter(Mandatory=$true)][string]$OutputCsv,
  [string]$OutputTestCsv = '',
  # px4_interceptor_bridge 已移至仓库归档区（archive/deprecated_packages），默认按脚本位置反推
  [string]$ArchiveRoot = (Join-Path $PSScriptRoot '..\..\..\..\archive\deprecated_packages')
)

$ErrorActionPreference = 'Stop'
$sources = @(
  @{ Component='mission'; Path=(Join-Path $UpstreamRoot 'px4_offboard_tracking/src/offboard_vehicle.cpp') },
  @{ Component='bridge'; Path=(Join-Path $ArchiveRoot 'px4_interceptor_bridge/src/interceptor_bridge_node.cpp') }
)

function Classify([string]$component, [string]$name) {
  if ($component -eq 'bridge') {
    if ($name -match '^(camera_|expected_frame_id)') { return @('ADAPTED','guidance_node camera extrinsics / FLU contract','FRD optical assumption replaced by measured FLU extrinsics') }
    if ($name -match '^(minimum_confidence|minimum_depth_m|maximum_measurement_age_s|maximum_position_variance|covariance_)') { return @('ADAPTED','TargetTrack + safety_node quality gates','Strong typed measurement and safety gate') }
    if ($name -match '^(timestamp_|timesync_|maximum_timesync_|require_timesync|maximum_future_s|measurement_queue_timeout_s|attitude_sync_tolerance_s|attitude_buffer_s)') { return @('PENDING_BASELINE','PX4 adapter time/attitude history','Requires matched PX4 firmware, px4_msgs and measured time alignment') }
    if ($name -match 'topic$|^input_topic$|^output_topic$') { return @('REPLACED','typed Jetson topics + standard PX4 adapter','Firmware custom TargetRelativeMeasurement bridge is not production path') }
    return @('DIAGNOSTIC','px4_adapter diagnostics','No control-semantic dependency')
  }
  if ($name -match '^(target_(scenario|heading_rad|motion_delay_s|path_length_m|path_width_m|radius_m|speed_mps)|takeoff_height_m|takeoff_ready_tolerance_m|attack_enable_delay_s|common_origin_topic|role)$') { return @('EXCLUDED_SIM','none','Gazebo target aircraft/takeoff/common-origin behavior') }
  if ($name -match '^(mpc_|hover_thrust|max_thrust|minimum_thrust|max_tilt_rad|attitude_|body_rate_|max_body_rate|maximum_attitude_|control_level|guidance_controller|acceleration_|max_horizontal_acceleration|max_vertical_acceleration|altitude_hold_max_acceleration)') { return @('EXCLUDED_LOW_LEVEL','PX4 inner loops','MPC, attitude, thrust and body-rate control remain in PX4') }
  if ($name -match '^(required_virtual_captures|reattack_cooldown_s)$') { return @('EXCLUDED_V0','single software FIRE then LAND','Double virtual capture/reattack removed') }
  if ($name -match '^startup_search_') { return @('ADAPTED','search_controller_node','Uses measured PX4 yaw and repeats until 120 s timeout') }
  if ($name -match '^(capture_|quality_|safety_|recovery_)') { return @('ADAPTED','safety_node / CaptureWindow','Real-camera quality, distinct source timestamps and one FIRE') }
  if ($name -match '^(corridor_|position_gain|velocity_feedforward|max_horizontal_speed|max_vertical_speed|standoff_distance|alignment_|aim_|invalid_estimate_)') { return @('ADAPTED','guidance_node / intercept_fsm_node','Forward-camera Body FLU corridor and conservative velocity limits') }
  if ($name -match 'topic$|frame_id$|^topic_prefix$|^target_system$') { return @('REPLACED','strong typed Jetson interfaces / px4_adapter','Topic and frame contract changed at the explicit FLU adapter boundary') }
  if ($name -match '^(offboard_rate_hz|offboard_warmup_s|state_timeout_s|preflight_stability_hold_s)$') { return @('ADAPTED','px4_adapter heartbeat/prestream/TTL','Manual takeoff and manual Offboard entry retained') }
  if ($name -match '^(mission_|altitude_correction_threshold_m|feasibility_)') { return @('ADAPTED','intercept_fsm_node / safety_node','Mission transition or diagnostic safety semantics') }
  return @('REVIEWED_NO_DIRECT_COPY','current conservative defaults','Upstream value is not copied without real calibration/SITL evidence')
}

$rows = @()
foreach ($source in $sources) {
  $text = Get-Content $source.Path -Raw
  $matches = [regex]::Matches($text, 'declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"')
  foreach ($match in $matches) {
    $name = $match.Groups[1].Value
    $decision = Classify $source.Component $name
    $rows += [pscustomobject]@{
      upstream_component = $source.Component
      upstream_parameter = $name
      disposition = $decision[0]
      jetson_mapping = $decision[1]
      rationale = $decision[2]
    }
  }
}

if ($rows.Count -ne 155) {
  throw "Expected exactly 155 upstream declarations (125 mission + 30 bridge), got $($rows.Count)"
}
$parent = Split-Path $OutputCsv -Parent
if ($parent) { New-Item -ItemType Directory -Force $parent | Out-Null }
$rows | Export-Csv -Path $OutputCsv -NoTypeInformation -Encoding utf8
Write-Output "wrote $($rows.Count) rows to $OutputCsv"

if ($OutputTestCsv) {
  $test_rows = @()
  $test_dirs = @(
    (Join-Path $UpstreamRoot 'px4_offboard_tracking/test'),
    (Join-Path $ArchiveRoot 'px4_interceptor_bridge/test')
  )
  foreach ($dir in $test_dirs) {
    Get-ChildItem $dir -Filter 'test_*.cpp' | ForEach-Object {
      $file = $_.Name
      $text = Get-Content $_.FullName -Raw
      foreach ($match in [regex]::Matches($text, 'TEST(?:_F)?\s*\(\s*([^,]+),\s*([^\)]+)\)')) {
        $suite = $match.Groups[1].Value.Trim()
        $case = $match.Groups[2].Value.Trim()
        $disposition = 'ADAPTED_NEEDS_DIFFERENTIAL'
        $coverage = 'test_real_flight_core + typed runtime smoke'
        $reason = 'Production algorithm retained with real-flight coordinate/state adaptation'
        if ($file -match 'attitude_setpoint|body_rate|mpc_guidance|alignment_feasibility') {
          $disposition = 'EXCLUDED_LOW_LEVEL'; $coverage = 'PX4 native controller'; $reason = 'Jetson outputs velocity/yaw only'
        } elseif ($file -match 'target_observation|target_trajectory') {
          $disposition = 'EXCLUDED_SIM'; $coverage = 'real camera/KF tests'; $reason = 'Gazebo truth, ideal camera, noise or target aircraft trajectory'
        } elseif ($file -eq 'test_common_frame_transform.cpp') {
          $disposition = 'REPLACED'; $coverage = 'FLU/FRD/NED sign tests'; $reason = 'No shared two-aircraft NED origin in production'
        } elseif ($file -eq 'test_relative_ekf.cpp') {
          $disposition = 'REPLACED'; $coverage = 'test_target_kf.py'; $reason = 'Existing six-state CV-KF is the single estimator'
        } elseif ($file -eq 'test_z_axis_alignment.cpp') {
          $disposition = 'REPLACED'; $coverage = 'forward optical/Body FLU geometry tests'; $reason = 'Upstream body -Z camera assumption is invalid for forward cameras'
        } elseif ($file -eq 'test_timestamp_offset_estimator.cpp') {
          $disposition = 'PENDING_BASELINE'; $coverage = 'SITL/bench time alignment'; $reason = 'Requires matched PX4 firmware and px4_msgs'
        } elseif ($file -eq 'test_bridge_geometry.cpp') {
          $disposition = 'ADAPTED'; $coverage = 'Coordinates.StandardOpticalToFlu + adapter FLU/FRD tests'; $reason = 'Conversion retained at the single adapter boundary'
        } elseif ($file -match 'capture_window|relative_kinematics|startup_search|tracking_quality_gate') {
          $disposition = 'ADAPTED'; $coverage = 'test_real_flight_core'; $reason = 'Pure production algorithm migrated and unit tested'
        } elseif ($file -match 'mission_state_machine|safety_manager') {
          $disposition = 'ADAPTED'; $coverage = 'typed happy/timeout/takeover runtime smoke'; $reason = 'Semantics adapted for manual Offboard, one FIRE and ABORT latch'
        }
        $test_rows += [pscustomobject]@{
          upstream_file = $file
          upstream_suite = $suite
          upstream_test = $case
          disposition = $disposition
          jetson_coverage = $coverage
          rationale = $reason
        }
      }
    }
  }
  if ($test_rows.Count -ne 92) {
    throw "Expected exactly 92 upstream C++ GTests, got $($test_rows.Count)"
  }
  $test_parent = Split-Path $OutputTestCsv -Parent
  if ($test_parent) { New-Item -ItemType Directory -Force $test_parent | Out-Null }
  $test_rows | Export-Csv -Path $OutputTestCsv -NoTypeInformation -Encoding utf8
  Write-Output "wrote $($test_rows.Count) rows to $OutputTestCsv"
}
