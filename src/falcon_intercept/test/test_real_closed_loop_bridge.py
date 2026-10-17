import math
from pathlib import Path

import numpy as np

from falcon_intercept.perception_bridge_node import (
    PinholeIntrinsics,
    depth_mount_rotation,
    depth_optical_to_canonical,
    optical_range,
    pixel_to_optical_los,
    rotate_covariance_3x3,
    search_optical_to_canonical,
)
from falcon_intercept.px4_upstream_adapter_node import (
    finite_vector,
    selected_setpoint_kind,
    valid_attitude_payload,
)


def test_search_principal_point_is_forward_optical_axis():
    intrinsics = PinholeIntrinsics(2560, 1440, 4059.0, 1247.0, 1279.5, 719.5)
    los, horizontal, vertical = pixel_to_optical_los(1279.5, 719.5, intrinsics)
    assert np.allclose(los, (0.0, 0.0, 1.0), atol=1.0e-12)
    assert horizontal == 0.0
    assert vertical == 0.0


def test_search_optical_bearing_signs_follow_rep103():
    intrinsics = PinholeIntrinsics(2560, 1440, 1000.0, 1000.0, 1280.0, 720.0)
    los, horizontal, vertical = pixel_to_optical_los(1380.0, 620.0, intrinsics)
    assert los[0] > 0.0  # image right -> optical +X
    assert los[1] < 0.0  # image up -> optical -Y
    assert horizontal > 0.0
    assert vertical < 0.0
    assert math.isclose(np.linalg.norm(los), 1.0, rel_tol=1.0e-12)


def test_depth_range_uses_optical_vector_without_extra_rotation():
    assert optical_range((3.0, 4.0, 12.0)) == 13.0


# --- Mount compensation (2026-08-13 field mapping: rig rolled -90 deg) -------


def _controller_search_body(canonical):
    """The hardcoded upstream mapping (search_los_ned) for a level attitude."""
    sine = 0.17364817766693033
    cosine = 0.98480775301220802
    ox, oy, oz = canonical
    return (cosine * oy + sine * oz, ox, sine * oy - cosine * oz)


def test_mount_compensation_identity_when_rig_matches_canonical():
    assert np.allclose(
        search_optical_to_canonical((0.3, -0.2, 1.0), 0.0, 0.0), (0.3, -0.2, 1.0)
    )
    # roll=0 时物理安装即规范安装，深度链输入输出同为 REP-103 optical，恒等。
    assert np.allclose(depth_optical_to_canonical((0.3, -0.2, 1.0), 0.0), (0.3, -0.2, 1.0))


def test_search_optical_axis_keeps_upstream_ten_degree_boresight():
    # The physical search camera is already tilted 10 degrees toward the nose.
    # Its optical-axis ray therefore needs only the measured -90 degree image
    # roll correction.  The upstream matrix then maps it to the physical
    # noseward boresight rather than incorrectly treating it as Body -Z.
    canonical = search_optical_to_canonical((0.0, 0.0, 1.0))
    assert np.allclose(canonical, (0.0, 0.0, 1.0), atol=1.0e-12)
    assert np.allclose(
        _controller_search_body(canonical),
        (0.17364817766693033, 0.0, -0.98480775301220802),
        atol=1.0e-12,
    )


def test_search_body_overhead_target_round_trips_once_through_ten_degree_mount():
    # For a lens tilted 10 degrees toward the nose, a target on Body -Z appears
    # 10 degrees toward the tail (raw image right).  Roll-only adaptation gives
    # canonical (0,-sin10,cos10); the upstream 10 degree matrix then returns the
    # target to Body (0,0,-1).  This is the physical case that previously got a
    # second 10 degree correction in the bridge.
    sine = 0.17364817766693033
    cosine = 0.98480775301220802
    canonical = search_optical_to_canonical((sine, 0.0, cosine))
    assert np.allclose(canonical, (0.0, -sine, cosine), atol=1.0e-12)
    assert np.allclose(_controller_search_body(canonical), (0.0, 0.0, -1.0), atol=1.0e-12)


def test_search_body_right_target_round_trips_through_controller():
    # Target to the body's right, 45 deg up: body vector (0, 1, -1).
    # With the physical lens tilted 10 degrees toward the nose and image right
    # toward the tail, the corresponding raw ray is (sin10, 1, cos10).
    sine = 0.17364817766693033
    cosine = 0.98480775301220802
    canonical = search_optical_to_canonical((sine, 1.0, cosine))
    assert canonical[0] > 0.0  # canonical +X = body right
    assert np.allclose(
        _controller_search_body(canonical), (0.0, 1.0, -1.0), atol=1.0e-12
    )


def test_depth_nose_up_target_round_trips_through_controller():
    # Target above the nose: body vector (1, 0, -1).
    # Solver output is REP-103 optical of the physical rig (image right = tail,
    # image down = body right, lens up): (-V_x, V_y, -V_z) = (-1, 0, 1).
    canonical = depth_optical_to_canonical((-1.0, 0.0, 1.0))
    assert np.allclose(canonical, (0.0, 1.0, 1.0), atol=1.0e-12)
    # Upstream depth mapping: body = (optical_y, optical_x, -optical_z).
    body = (canonical[1], canonical[0], -canonical[2])
    assert np.allclose(body, (1.0, 0.0, -1.0), atol=1.0e-12)


def test_depth_covariance_rotation_preserves_trace_and_shape():
    cov = [4.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 16.0]
    rotated = rotate_covariance_3x3(cov, depth_mount_rotation(-90.0))
    assert len(rotated) == 9
    assert math.isclose(rotated[0] + rotated[4] + rotated[8], 29.0)
    assert all(
        math.isclose(rotated[3 * i + j], rotated[3 * j + i])
        for i in range(3)
        for j in range(3)
    )


def test_gate_accepts_only_trajectory_or_attitude_control_classes():
    common = dict(body_rate=False, thrust_and_torque=False, direct_actuator=False)
    assert selected_setpoint_kind(
        position=False, velocity=False, acceleration=False, attitude=True, **common
    ) == "attitude"
    assert selected_setpoint_kind(
        position=False, velocity=True, acceleration=False, attitude=False, **common
    ) == "trajectory"
    assert selected_setpoint_kind(
        position=False,
        velocity=False,
        acceleration=False,
        attitude=False,
        body_rate=True,
        thrust_and_torque=False,
        direct_actuator=False,
    ) is None
    assert selected_setpoint_kind(
        position=False, velocity=True, acceleration=False, attitude=True, **common
    ) is None


def test_attitude_passthrough_payload_validation():
    assert valid_attitude_payload((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, -0.5))
    assert not valid_attitude_payload((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, -0.5))
    assert not valid_attitude_payload((1.0, 0.0, 0.0, 0.0), (0.0, math.nan, -0.5))
    assert finite_vector((1.0, 2.0, 3.0), 3)


def test_real_strike_launch_requires_two_explicit_live_confirmations():
    launch_source = (
        Path(__file__).parents[1] / "launch" / "real_strike_v0.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("adapter_mode", default_value="shadow")' in launch_source
    assert 'DeclareLaunchArgument("allow_live_control", default_value="false")' in launch_source
    assert 'DeclareLaunchArgument("calibration_ready", default_value="false")' in launch_source
    assert 'LaunchConfiguration("allow_live_control")' in launch_source
    assert 'LaunchConfiguration("calibration_ready")' in launch_source
    assert '"search_canonical_tilt_deg": 0.0' in launch_source
    assert 'src="/fmu/in/vehicle_command"' in launch_source
    assert 'dst="/upstream/blocked/vehicle_command"' in launch_source


def test_field_live_terminal_requires_authorization_and_preserves_three_gates():
    launch_source = (
        Path(__file__).parents[1] / "launch" / "field_terminal_3_live.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("field_authorized", default_value="false")' in launch_source
    assert '"adapter_mode": "live"' in launch_source
    assert '"allow_live_control": "true"' in launch_source
    assert '"calibration_ready": "true"' in launch_source
    assert "REAL FLIGHT REFUSED" in launch_source


def test_field_terminal_launches_and_wrappers_are_complete():
    package_root = Path(__file__).parents[1]
    for number, name in (
        (1, "camera"),
        (2, "evidence"),
        (3, "live"),
        (4, "monitor"),
    ):
        assert (package_root / "launch" / f"field_terminal_{number}_{name}.launch.py").is_file()
        wrapper = package_root / "tools" / f"field_terminal_{number}_{name}.sh"
        assert wrapper.is_file()
        wrapper_source = wrapper.read_text(encoding="utf-8")
        assert "field_terminal_common.sh" in wrapper_source
        assert f"field_terminal_{number}_{name}.launch.py" in wrapper_source
