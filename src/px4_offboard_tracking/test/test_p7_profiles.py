import importlib.util
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = ROOT / "launch" / "p7_interceptor_sim.launch.py"
SPEC = importlib.util.spec_from_file_location("p7_interceptor_sim_launch", LAUNCH_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_all_profiles_preserve_distance_window_and_safe_speed_margin():
    config = ROOT / "config" / "high_speed_profiles.yaml"
    previous_target_speed = -math.inf
    previous_pursuer_speed = -math.inf
    for profile_name in ("v0", "v1", "v2", "v3", "v4", "v5"):
        case = MODULE.load_case(config, profile_name, "escape")
        assert case["minimum_range_m"] == 15.0
        assert case["maximum_range_m"] == 35.0
        assert 15.0 <= case["preferred_range_m"] <= 35.0
        assert case["pursuer_command_speed_mps"] < case["pursuer_safety_speed_mps"]
        assert case["first_attack_tracking_hold_s"] >= 3.0
        assert case["target_speed_mps"] > previous_target_speed
        assert case["pursuer_command_speed_mps"] >= previous_pursuer_speed
        previous_target_speed = case["target_speed_mps"]
        previous_pursuer_speed = case["pursuer_command_speed_mps"]


def test_initial_geometry_matches_configured_slant_range_and_search_fov():
    config = ROOT / "config" / "high_speed_profiles.yaml"
    for profile_name in ("v0", "v1", "v2", "v3", "v4", "v5"):
        case = MODULE.load_case(config, profile_name, "escape")
        vertical = (
            case["target_height_m"] - case["pursuer_takeoff_height_m"]
        )
        horizontal = case["initial_horizontal_offset_m"]
        assert math.hypot(horizontal, vertical) == pytest.approx(
            case["initial_range_m"], abs=0.02
        )
        assert math.atan2(horizontal, vertical) < math.radians(40.0)


def test_stationary_is_only_zero_speed_case():
    config = ROOT / "config" / "high_speed_profiles.yaml"
    stationary = MODULE.load_case(config, "v0", "stationary")
    escape = MODULE.load_case(config, "v0", "escape")
    assert stationary["resolved_target_speed_mps"] == 0.0
    assert escape["resolved_target_speed_mps"] == 2.0
    assert escape["target_heading_rad"] == pytest.approx(math.pi / 2.0)


def test_v1_maneuver_geometry_overrides_are_loaded():
    config = ROOT / "config" / "high_speed_profiles.yaml"
    circle = MODULE.load_case(config, "v1", "circle")
    figure_eight = MODULE.load_case(config, "v1", "figure_eight")
    straight = MODULE.load_case(config, "v1", "straight")
    assert straight["initial_horizontal_offset_m"] == pytest.approx(3.5)
    assert circle["target_radius_m"] == pytest.approx(7.5)
    assert circle["initial_horizontal_offset_m"] == pytest.approx(3.5)
    assert circle["egress_distance_m"] == pytest.approx(2.0)
    assert circle["egress_speed_mps"] == pytest.approx(0.5)
    assert figure_eight["target_path_length_m"] == pytest.approx(20.0)
    assert figure_eight["target_path_width_m"] == pytest.approx(10.0)
    assert figure_eight["initial_horizontal_offset_m"] == pytest.approx(3.5)
    assert figure_eight["egress_distance_m"] == pytest.approx(2.0)


def test_profile_rejects_unapproved_encounter():
    config = ROOT / "config" / "high_speed_profiles.yaml"
    with pytest.raises(ValueError):
        MODULE.load_case(config, "v1", "head_on")
