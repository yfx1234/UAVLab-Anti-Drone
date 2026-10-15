import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tools" / "compare_shadow_outputs.py"
SPEC = importlib.util.spec_from_file_location("shadow_compare", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def row(t, state="SEARCH", velocity=(0.0, 0.0, 0.0), yaw=0.2, land=False):
    return {"timestamp_s": t, "state": state, "velocity_flu": velocity,
            "yaw_rate_flu": yaw, "land_request": land, "reason": "test"}


def test_alignment_and_differences_are_reported():
    upstream = [row(1.0), row(1.1, state="TRACK", yaw=0.0)]
    jetson = [row(1.01), row(1.11, state="APPROACH", velocity=(0.1, 0.0, 0.0), yaw=0.0)]
    report = MODULE.compare(upstream, jetson, tolerance_s=0.02)
    assert report["aligned_samples"] == 2
    assert report["state_mismatch_count"] == 1
    assert abs(report["velocity_vector_rmse_mps"] - (0.01 / 2) ** 0.5) < 1e-12


def test_no_alignment_is_rejected():
    try:
        MODULE.compare([row(1.0)], [row(2.0)], tolerance_s=0.01)
    except ValueError as error:
        assert "no samples aligned" in str(error)
    else:
        raise AssertionError("expected no-alignment rejection")
