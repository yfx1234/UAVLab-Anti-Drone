import csv
import importlib.util
from pathlib import Path


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts" / "p7_acceptance_plotting.py"
)
SPEC = importlib.util.spec_from_file_location("p7_acceptance_plotting", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _row(stamp, state, state_name, count, range_m, error_rad, expanded=True):
    row = [
        int(stamp),
        int((stamp - int(stamp)) * 1e9),
        "map_ned",
        state,
        state_name,
        "test",
        "True",
        "True",
        "True",
        "True",
        "True",
        "True",
        "True",
        count,
        2,
        range_m,
        0.5,
        error_rad,
        0.01,
        0.01,
        0.01,
    ]
    if expanded:
        row.extend([0.017, error_rad, 0.005, "False"])
    return row


def test_load_status_and_capture_edges(tmp_path):
    path = tmp_path / "status.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(_row(10.0, 5, "INTERCEPT", 0, 25.0, 0.02))
        writer.writerow(_row(10.2, 7, "EGRESS_REATTACK", 1, 24.8, 0.01))
        writer.writerow(_row(14.0, 7, "EGRESS_REATTACK", 2, 25.3, 0.01))
    status = MODULE.load_status_csv(path)
    assert status["state_name"].tolist() == [
        "INTERCEPT",
        "EGRESS_REATTACK",
        "EGRESS_REATTACK",
    ]
    assert MODULE.capture_indices(status).tolist() == [1, 2]
    assert abs(status["time_s"][-1] - 4.0) < 1e-6
    assert status["expanded_axis_fields"].tolist() == [True]
    assert abs(status["actual_attack_axis_error_deg"][1] - 0.5729578) < 1e-5


def test_attack_axis_identity_points_up_in_ned():
    import numpy as np

    axis = MODULE.attack_axis_ned(np.asarray([[1.0, 0.0, 0.0, 0.0]]))
    assert np.allclose(axis, [[0.0, 0.0, -1.0]])
    body_axes = MODULE.body_axes_ned(
        np.asarray([[1.0, 0.0, 0.0, 0.0]])
    )
    assert np.allclose(body_axes[0], np.eye(3))
    assert np.allclose(
        MODULE.quaternion_euler_deg(np.asarray([1.0, 0.0, 0.0, 0.0])),
        (0.0, 0.0, 0.0),
    )
