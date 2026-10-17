import importlib.util
import sys
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "realtime_attitude_monitor.py"
SPEC = importlib.util.spec_from_file_location("realtime_attitude_monitor", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_identity_quaternion_has_zero_euler_and_identity_rotation():
    np.testing.assert_allclose(MODULE.quaternion_to_euler([1, 0, 0, 0]), np.zeros(3))
    np.testing.assert_allclose(MODULE.quaternion_rotation_matrix([1, 0, 0, 0]), np.eye(3))


def test_yaw_is_unwrapped_across_pi_boundary():
    buffer = MODULE.AttitudeBuffer(10.0)
    for time, yaw in ((1.0, np.deg2rad(179.0)), (2.0, np.deg2rad(-179.0))):
        buffer.add_attitude(time, [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    yaw_values = buffer.snapshot(2.0).euler[:, 2]
    assert np.rad2deg(yaw_values[1] - yaw_values[0]) == pytest.approx(2.0)


def test_rate_derivative_is_reported_as_smoothness_metric():
    buffer = MODULE.AttitudeBuffer(10.0)
    buffer.add_attitude(1.0, [1, 0, 0, 0])
    buffer.add_rates(1.0, [0, 0, 0])
    buffer.add_attitude(1.5, [1, 0, 0, 0])
    buffer.add_rates(1.5, [1, 0, 0])
    assert buffer.snapshot(1.5).angular_acceleration[-1] == pytest.approx(2.0)


import pytest  # noqa: E402
