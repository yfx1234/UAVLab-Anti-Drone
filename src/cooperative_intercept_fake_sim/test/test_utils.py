"""cooperative_intercept_fake_sim.utils 单测（2026-10-07 新增，钉住共用工具契约）。"""

import numpy as np

from cooperative_intercept_fake_sim.utils import (
    clamp,
    clamp_f,
    normalize,
    quat_normalize,
    quat_to_R,
    safe_atan2,
)


def test_clamp_and_clamp_f():
    assert clamp(1.5, 0.0, 1.0) == 1.0
    assert clamp(-1.5, 0.0, 1.0) == 0.0
    assert clamp_f(-3.0) == 0.0
    assert clamp_f(0.5) == 0.5
    assert clamp_f(4.0) == 1.0


def test_normalize_zero_safe():
    assert np.allclose(normalize([0.0, 0.0, 0.0]), 0.0)
    assert np.isclose(np.linalg.norm(normalize([3.0, 4.0, 0.0])), 1.0)


def test_safe_atan2_fallback():
    assert safe_atan2(0.0, 0.0) == 0.0
    assert safe_atan2(0.0, 0.0, fallback=0.25) == 0.25
    assert np.isclose(safe_atan2(1.0, 1.0), np.pi / 4.0)


def test_quat_roundtrip_yaw():
    yaw = 0.7
    q = quat_normalize([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])
    r = quat_to_R(q)
    heading = np.arctan2(r[1, 0], r[0, 0])
    assert np.isclose(heading, yaw)
