"""geometry_math 共享模块单测（同时钉住双链统一后的契约）。"""

import math

from falcon_intercept.geometry_math import low_pass, rotate_vector


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
SWAP_XY = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def test_rotate_vector_identity_and_swap():
    assert rotate_vector(IDENTITY, (1.0, -2.0, 3.0)) == (1.0, -2.0, 3.0)
    assert rotate_vector(SWAP_XY, (5.0, 7.0, 0.0)) == (7.0, 5.0, 0.0)


def test_rotate_vector_preserves_norm_for_rotation_matrix():
    rotation = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    result = rotate_vector(rotation, (3.0, 4.0, 5.0))
    assert math.isclose(sum(value * value for value in result), 50.0, rel_tol=1e-12)


def test_low_pass_seeds_holds_and_converges():
    assert low_pass(None, 5.0, 0.3, 0.1) == 5.0
    assert low_pass(3.0, 10.0, 0.3, 0.0) == 3.0
    value = 0.0
    for _ in range(200):
        value = low_pass(value, 10.0, 0.3, 0.1)
    assert math.isclose(value, 10.0, abs_tol=1e-6)


def test_low_pass_tau_zero_is_passthrough():
    assert low_pass(2.0, 9.0, 0.0, 0.1) == 9.0


def test_low_pass_negative_tau_degrades_to_passthrough_not_divergence():
    # 统一后的防护契约：tau<0 视为 0（直通），不得除零或外推放大。
    assert low_pass(2.0, 9.0, -0.1, 0.1) == 9.0
    assert low_pass(2.0, 9.0, -0.1, 0.1) == low_pass(2.0, 9.0, 0.0, 0.1)
