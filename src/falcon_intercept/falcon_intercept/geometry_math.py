"""Falcon 双链（search_servo / contact）共享的纯数学函数。

只收录两链契约完全一致、且已被两侧测试覆盖锚定的函数；
实现取自 falcon_intercept/falcon_intercept/search_servo_node.py 与
falcon_contact/falcon_contact/contact_logic.py 的原实现（行为不变）。

注意：同名但契约已分叉的函数不放入本模块，留在各自模块并互相注释：
- wrap_pi：servo 版（循环）在 3π 边界返回 +π，contact 版（取模）返回 -π，
  test_search_servo.py 已钉住 +π 行为；
- quaternion_to_rotation_matrix：退化四元数时 servo 返回单位阵、contact 抛
  ValueError，故障策略相反；
- apply_lens_tilt：contact 版对结果归一化，servo 版不归一化。
"""

from __future__ import annotations


def low_pass(
    previous: float | None, value: float, tau_s: float, dt_s: float
) -> float:
    """First-order low pass; unseeded passes the value through, dt<=0 holds.

    ``tau_s`` is clamped to >= 0 so a negative time constant degrades to a
    pass-through instead of dividing by zero or extrapolating.
    """
    value = float(value)
    if previous is None:
        return value
    if float(dt_s) <= 0.0:
        return float(previous)
    alpha = float(dt_s) / (max(0.0, float(tau_s)) + float(dt_s))
    return float(previous) + alpha * (value - float(previous))


def rotate_vector(
    rotation: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Apply a 3x3 row-major rotation to a vector."""
    x, y, z = (float(value) for value in vector)
    return tuple(
        rotation[row][0] * x + rotation[row][1] * y + rotation[row][2] * z
        for row in range(3)
    )  # type: ignore[return-value]
