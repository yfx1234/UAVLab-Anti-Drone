#!/usr/bin/env python3
"""track_hold_node 纯函数区单元测试:速度律、限幅、指向、状态机。"""

import math
import unittest

import importlib.util
import pathlib

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "track_hold_node.py"
spec = importlib.util.spec_from_file_location("track_hold_node", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

PARAMS = {
    "hold_min_m": 6.5,
    "hold_max_m": 8.0,
    "approach_exit_m": 8.5,
    "aim_error_threshold_deg": 5.0,
}


class TestVelocityLaw(unittest.TestCase):
    def test_far_target_commands_approach(self):
        # 目标在正上方偏北 15 m:应命令向北+向上接近。
        rel = [10.0, 0.0, -12.0]  # NED:目标在北 10 m、上 12 m。
        velocity, distance = module.velocity_towards_hold(
            rel, [0.0, 0.0, 0.0], 7.5, 2.5, 0.8, 3.0, 1.5)
        self.assertAlmostEqual(distance, math.hypot(math.hypot(10, 0), 12), places=9)
        self.assertGreater(velocity[0], 0.0)   # 北向分量:靠近目标
        self.assertLess(velocity[2], 0.0)      # 上(负下):爬升
        speed = math.hypot(velocity[0], velocity[1])
        self.assertLessEqual(speed, 3.0 + 1e-9)

    def test_at_hold_distance_zero_velocity(self):
        # 恰好在斜下方保持点(后方 2.5 m、下方 sqrt(7.5^2-2.5^2)):
        # 目标相对本机 = 北 2.5 m、上方 7.07 m。
        vertical = math.sqrt(7.5 ** 2 - 2.5 ** 2)
        rel = [2.5, 0.0, -vertical]
        velocity, _ = module.velocity_towards_hold(
            rel, [0.0, 0.0, 0.0], 7.5, 2.5, 0.8, 3.0, 1.5)
        for axis in velocity:
            self.assertAlmostEqual(axis, 0.0, places=9)

    def test_hold_point_has_horizontal_offset(self):
        # 斜下方保持点必须有水平分量,否则机头指向无参考。
        vertical = math.sqrt(7.5 ** 2 - 2.5 ** 2)
        self.assertGreater(2.5, 0.0)
        self.assertAlmostEqual(math.hypot(2.5, vertical), 7.5, places=9)

    def test_directly_below_target_moves_back(self):
        # 正下方(视线竖直)时:让出水平偏移。水平缺省单位取北,期望
        # rel_x=2.5,当前 0 -> 机体向南退,使目标出现在机体北 2.5 m。
        rel = [0.0, 0.0, -7.5]
        velocity, _ = module.velocity_towards_hold(
            rel, [0.0, 0.0, 0.0], 7.5, 2.5, 0.8, 3.0, 1.5)
        self.assertLess(velocity[0], 0.0)

    def test_target_velocity_feedforward(self):
        # 目标移动时前馈:即使已在保持点也跟随。
        vertical = math.sqrt(7.5 ** 2 - 2.5 ** 2)
        rel = [2.5, 0.0, -vertical]
        velocity, _ = module.velocity_towards_hold(
            rel, [1.2, 0.0, 0.0], 7.5, 2.5, 0.8, 3.0, 1.5)
        self.assertAlmostEqual(velocity[0], 1.2, places=9)

    def test_too_close_backs_off(self):
        # 5 m:过近,速度应沿视线远离目标(rel 指向目标,远离为 -unit)。
        rel = [0.0, 0.0, -5.0]
        velocity, _ = module.velocity_towards_hold(
            rel, [0.0, 0.0, 0.0], 7.5, 2.5, 0.8, 3.0, 1.5)
        self.assertGreater(velocity[2], 0.0)   # 向下(远离上方目标)

    def test_vertical_limit(self):
        rel = [0.0, 0.0, -20.0]
        velocity, _ = module.velocity_towards_hold(
            rel, [0.0, 0.0, 0.0], 7.5, 2.5, 0.8, 3.0, 1.5)
        self.assertGreaterEqual(velocity[2], -1.5 - 1e-9)

    def test_back_zero_falls_back_to_bearing_hold(self):
        # hold_back=0:纯视线保持,恰在 7.5 m 处速度为零。
        rel = [0.0, 0.0, -7.5]
        velocity, _ = module.velocity_towards_hold(
            rel, [0.0, 0.0, 0.0], 7.5, 0.0, 0.8, 3.0, 1.5)
        for axis in velocity:
            self.assertAlmostEqual(axis, 0.0, places=9)


class TestLimits(unittest.TestCase):
    def test_rate_limit_bounds_acceleration(self):
        command = [10.0, 0.0, 0.0]
        previous = [0.0, 0.0, 0.0]
        limited = module.rate_limit_velocity(command, previous, 0.033, 4.0, 2.0)
        self.assertAlmostEqual(limited[0], 4.0 * 0.033, places=9)

    def test_rate_limit_passes_small_changes(self):
        limited = module.rate_limit_velocity(
            [0.1, 0.0, -0.1], [0.0, 0.0, 0.0], 0.033, 4.0, 2.0)
        self.assertAlmostEqual(limited[0], 0.1, places=9)

    def test_retreat_direction(self):
        retreat = module.retreat_velocity([0.0, 0.0, -6.0], 1.0)
        norm = math.sqrt(sum(v * v for v in retreat))
        self.assertAlmostEqual(norm, 1.0, places=9)
        self.assertGreater(retreat[2], 0.0)


class TestAiming(unittest.TestCase):
    def test_yaw_north_and_east(self):
        self.assertAlmostEqual(module.yaw_towards([5.0, 0.0, 0.0]), 0.0, places=9)
        self.assertAlmostEqual(
            module.yaw_towards([0.0, 5.0, 0.0]), math.pi / 2, places=9)

    def test_aim_error_wraps(self):
        error = module.aim_error_rad([5.0, 0.0, 0.0], math.pi - 0.1)
        self.assertAlmostEqual(error, -(math.pi - 0.1), places=9)


class TestStateMachine(unittest.TestCase):
    def test_search_to_approach(self):
        state, actions = module.step_state_machine(
            "SEARCH", track_valid=True, range_m=15.0,
            aim_error_rad_value=0.0, fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "APPROACH")

    def test_search_to_hold_when_near(self):
        state, _ = module.step_state_machine(
            "SEARCH", track_valid=True, range_m=7.5,
            aim_error_rad_value=0.0, fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "HOLD")

    def test_hold_to_aim_when_aligned(self):
        state, _ = module.step_state_machine(
            "HOLD", track_valid=True, range_m=7.5,
            aim_error_rad_value=math.radians(2.0),
            fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "AIM")

    def test_hold_stays_when_misaligned(self):
        state, _ = module.step_state_machine(
            "HOLD", track_valid=True, range_m=7.5,
            aim_error_rad_value=math.radians(20.0),
            fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "HOLD")

    def test_hold_retreats_when_too_close(self):
        state, actions = module.step_state_machine(
            "HOLD", track_valid=True, range_m=6.0,
            aim_error_rad_value=0.0, fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "HOLD")
        self.assertIn("retreat", actions)

    def test_hold_to_approach_when_far(self):
        state, _ = module.step_state_machine(
            "HOLD", track_valid=True, range_m=10.0,
            aim_error_rad_value=0.0, fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "APPROACH")

    def test_lost_on_track_drop(self):
        state, _ = module.step_state_machine(
            "HOLD", track_valid=False, range_m=7.5,
            aim_error_rad_value=0.0, fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "LOST")

    def test_reacquire_from_lost(self):
        state, _ = module.step_state_machine(
            "LOST", track_valid=True, range_m=7.5,
            aim_error_rad_value=0.0, fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "HOLD")

    def test_fire_then_cooldown_blocks_aim(self):
        state, actions = module.step_state_machine(
            "FIRE", track_valid=True, range_m=7.5,
            aim_error_rad_value=0.0, fire_cooldown_active=True, params=PARAMS)
        self.assertEqual(actions, ["emit_shot"])
        self.assertEqual(state, "HOLD")

    def test_aim_exits_when_misaligned(self):
        state, _ = module.step_state_machine(
            "AIM", track_valid=True, range_m=7.5,
            aim_error_rad_value=math.radians(10.0),
            fire_cooldown_active=False, params=PARAMS)
        self.assertEqual(state, "HOLD")


if __name__ == "__main__":
    unittest.main()
