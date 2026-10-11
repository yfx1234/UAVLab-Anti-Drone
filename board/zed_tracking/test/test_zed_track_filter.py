#!/usr/bin/env python3
"""zed_track_filter_node 纯函数区单元测试。"""

import math
import unittest

import numpy as np

import importlib.util
import pathlib
import sys

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "zed_track_filter_node.py"
spec = importlib.util.spec_from_file_location("zed_track_filter_node", SCRIPT)
module = importlib.util.module_from_spec(spec)
# 纯函数区不依赖 rclpy/px4_msgs,导入应当成功。
spec.loader.exec_module(module)

quat = module.quat_wxyz_to_matrix
flu_to_frd = module.body_flu_to_frd
to_ned = module.transform_body_to_ned
absolute = module.observation_absolute_position
sigma = module.observation_sigma
extrapolate = module.extrapolate_state
parse = module.parse_target_message


class TestQuaternion(unittest.TestCase):
    def test_identity(self):
        rotation = quat((1.0, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(rotation, np.eye(3), atol=1e-12)

    def test_quarter_turn_yaw_maps_forward_to_east(self):
        # 机体FRD->NED 偏航 +90°(向右转):机头(前,x_frd=1)指向东。
        half = math.sqrt(0.5)
        rotation = quat((half, 0.0, 0.0, half))
        result = rotation @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(result, [0.0, 1.0, 0.0], atol=1e-12)


class TestFrameTransforms(unittest.TestCase):
    def test_flu_to_frd(self):
        np.testing.assert_allclose(
            flu_to_frd([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0], atol=1e-12
        )

    def test_identity_attitude(self):
        result = to_ned((1.0, 0.0, 0.0, 0.0), [1.0, 2.0, 3.0])
        np.testing.assert_allclose(result, [1.0, -2.0, -3.0], atol=1e-12)

    def test_yaw_attitude(self):
        # 相同点在前右下系验证:机体正上方 3 m(FLU z=3 -> FRD -3)偏航 90° 后仍在 -D。
        half = math.sqrt(0.5)
        result = to_ned((half, 0.0, 0.0, half), [0.0, 0.0, 3.0])
        np.testing.assert_allclose(result, [0.0, 0.0, -3.0], atol=1e-12)

    def test_absolute_position_adds_own_offset(self):
        # 机体在 (5, -2, 1),目标在机体正上方 8 m(FLU z=8):
        # 绝对位置 = 自身 + 相对。
        result = absolute(
            (1.0, 0.0, 0.0, 0.0), [0.0, 0.0, 8.0], (5.0, -2.0, 1.0))
        np.testing.assert_allclose(result, [5.0, -2.0, -7.0], atol=1e-12)

    def test_absolute_position_with_yaw(self):
        # 机体偏航 90°、自身位于原点,目标在机头前(FLU x=4, 前右下系 x=4):
        # NED 下指向东。
        half = math.sqrt(0.5)
        result = absolute(
            (half, 0.0, 0.0, half), [4.0, 0.0, 0.0], (0.0, 0.0, 0.0))
        np.testing.assert_allclose(result, [0.0, 4.0, 0.0], atol=1e-12)


class TestObservationSigma(unittest.TestCase):
    def test_depth_mode_grows_with_range(self):
        near = sigma("depth", 5.0, 1266.5, 2.0, 0.15, 0.06, 0.5, 0.25)
        far = sigma("depth", 10.0, 1266.5, 2.0, 0.15, 0.06, 0.5, 0.25)
        self.assertTrue(np.all(far > near))
        # 5 m 深度:0.15+0.06*5=0.45;横向约 2/1266.5*5=0.0079。
        self.assertAlmostEqual(near[2], 0.45, places=6)
        self.assertLess(near[0], 0.02)

    def test_pseudo_mode_much_larger(self):
        pseudo = sigma("pseudo", 10.0, 1266.5, 2.0, 0.15, 0.06, 0.5, 0.25)
        depth = sigma("depth", 10.0, 1266.5, 2.0, 0.15, 0.06, 0.5, 0.25)
        self.assertTrue(np.all(pseudo > depth))
        # 10 m 伪测距各向同性 0.5+0.25*10=3.0 m。
        np.testing.assert_allclose(pseudo, [3.0, 3.0, 3.0], atol=1e-9)


class TestExtrapolate(unittest.TestCase):
    def test_zero_dt_returns_copy(self):
        state = np.array([1.0, 2.0, 3.0, 0.5, -0.5, 0.0])
        covariance = np.eye(6) * 0.1
        state_now, covariance_now = extrapolate(state, covariance, 0.0, 4.0)
        np.testing.assert_allclose(state_now, state)
        np.testing.assert_allclose(covariance_now, covariance)
        # 必须是拷贝,改返回值不影响输入。
        state_now[0] = 99.0
        self.assertNotEqual(state[0], 99.0)

    def test_moves_by_velocity(self):
        state = np.array([0.0, 0.0, 0.0, 1.0, 2.0, -3.0])
        state_now, _ = extrapolate(state, np.eye(6) * 0.01, 0.5, 4.0)
        np.testing.assert_allclose(state_now[0:3], [0.5, 1.0, -1.5], atol=1e-12)

    def test_covariance_grows(self):
        covariance = np.eye(6) * 0.01
        _, covariance_now = extrapolate(
            np.zeros(6), covariance, 1.0, 4.0)
        self.assertGreater(float(np.trace(covariance_now)), float(np.trace(covariance)))


class TestParseTargetMessage(unittest.TestCase):
    def test_none_mode(self):
        stamp, mode, body, _ = parse('{"stamp_s":1.0,"sequence":5,"mode":"none"}')
        self.assertEqual(mode, "none")
        self.assertIsNone(body)

    def test_depth_payload(self):
        text = (
            '{"stamp_s":100.5,"mode":"depth","body_xyz":[0.1,0.0,5.12],'
            '"range_m":5.12}'
        )
        stamp, mode, body, range_m = parse(text)
        self.assertAlmostEqual(stamp, 100.5)
        self.assertEqual(mode, "depth")
        self.assertEqual(body, [0.1, 0.0, 5.12])
        self.assertAlmostEqual(range_m, 5.12)

    def test_invalid_json(self):
        _, mode, _, _ = parse("not-json")
        self.assertEqual(mode, "invalid")

    def test_missing_body(self):
        _, mode, _, _ = parse('{"stamp_s":1.0,"mode":"depth"}')
        self.assertEqual(mode, "invalid")


if __name__ == "__main__":
    unittest.main()
