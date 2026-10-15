from pathlib import Path
import unittest


# 相对测试文件定位被测 launch（原先硬编码板端绝对路径
# /home/uavlab/Jetson_ROS2_V0/...，导致该测试离开板端必然失败）。
LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "field_terminal_1_record.launch.py"
)


def _load_launch_module(test_case):
    if not LAUNCH_PATH.exists():
        test_case.fail(f"recording launch does not exist: {LAUNCH_PATH}")

    import importlib.util
    from unittest.mock import patch

    spec = importlib.util.spec_from_file_location(
        "field_terminal_1_record_launch", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # 被测 launch 在模块执行期解析 falcon_intercept_deepstream 的 share 路径；
    # 该包需 CUDA/DeepStream 才能安装，离板环境用桩目录替代。Include 动作只被
    # 检查参数、不真正执行，桩路径无需存在。
    with patch(
        "ament_index_python.packages.get_package_share_directory",
        return_value="/tmp/falcon_intercept_deepstream_share",
    ):
        spec.loader.exec_module(module)
    return module


class RecordingLaunchTest(unittest.TestCase):
    def test_recording_launch_uses_headless_production_perception(self):
        from launch.actions import IncludeLaunchDescription

        module = _load_launch_module(self)
        description = module.generate_launch_description()
        include = next(
            action
            for action in description.entities
            if isinstance(action, IncludeLaunchDescription)
        )
        arguments = dict(include.launch_arguments)

        self.assertEqual(arguments["display"], "false")
        self.assertEqual(arguments["record"], "true")
        self.assertEqual(arguments["stereo_roi_enabled"], "true")
        self.assertEqual(arguments["confidence"], "0.35")
        self.assertEqual(arguments["record_segment_seconds"], "5")
        self.assertIn("record_output_dir", arguments)

    def test_recording_launch_contains_no_flight_command_path(self):
        if not LAUNCH_PATH.exists():
            self.fail(f"recording launch does not exist: {LAUNCH_PATH}")
        source = LAUNCH_PATH.read_text(encoding="utf-8")

        self.assertNotIn("VehicleCommand", source)
        self.assertNotIn("vehicle_command", source)
        self.assertNotIn("allow_live_control", source)
        self.assertNotIn("arming", source.lower())
        self.assertNotIn("takeoff", source.lower())

    def test_recording_launch_accepts_outer_output_directory(self):
        source = LAUNCH_PATH.read_text(encoding="utf-8")
        self.assertIn("FALCON_RECORD_OUTPUT_DIR", source)


if __name__ == "__main__":
    unittest.main()
