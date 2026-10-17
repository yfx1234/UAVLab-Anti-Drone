import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "field_terminal_stitch_recording.py"


def load_module():
    spec = importlib.util.spec_from_file_location("field_terminal_stitch_recording", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class StitchRecordingTest(unittest.TestCase):
    def test_collects_only_nonempty_numbered_segments_in_order(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "front_yolo_00002.mkv").write_bytes(b"2")
            (root / "front_yolo_00000.mkv").write_bytes(b"0")
            (root / "front_yolo_00001.mkv").write_bytes(b"1")
            (root / "front_yolo_full.mkv").write_bytes(b"old")
            paths = module.collect_segments(root, "front")
            self.assertEqual([path.name for path in paths], [
                "front_yolo_00000.mkv",
                "front_yolo_00001.mkv",
                "front_yolo_00002.mkv",
            ])

    def test_build_command_uses_splitmuxsrc_and_continuous_output(self):
        module = load_module()
        command = module.build_gst_command(
            [Path("/records/front_yolo_00000.mkv")],
            Path("/records/front_yolo_full.mkv"),
            25,
        )
        rendered = " ".join(command)
        self.assertIn("splitmuxsrc", rendered)
        self.assertIn("front_yolo_?????.mkv", rendered)
        self.assertIn("avdec_h264", rendered)
        self.assertIn("front_yolo_full.mkv", rendered)

    def test_legacy_left_right_recordings_fall_back(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "left_yolo_00001.mkv").write_bytes(b"1")
            (root / "left_yolo_00000.mkv").write_bytes(b"0")
            (root / "right_yolo_00000.mkv").write_bytes(b"0")
            front = module.collect_segments(root, "front")
            back = module.collect_segments(root, "back")
            self.assertEqual([path.name for path in front], [
                "left_yolo_00000.mkv",
                "left_yolo_00001.mkv",
            ])
            self.assertEqual([path.name for path in back], ["right_yolo_00000.mkv"])


if __name__ == "__main__":
    unittest.main()
