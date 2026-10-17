import csv
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "trajectory_plotting.py"
SPEC = importlib.util.spec_from_file_location("trajectory_plotting", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_plot_record_generates_both_images(tmp_path):
    csv_path = tmp_path / "record.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=MODULE.REQUIRED_COLUMNS)
        writer.writeheader()
        for index in range(5):
            writer.writerow(
                {
                    "time_s": 10.0 + index,
                    "target_n_m": index,
                    "target_e_m": 0.5 * index,
                    "target_d_m": -12.0,
                    "pursuer_n_m": 0.5 * index,
                    "pursuer_e_m": 0.0,
                    "pursuer_d_m": -5.0,
                    "camera_u_px": 300.0 + index,
                    "camera_v_px": 240.0 - index,
                    "camera_depth_m": 7.0,
                    "camera_visible": 1,
                    "estimate_valid": 1,
                }
            )
    outputs = MODULE.plot_record(csv_path, tmp_path / "plots")
    assert all(path.exists() and path.stat().st_size > 0 for path in outputs)


def test_load_record_rejects_missing_columns(tmp_path):
    csv_path = tmp_path / "bad.csv"
    csv_path.write_text("time_s\n1.0\n", encoding="utf-8")
    try:
        MODULE.load_record(csv_path)
    except ValueError as error:
        assert "missing columns" in str(error)
    else:
        raise AssertionError("missing columns were accepted")


def test_capture_event_indices_use_held_rising_edges(tmp_path):
    csv_path = tmp_path / "captures.csv"
    fieldnames = list(MODULE.REQUIRED_COLUMNS) + [
        "pursuer_yaw_rad",
        "capture_held",
        "capture_count",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        held_values = [0, 1, 1, 0, 1, 1]
        for index, held in enumerate(held_values):
            writer.writerow(
                {
                    "time_s": 10.0 + index * 0.02,
                    "target_n_m": index,
                    "target_e_m": 1.0,
                    "target_d_m": -12.0,
                    "pursuer_n_m": index - 1.0,
                    "pursuer_e_m": 0.0,
                    "pursuer_d_m": -5.0,
                    "pursuer_yaw_rad": 0.25,
                    "camera_u_px": 300.0 + index,
                    "camera_v_px": 240.0 - index,
                    "camera_depth_m": 7.0,
                    "camera_visible": 1,
                    "estimate_valid": 1,
                    "capture_held": held,
                    "capture_count": 1 if index < 4 else 2,
                }
            )
    data = MODULE.load_record(csv_path)
    assert MODULE.capture_event_indices(data).tolist() == [1, 4]
    outputs = MODULE.plot_record(csv_path, tmp_path / "capture_plots")
    assert all(path.exists() and path.stat().st_size > 0 for path in outputs)


def test_dual_camera_projection_matches_body_frd_mapping(tmp_path):
    csv_path = tmp_path / "projection.csv"
    fieldnames = list(MODULE.REQUIRED_COLUMNS) + [
        "pursuer_q_w",
        "pursuer_q_x",
        "pursuer_q_y",
        "pursuer_q_z",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(
            {
                "time_s": 0.0,
                "target_n_m": 0.0,
                "target_e_m": 0.0,
                "target_d_m": -10.0,
                "pursuer_n_m": 0.0,
                "pursuer_e_m": 0.0,
                "pursuer_d_m": 0.0,
                "pursuer_q_w": 1.0,
                "pursuer_q_x": 0.0,
                "pursuer_q_y": 0.0,
                "pursuer_q_z": 0.0,
                "camera_u_px": 1919.5,
                "camera_v_px": 1079.5,
                "camera_depth_m": 10.0,
                "camera_visible": 1,
                "estimate_valid": 1,
            }
        )
    tracks = MODULE.simulated_camera_tracks(MODULE.load_record(csv_path))
    assert tracks["depth_visible"].tolist() == [True]
    assert abs(tracks["depth_u_px"][0] - 1919.5) < 1e-6
    assert abs(tracks["depth_v_px"][0] - 1079.5) < 1e-6
    assert tracks["search_visible"].tolist() == [True]
    assert tracks["search_v_px"][0] < 1079.5


def test_plot_run_archives_raw_files_but_leaves_images_outside(tmp_path):
    csv_path = tmp_path / "record.csv"
    log_path = tmp_path / "tracking.log"
    log_path.write_text("test", encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=MODULE.REQUIRED_COLUMNS)
        writer.writeheader()
        writer.writerow(
            {
                "time_s": 0.0,
                "target_n_m": 0.0,
                "target_e_m": 0.0,
                "target_d_m": -10.0,
                "pursuer_n_m": 0.0,
                "pursuer_e_m": 0.0,
                "pursuer_d_m": 0.0,
                "camera_u_px": 1919.5,
                "camera_v_px": 1079.5,
                "camera_depth_m": 10.0,
                "camera_visible": 1,
                "estimate_valid": 1,
            }
        )
    outputs = MODULE.plot_run(csv_path, tmp_path)
    assert all(path.parent == tmp_path and path.exists() for path in outputs)
    assert (tmp_path / "原始数据" / "CSV" / "record.csv").exists()
    assert (tmp_path / "原始数据" / "日志" / "tracking.log").exists()
    assert not csv_path.exists()
    assert not log_path.exists()
