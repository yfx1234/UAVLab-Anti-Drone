#!/usr/bin/env python3
"""Join splitmuxsink recording fragments into one playable file per camera."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path


SEGMENT_RE = re.compile(r"^(front|back|search|left|right)_yolo_(\d{5})\.mkv$")
SEGMENT_GLOB = "{camera}_yolo_?????.mkv"
FPS = {"front": 25, "back": 25, "search": 15, "left": 25, "right": 25}
# Recordings made before the 2026-09-03 front/back rename used left/right names.
LEGACY_CAMERA = {"front": "left", "back": "right"}


def collect_segments(root: Path, camera: str) -> list[Path]:
    """Return non-empty numbered fragments in numeric filename order."""
    segments = []
    for path in root.glob(f"{camera}_yolo_*.mkv"):
        match = SEGMENT_RE.fullmatch(path.name)
        if match and path.stat().st_size > 0:
            segments.append(path)
    if not segments and camera in LEGACY_CAMERA:
        return collect_segments(root, LEGACY_CAMERA[camera])
    return sorted(segments, key=lambda path: int(SEGMENT_RE.fullmatch(path.name).group(2)))


def build_gst_command(segments: list[Path], output: Path, fps: int) -> list[str]:
    """Build a decode/re-encode pipeline that resets the output timeline."""
    if not segments:
        raise ValueError("at least one segment is required")
    root = segments[0].parent
    camera = SEGMENT_RE.fullmatch(segments[0].name).group(1)
    pattern = str(root / SEGMENT_GLOB.format(camera=camera))
    return [
        "gst-launch-1.0", "-e",
        "splitmuxsrc", f"location={pattern}", "!",
        "h264parse", "!", "avdec_h264", "!", "videoconvert", "!",
        "videoscale", "!", f"video/x-raw,width=960,height=540,framerate={fps}/1", "!",
        "x264enc", "speed-preset=ultrafast", "tune=zerolatency",
        "bitrate=4000", "key-int-max=30", "bframes=0", "!",
        "h264parse", "!", "matroskamux", "streamable=true", "!",
        "filesink", f"location={output}",
    ]


def quarantine_empty_segments(root: Path) -> None:
    """Move 0-byte splitmuxsink leftovers aside.

    A recording killed mid-write leaves empty numbered fragments; splitmuxsrc
    fails to preroll the whole stitch as soon as its glob enumerates one.
    """
    for path in root.glob("*_yolo_*.mkv"):
        if path.is_file() and path.stat().st_size == 0:
            path.rename(path.with_suffix(".mkv.empty"))


def stitch_camera(root: Path, camera: str, output: Path) -> None:
    quarantine_empty_segments(root)
    segments = collect_segments(root, camera)
    if not segments:
        raise RuntimeError(f"no non-empty {camera} segments found in {root}")
    output.unlink(missing_ok=True)
    command = build_gst_command(segments, output, FPS[camera])
    log_path = root / f"{camera}_stitch.log"
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
    if result.returncode != 0 or not output.exists() or output.stat().st_size == 0:
        raise RuntimeError(f"GStreamer stitch failed for {camera}; see {log_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    root = args.input_dir.resolve()
    output_dir = (args.output_dir or root).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if shutil.which("gst-launch-1.0") is None:
        print("gst-launch-1.0 is required for stitching", file=sys.stderr)
        return 2
    failures = []
    for camera in ("front", "back", "search"):
        try:
            stitch_camera(root, camera, output_dir / f"{camera}_yolo_full.mkv")
            print(f"STITCH_OK camera={camera} output={output_dir / f'{camera}_yolo_full.mkv'}")
        except RuntimeError as error:
            failures.append(str(error))
            print(f"STITCH_FAIL {error}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
