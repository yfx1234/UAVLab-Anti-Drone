#!/usr/bin/env python3
"""Prepare a deterministic provisional INT8 calibration set from camera recordings."""

import argparse
import json
from pathlib import Path

import cv2


def sample_indices(frame_count: int, requested: int) -> list[int]:
    if frame_count <= 0:
        raise RuntimeError("video does not expose a positive frame count")
    count = min(frame_count, requested)
    if count == 1:
        return [frame_count // 2]
    return [round(index * (frame_count - 1) / (count - 1)) for index in range(count)]


def deterministic_crop(image, sample_index: int):
    height, width = image.shape[:2]
    scales = (0.28, 0.40, 0.55, 0.70)
    scale = scales[sample_index % len(scales)]
    crop_width = max(32, int(width * scale))
    crop_height = max(32, int(height * scale))
    span_x = max(1, width - crop_width)
    span_y = max(1, height - crop_height)
    x = (sample_index * 104729) % span_x
    y = (sample_index * 130363) % span_y
    return image[y:y + crop_height, x:x + crop_width]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--search-video", type=Path, required=True)
    parser.add_argument("--left-video", type=Path, required=True)
    parser.add_argument("--right-video", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--frames-per-video", type=int, default=96)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()) and not args.force:
        raise FileExistsError(f"refusing to replace non-empty directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.force:
        for path in output_dir.glob("*.jpg"):
            path.unlink()

    sources = (
        ("search", args.search_video, False),
        ("left", args.left_video, True),
        ("right", args.right_video, True),
    )
    manifest = {"network_pre_input": [1280, 720], "sources": {}, "images": []}

    for source_name, video_path, include_roi in sources:
        video_path = video_path.expanduser().resolve()
        capture = cv2.VideoCapture(str(video_path))
        if not capture.isOpened():
            raise RuntimeError(f"cannot open video: {video_path}")
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        indices = sample_indices(frame_count, args.frames_per_video)
        written = 0
        for sample_number, frame_index in enumerate(indices):
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError(f"failed reading {video_path} frame {frame_index}")
            resized = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
            full_name = f"{source_name}_full_{sample_number:04d}.jpg"
            if not cv2.imwrite(str(output_dir / full_name), resized, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                raise RuntimeError(f"failed writing {full_name}")
            manifest["images"].append({"file": full_name, "source": source_name, "kind": "full"})
            written += 1

            if include_roi:
                crop = deterministic_crop(frame, sample_number)
                roi = cv2.resize(crop, (1280, 720), interpolation=cv2.INTER_LINEAR)
                roi_name = f"{source_name}_roi_{sample_number:04d}.jpg"
                if not cv2.imwrite(str(output_dir / roi_name), roi, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                    raise RuntimeError(f"failed writing {roi_name}")
                manifest["images"].append({"file": roi_name, "source": source_name, "kind": "roi"})
                written += 1
        capture.release()
        manifest["sources"][source_name] = {
            "video": str(video_path),
            "video_frames": frame_count,
            "calibration_images": written,
        }

    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"INT8_CALIBRATION_SET_PASS images={len(manifest['images'])} path={output_dir}")
    print(json.dumps(manifest["sources"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
