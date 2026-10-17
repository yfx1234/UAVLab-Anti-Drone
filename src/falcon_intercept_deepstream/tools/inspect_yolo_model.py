#!/usr/bin/env python3
"""Print deployment-relevant metadata from an Ultralytics YOLO checkpoint."""

import argparse
import hashlib
import json
from pathlib import Path

from ultralytics import YOLO


def json_safe(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=Path)
    arguments = parser.parse_args()

    checkpoint = arguments.checkpoint.expanduser().resolve()
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    model = YOLO(str(checkpoint))
    checkpoint_data = getattr(model, "ckpt", {}) or {}
    train_args = checkpoint_data.get("train_args", {}) or {}
    names = getattr(model, "names", None) or getattr(model.model, "names", {})
    if isinstance(names, dict):
        ordered_names = [str(names[index]) for index in sorted(names)]
    else:
        ordered_names = [str(name) for name in names]

    metadata = {
        "path": str(checkpoint),
        "bytes": checkpoint.stat().st_size,
        "sha256": digest,
        "task": getattr(model, "task", None),
        "model_class": type(model.model).__name__,
        "class_count": len(ordered_names),
        "class_names": ordered_names,
        "stride": getattr(model.model, "stride", None),
        "training_imgsz": train_args.get("imgsz"),
        "training_epochs": train_args.get("epochs"),
        "training_batch": train_args.get("batch"),
        "training_data": train_args.get("data"),
        "ultralytics_version": checkpoint_data.get("version"),
        "checkpoint_date": checkpoint_data.get("date"),
        "best_fitness": checkpoint_data.get("best_fitness"),
    }
    print(json.dumps(json_safe(metadata), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
